# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import time
import logging
import multiprocessing as mp

from abc import ABC, abstractmethod
from typing import Dict

from kspdg.base_envs import KSPDGBaseEnv
from kspdg.utils.loggers import create_logger
from kspdg.agent_api.ksp_interface import ksp_interface_loop

DEFAULT_RUNNER_TIMEOUT = 600 #[s]
DEFAULT_ACTION_ROLLOUT_TIME_HORIZON = 5.0 # [s]

class BaseAgentEnvRunner(ABC):
    def __init__(self, 
        env_cls:type[KSPDGBaseEnv], 
        env_kwargs:Dict, 
        runner_timeout:float=DEFAULT_RUNNER_TIMEOUT, 
        action_rollout_time_horizon:float=DEFAULT_ACTION_ROLLOUT_TIME_HORIZON,
        debug:bool=False):
        """ instantiates agent-environment pair and brokers communication between processes

        Even though KSBDGBaseEnv objects are Gym (Gymnasium) Environments with the standard API
        (e.g. step(), reset(), observation_space, action_space, etc.), they have several characteristics
        that necessitate this additional class to pair an agent to an environment
            1. A kRPC connection is required to send commands and get observations from KSP. This class 
                helps manage that connection
            2. KSBDG environments run in non-blocking fashion; i.e. environment simulation time
                marches forward in between calls to step(). This is a key difference from traditional
                Gym environments. Therefore it is desirable and necessary to have a separate process 
                running in parallel for your agent to compute actions. This class manages those
                processes

        Args:
            env_cls : type[KSPDGBaseEnv]
                class of (not instance of) KSPDG environment to run agent within
            env_kwargs : dict
                keyword args to be passed when instantiating environment class
            runner_timeout : float
                total time to run agent-environment pair
            action_rollout_time_horizon : float
                amount of time to allocate for an action to rollout in the environment
                before a new action is queried from the agent
            debug : bool
                if true, set logging level to debug
        """
        
        self.env_cls = env_cls
        self.env_kwargs = env_kwargs
        self.action_rollout_time_horizon = action_rollout_time_horizon
        self.runner_timeout = runner_timeout
        self.debug = debug

        # create logger for Agent-Env Runner
        # (distinct from environment logger)
        self.logger = create_logger(
            __name__, 
            stream_log_level=logging.DEBUG if self.debug else logging.INFO)

    def run(self):
        """start environment-interface process and agent policy loop"""

        # create ways for processes to talk to each other
        self.termination_event = mp.Event()
        self.observation_query_event = mp.Event()
        self.obs_conn_recv, obs_conn_send = mp.Pipe(duplex=False)
        act_conn_recv, self.act_conn_send = mp.Pipe(duplex=False)

        # create parallel process to run KSPDG environment with 
        # interface for agent interaction, return information
        return_dict = mp.Manager().dict()
        self.env_interface_process = mp.Process(
            target=ksp_interface_loop, 
            args=(
                self.env_cls, 
                self.env_kwargs,
                obs_conn_send, 
                act_conn_recv, 
                self.termination_event, 
                self.observation_query_event,
                return_dict,
                self.debug
                )
        )
        self.env_interface_process.start()

        # start agent policy loop that computes actions
        # given observations from environment
        self.policy_loop()

        # cleanup
        self.stop_agent()

        # return info from environment
        return dict(return_dict)
    
    def __del__(self):
        self.stop_agent()

    def stop_agent(self):

        # send termination event and join processes
        self.termination_event.set()
        self.env_interface_process.join()

    def policy_loop(self):
        """ this is the agent's "main loop" that computes/infers actions based on observations
        """

        # track start time so we can enforce a timeout
        policy_loop_start = time.time()

        while not self.termination_event.is_set():

            # request/receive observation from environment
            self.logger.debug("requesting new environment observation")
            self.observation_query_event.set()
            if self.obs_conn_recv.poll(timeout=20.0):
                try:
                    observation = self.obs_conn_recv.recv()
                except EOFError:
                    self.logger.info("environment closed, terminating agent...")
                    self.termination_event.set()
                    break
            else:
                self.logger.info("non-responsive executor, terminating agent...")
                self.termination_event.set()
                break

            # compute agent-specific action and send to env interface process
            action = self.get_action(observation=observation)
            self.act_conn_send.send(action)

            # allow current schedule to be executed for fixed time horizon
            self.logger.debug("sleeping {:.2f} sec to allow action rollout".format(self.action_rollout_time_horizon))
            time.sleep(self.action_rollout_time_horizon)

            # check for agent timeout
            if time.time() - policy_loop_start > self.runner_timeout:
                self.termination_event.set()
                self.logger.info("\n~~~AGENT TIMEOUT REACHED~~~\n")
                break

        # cleanup agent
        self.stop_agent()

    @abstractmethod
    def get_action(self, observation):
        raise NotImplementedError("Must be implemented by child class")