# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import time
import logging
import multiprocessing as mp
import math

from typing import Dict

from kspdg.base_envs import KSPDGBaseEnv
from kspdg.agent_api.base_agent import KSPDGBaseAgent
from kspdg.utils.loggers import create_logger
from kspdg.agent_api.ksp_interface import ksp_interface_loop
from kspdg.utils.plotters import run_dpg_plotter


class AgentEnvRunner():

    OBSERVATION_POLL_TIMEOUT = 20.0  #[s]
    ENVIRONMENT_ACTIVATION_TIMEOUT = 20.0  #[s]

    def __init__(self, 
        agent:KSPDGBaseAgent,
        env_cls:type[KSPDGBaseEnv], 
        env_kwargs:Dict, 
        runner_timeout:float=None, 
        debug:bool=False,
        enable_plots:bool=False):
        """ instantiates agent-environment pair and brokers communication between processes

        Even though KSPDGBaseEnv objects are Gym (Gymnasium) Environments with the standard API
        (e.g. step(), reset(), observation_space, action_space, etc.), they have several characteristics
        that necessitate this additional class to pair an agent to an environment
            1. A kRPC connection is required to send commands and get observations from KSP. This class 
                helps manage that connection
            2. KSPDG environments run in non-blocking fashion; i.e. environment simulation time
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
                total time to run agent-environment pair,
                if None, wait for environment done
            debug : bool
                if true, set logging level to debug
            enable_plots : bool
                if true, real-time plots will be generated and displayed using DreamPyGui
        """
        
        self.agent = agent
        self.env_cls = env_cls
        self.env_kwargs = env_kwargs
        self.runner_timeout = runner_timeout
        self.debug = debug

        # plotter process objects
        self.enable_plots = enable_plots
        self._plot_proc = None
        self._plot_q = None
        self._plot_stop_evt = None

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

        # start plotter process
        if self.enable_plots:
            self._start_plotter()

        try: 
            # start agent policy loop that computes actions
            # given observations from environment
            self.policy_loop()

        finally:
            # cleanup
            self.stop_agent()
            if self.enable_plots:
                self._stop_plotter()

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
            if self.debug:
                obs_req_time = time.time()
                self.logger.debug("Requesting new environment observation")
            self.observation_query_event.set()
            if self.obs_conn_recv.poll(timeout=self.OBSERVATION_POLL_TIMEOUT):
                try:
                    observation = self.obs_conn_recv.recv()
                    if observation is None and self.termination_event.is_set():
                        self.logger.info("Termination event set during observation request, terminating agent...")
                        break
                except EOFError:
                    self.logger.info("Environment closed, terminating agent...")
                    self.termination_event.set()
                    break

                # PLOTTER DEMO: still plotting a sine wave here; switch to obs when ready
                if self.enable_plots:
                    t = time.time() - policy_loop_start
                    y = math.sin(2.0 * math.pi * 0.5 * t)

                    # Non-blocking put with drop-old behavior
                    if self._plot_q is not None:
                        if self._plot_q.full():
                            try:
                                self._plot_q.get_nowait()  # drop old
                            except Exception:
                                pass
                        try:
                            self._plot_q.put_nowait((t, y))
                        except Exception:
                            pass
            else:
                self.logger.info("Non-responsive environment, terminating agent...")
                self.termination_event.set()
                break
            if self.debug:
                self.logger.debug(f"Environment observation received after {time.time()-obs_req_time:.5g} sec")

            # compute agent-specific action and send to env interface process
            action = self.agent.get_action(observation=observation)

            # Agent can choose to not change or update it's current action
            # by returning a value of None
            # Only update the action to be rolled out in the environment
            # if agent returns non-None action
            if action is not None:
                self.act_conn_send.send(action)

            # check for agent timeout
            if self.runner_timeout is not None:
                if time.time() - policy_loop_start > self.runner_timeout:
                    self.termination_event.set()
                    self.logger.info("\n~~~AGENT TIMEOUT REACHED~~~\n")
                    break

        # cleanup agent
        self.stop_agent()

    def _start_plotter(self):
        self._plot_q = mp.Queue(maxsize=1)           # coalesce to newest
        self._plot_stop_evt = mp.Event()
        self._plot_proc = mp.Process(
            target=run_dpg_plotter,
            args=(self._plot_q, self._plot_stop_evt),
            kwargs=dict(
                title="kspdg – Live Telemetry",
                fps=30,
                history_sec=10.0,
                ingest_cap_hz=240,
                show_sine_if_no_data=True,  # shows sine until first obs
            ),
            daemon=True,  # dies with parent if parent exits
        )
        self._plot_proc.start()

    def _stop_plotter(self, timeout=2.0):
        if self._plot_stop_evt is not None:
            self._plot_stop_evt.set()
        if self._plot_proc is not None:
            self._plot_proc.join(timeout=timeout)
        self._plot_proc = None
        self._plot_q = None
        self._plot_stop_evt = None