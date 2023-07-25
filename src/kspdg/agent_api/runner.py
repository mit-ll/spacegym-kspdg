# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import time
import numpy as np
import multiprocessing as mp

from abc import ABC, abstractmethod
from typing import Dict

from kspdg.base_envs import KSPDGBaseEnv

DEFAULT_RUNNER_TIMEOUT = 600 #[s]
DEFAULT_ACTION_ROLLOUT_TIME_HORIZON = 5.0 # [s]

class BaseAgentEnvRunner(ABC):
    def __init__(self, 
        env_cls:type[KSPDGBaseEnv], 
        env_kwargs:Dict, 
        runner_timeout:float=DEFAULT_RUNNER_TIMEOUT, 
        action_rollout_time_horizon:float=DEFAULT_ACTION_ROLLOUT_TIME_HORIZON):
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
        """
        self.env_cls = env_cls
        self.env_kwargs = env_kwargs
        self.action_rollout_time_horizon = action_rollout_time_horizon
        self.runner_timeout = runner_timeout