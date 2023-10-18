# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Abstract base classes for KSPDG environment classes

import krpc
import logging
import gymnasium as gym

from abc import ABC, abstractmethod
from threading import Thread
from typing import Tuple, Dict
from numpy.typing import ArrayLike

from kspdg.utils.loggers import create_logger

class KSPDGBaseEnv(ABC, gym.Env):
    """ Abstract base class for all KSPDG gym environments
    """

    def __init__(self, debug:bool=False) -> None:
        """
        Args:
            debug : bool
                if true, set logging config to debug
        """
        
        super().__init__()

        # create logger to store environment metrics
        self.logger = create_logger(
            __name__, 
            stream_log_level=logging.DEBUG if debug else logging.INFO)

    def reset(self) -> Tuple[ArrayLike, Dict]:
        """Restart env episode managing

        High-level reset process involves:
            + managing connection to KSP game via kRPC server
            + attributing vessels in the game to agents and/or bots
            + restarting episode performance metrics
            + managing connections to threads that continually run
            bots in the episode and check for termination conditions
            + return initial observation and info about environment

        Returns:
            obs : ArrayLike
                observation of environment (e.g. sample of a gym space)
            info : Dict
                general information about environment not directly used for action
                selection (e.g. performance metrics for episode)
        """

        # connect to KRPC server and load mission save file 
        self._connect_and_load_on_reset()

        # reset initial vessel configuration
        self._reset_vessels()

        # reset performance metrics
        self._reset_episode_metrics()

        # start process for internal bots' policies
        self._start_bot_threads()

        # start thread for checking episode termination
        self._start_episode_termination_thread()

        # package observation and performance metric info for return
        obs = self.get_observation()
        info = self.get_info(obs, False)

        return obs, info

    def _connect_and_load_on_reset(self) -> None:
        """Connect to KRPC server and load mission save file 
        """

        # remove prior connection if it exists
        if hasattr(self, 'conn'):
            self.close()

        # establish krpc connect to send remote commands
        self.conn = krpc.connect(name='kspdg_env')
        self.logger.info("Connected to kRPC server")

        # Load save file from start of mission scenario
        self.conn.space_center.load(self.loadfile)

    @abstractmethod
    def _reset_vessels(self) -> None:
        """ Define vessel attirbutes and walkthrough initial configuration process
        """
        raise NotImplementedError()

    @abstractmethod
    def _reset_episode_metrics(self) -> None:
        """ Reset attributes that track performance throughout episode
        """
        raise NotImplementedError()

    @abstractmethod
    def _start_bot_threads(self) -> None:
        """ Start parallel thread(s) that continually run bot agent policies
        """
        raise NotImplementedError()

    def _start_episode_termination_thread(self) -> None:
        """ Start thread to continually check episode termination conditions
        """
        self.is_episode_done = False
        self.stop_episode_termination_thread = False

        if hasattr(self, "episode_termination_thread"):
            if self.episode_termination_thread.is_alive():
                raise ConnectionError("episode_termination_thread is already running."+ 
                    " Close and join episode_termination_thread before restarting")

        self.episode_termination_thread = Thread(target=self.enforce_episode_termination)
        self.episode_termination_thread.start()

    @abstractmethod
    def enforce_episode_termination(self) -> None:
        """ determine if episode termination criteria are met and raise flags if so
        """
        raise NotImplementedError()

    @abstractmethod
    def get_observation(self) -> ArrayLike:
        """ return current observation of environment

        Returns:
            observation : ArrayLike
                current observation of environment formatted as array-like object
        """
        raise NotImplementedError

    def get_info(self, observation: ArrayLike, done: bool) -> Dict:
        """ general info about environment state not directly intended for action selection 
        
        Example: cumulative performance metrics for episode other than raw reward signal

        Args:
            observation : ArrayLike
                current / most recent observation of environment
            done : bool
                True if last step of episode
        """
        return {"is_episode_done": done}
