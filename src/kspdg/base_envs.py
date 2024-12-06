# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Abstract base classes for KSPDG environment classes

import time
import krpc
import logging
import gymnasium as gym
import numpy as np

from abc import ABC, abstractmethod
from threading import Thread
from typing import Tuple, Dict, List
from numpy.typing import ArrayLike
from types import SimpleNamespace

from kspdg.utils.loggers import create_logger
import kspdg.utils.utils as U

class KSPDGBaseEnv(ABC, gym.Env):
    """ Abstract base class for all KSPDG gym environments
    """

    BOT_HEALTHY_FLAG = 0
    BOT_ERROR_FLAG = 1

    def __init__(self, debug:bool=False) -> None:
        """
        Args:
            debug : bool
                if true, set logging config to debug
        """
        
        super().__init__()

        # create logger to store environment metrics
        # use environment child class name for more specificity on origin of log statements
        self.debug = debug
        self.logger = create_logger(
            self.__class__.__name__, 
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
    
    def _start_bot_threads(self) -> None:
        """ Start parallel thread to continuously execute lady-guard policy
        """

        self.stop_bot_thread = False
        self.bot_thread_status = self.BOT_HEALTHY_FLAG

        # check that thread does not exist or is not running
        if hasattr(self, "bot_thread"):
            if self.bot_thread.is_alive():
                raise ConnectionError("bot_thread is already running."+ 
                    " Close and join bot_thread before restarting")

        self.bot_thread = Thread(target=self.bot_policy, name='BotThread')
        self.bot_thread.start()

    def _start_episode_termination_thread(self) -> None:
        """ Start thread to continually check episode termination conditions
        """
        self.is_episode_done = False
        self.stop_episode_termination_thread = False

        if hasattr(self, "episode_termination_thread"):
            if self.episode_termination_thread.is_alive():
                raise ConnectionError("episode_termination_thread is already running."+ 
                    " Close and join episode_termination_thread before restarting")

        self.episode_termination_thread = Thread(target=self.enforce_episode_termination, name="TerminationThread")
        self.episode_termination_thread.start()

    def close(self):
        """ Gracefully handle thread closure and krpc connection closure"""

        # handle evasive maneuvering thread
        self.stop_bot_thread = True
        self.bot_thread.join()

        # handle episode termination thread
        self.stop_episode_termination_thread = True
        self.episode_termination_thread.join()

        # close connection to krpc server
        self.conn.close()

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

    def convert_rhcbci_to_rhvbody(self, v__rhcbci: List[float], vessel) -> List[float]:
        '''Converts vector in right-handed celestial-body-centered-inertial frame to 
        right-oriented body frame of specified vessel

        Args:
            v__rhcbci : List[float]
                3-vector represented in celestial-body-centered-inertial fram 
                (similar to ECI coords, but we aren't necessarily orbitiing Earth)
            vessel : kRPC.Vessel
                kRPC Vessel object for which conversion to right-hand body frame is to be done
        
        Returns
            v__rhvbody : List[float]
                3-vector vector represented in agent vessel's right-hadded body coords (forward, right, down)
        
        Ref:
            Left-handed vessel body system: 
                https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
            KSP's body-centered inertial reference frame is left-handed
            (see https://krpc.github.io/krpc/python/api/space-center/celestial-body.html#SpaceCenter.CelestialBody.non_rotating_reference_frame)
            Right-handed ECI system: Vallado, 3rd Edition Sec 3.3
        '''

        # convert right-handed CBCI coords to left-handed CBCI
        v__lhcbci = U.convert_rhcbci_to_lhcbci(v__rhcbci=v__rhcbci)

        # convert left-handed CBCI to left-handed vessel body coords
        # ref: https://krpc.github.io/krpc/python/api/space-center/space-center.html#SpaceCenter.transform_direction
        # ref: https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
        v__lhvbody = list(self.conn.space_center.transform_direction(
            direction = tuple(v__lhcbci),
            from_ = vessel.orbit.body.non_rotating_reference_frame,
            to = vessel.reference_frame
        ))

        # convert left-handed body coords (right, forward, down) to right-handed body coords (forward, right, down)
        v__rhvbody = U.convert_lhbody_to_rhbody(v__lhbody=v__lhvbody)

        return v__rhvbody
    
    def convert_rhntw_to_rhvbody(self, v__rhntw: List[float], vessel) -> List[float]:
        '''Converts vector in right-handed NTW frame to agent vessel right-oriented body frame
        Args:
            v__ntw : List[float]
                3-vector represented in orbital NTW coords
        
        Returns
            v__rhvbody : List[float]
                3-vector vector represented in agent vessel's right-hadded body coords (forward, right, down)
            vessel : kRPC.Vessel
                kRPC Vessel object for which conversion to right-hand body frame is to be done
        
        Ref:
            Left-handed vessel body system: 
                https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
            Right-handed NTW system: Vallado, 3rd Edition Sec 3.3.3
        '''

        # convert right-handed NTW coords to left-handed NTW
        v__lhntw = U.convert_rhntw_to_lhntw(v__rhntw=v__rhntw)

        # convert left-handed NTW to left-handed vessel body coords
        # ref: https://krpc.github.io/krpc/python/api/space-center/space-center.html#SpaceCenter.transform_direction
        # ref: https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
        v__lhvbody = list(self.conn.space_center.transform_direction(
            direction = tuple(v__lhntw),
            from_ = vessel.orbital_reference_frame,
            to = vessel.reference_frame
        ))

        # convert left-handed body coords (right, forward, down) to right-handed body coords (forward, right, down)
        v__rhvbody = U.convert_lhbody_to_rhbody(v__lhbody=v__lhvbody)

        return v__rhvbody

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
    def bot_policy(self) -> None:
        """ Behavioral policy to be continuously run for pre-scripted bots 
        (e.g. evader in pe1, lady & guard in lbg1)
        """
        raise NotImplementedError()

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

class Group1BaseEnv(KSPDGBaseEnv):
    """Abstract base class for all group-1 environments

    Group 1 environments all share a common action space,
    but not a common observation space or reward structure
    """

    PARAMS = SimpleNamespace()
    PARAMS.ACTION = SimpleNamespace()
    PARAMS.OBSERVATION = SimpleNamespace()
    PARAMS.INFO = SimpleNamespace()

    # action space params
    PARAMS.ACTION.K_BURN_VEC = "burn_vec"   # 4-tuple: 3D vector [0:3] and burn duration [3]
    PARAMS.ACTION.K_REF_FRAME = "ref_frame" # discrete: reference frame in which burn vector is defined
    PARAMS.ACTION.K_VEC_TYPE = "vec_type"   # discrete: what does the vector represent: throttle, thrust, acceleration

    # info space params
    PARAMS.INFO.K_WEIGHTED_SCORE = "weighted_score"

    def __init__(self, **kwargs) -> None:
        
        super().__init__(**kwargs)

        # define common action space 
        # see vessel_step() for mapping
        self.action_space = gym.spaces.Dict(
            {
                self.PARAMS.ACTION.K_BURN_VEC: gym.spaces.Box(
                    low=np.array([-1.0, -1.0, -1.0, 0.0]), 
                    high=np.array([1.0, 1.0, 1.0, 10.0])
                ),
                self.PARAMS.ACTION.K_REF_FRAME: gym.spaces.Discrete(3),
                self.PARAMS.ACTION.K_VEC_TYPE: gym.spaces.Discrete(3)
            }
        )

    def vessel_step(self, action: Dict, vesAgent):
        """ Apply propulsive burn for the specified vessel for specified time duration
        
        This is step function that is common across a subset of environments (e.g. pe1, lbg1, sb1)
        so abstracting to parent class to reduce redundancy

        Args:
            action : dict
                "burn_vec" :
                    4-tuple of burn vector in 3D and duration of burn
                    [0:3] - burn vector expressed in ref_frame coords. 
                    [3] - time duration to execute burn in seconds
                "vec_type" : int
                    description of what the burn vector represents
                    0: throttle vector - unitless vector in range -1 to 1 in each dimension
                        1.0 represents a maximum forward thrust in that dimension where as 
                        -1.0 represent a maximum reverse thrust in that dimension
                        inputing values outside the range [-1, 1] will not throw errors,
                        However the spacecraft will not be able to "overthrottle" in anyway
                    1: thrust - [N] force vector that spacecraft attempts to reproduce
                        inputing force vectors that are not achievable by the spacecraft
                        will not throw errors, but the spacecraft will not behave as commanded
                    Note: acceleration is not included as a vector type because it requires
                        vehicle mass to convert to thrust and then throttle.
                        The instantaneous vehicle mass does not have a top-level representation
                        in Group1BaseEnv and would need to be distinguished based on child classes
                        (e.g. self.vesPursue.mass or self.vesBandit.mass, etc). Furthermore,
                        the vessel mass is already included in the observation vector sent to the
                        agent object that is generating the action; therefore the agent can
                        more directly make this conversion from accel to thrust than this top-
                        level environment class
                "ref_frame" : int
                    designates the reference frame the burn vector is expressed in
                    0: rhvbody - right-handed body frame of agent vessel (forward, right, down)
                        see https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
                    1: rhcbci - right-handed celestial-body-centered inertial frame 
                        see https://github.com/mit-ll/spacegym-kspdg/tree/main#code-notation
                    2: rhntw - right-handed NTW frame (y-axis velocity aligned, z-axis is orbit normal, x-axis completes
                                right-handed system)
                        see https://github.com/mit-ll/spacegym-kspdg/tree/main#code-notation

            vesAgent : krpc.Vessel
                krpc Vessel object for vessel that is acting as the agent (as opposed to pre-scripted bot vessels)
        
        """
        
        vessel_step_start = time.time()
        self.logger.debug("Beginning vessel_step...")

        # check that bot thread has not errored
        assert self.bot_thread_status == self.BOT_HEALTHY_FLAG

        k_burn_vec = self.PARAMS.ACTION.K_BURN_VEC
        k_vec_type = self.PARAMS.ACTION.K_VEC_TYPE
        k_ref_frame = self.PARAMS.ACTION.K_REF_FRAME

        if isinstance(action, dict):

            # process the burn vector in body coords
            burn__rhvbody, burn_dur = self.get_burn__rhvbody(
                burn_vec=action[k_burn_vec], 
                ref_frame=action[k_ref_frame],
                vesAgent=vesAgent)

            # convert burn vector into throttle values along each body axes
            if k_vec_type not in action.keys() or action[k_vec_type] == 0:
                # burn vector already represents throttle
                thr__rhvbody = burn__rhvbody

            elif action[k_vec_type] == 1:

                # burn vector represents thrust in Newtons, use max thrusts to scale to throttle
                f_max_forward = self.agent_max_thrust_forward if burn__rhvbody[0] >= 0 else \
                    self.agent_max_thrust_reverse
                f_max_right = self.agent_max_thrust_right if burn__rhvbody[1] >= 0 else \
                    self.agent_max_thrust_left
                f_max_down = self.agent_max_thrust_down if burn__rhvbody[2] >= 0 else \
                    self.agent_max_thrust_up
                
                # convert to throttle
                thr__rhvbody = [
                    burn__rhvbody[0]/f_max_forward,
                    burn__rhvbody[1]/f_max_right,
                    burn__rhvbody[2]/f_max_down,
                ]
                
            else:
                raise ValueError(f"Unrecognized burn vector type: {action[k_vec_type]}")


        elif len(action) == 4:

            # Action as list is for backward compatability
            thr__rhvbody = [
                action[0],
                action[1],
                action[2]
            ]
            burn_dur = action[3]

        else:
            raise TypeError(f"Unrecognized action format: {action}")
        
        # set throttle values for agent vehicle
        vesAgent.control.forward = thr__rhvbody[0]
        vesAgent.control.right = thr__rhvbody[1]
        vesAgent.control.up = -thr__rhvbody[2]

        # execute maneuver for specified time, checking for end
        # conditions while you do
        timeout = time.time() + burn_dur
        while True: 
            if self.is_episode_done or time.time() > timeout:
                break

        # zero out thrusts
        vesAgent.control.forward = 0.0
        vesAgent.control.up = 0.0
        vesAgent.control.right = 0.0

        # get observation
        obs = self.get_observation()

        # compute performance metrics
        info = self.get_info(obs, self.is_episode_done)

        # display weighted score
        self.logger.info(f"CURRENT SCORE: {info[self.PARAMS.INFO.K_WEIGHTED_SCORE]}")

        # compute reward
        rew = self.get_reward(info, self.is_episode_done)

        self.logger.debug(f"vessel_step complete after {time.time()-vessel_step_start:.5g} sec")
        return obs, rew, self.is_episode_done, info

    def get_burn__rhvbody(self, burn_vec, ref_frame, vesAgent):
        """map the burn vec from the given reference frame into the right-hand vessel body frame
        
        Args:
            burn_vec : arraylike
                    4-tuple of burn vector in 3D and duration of burn
                    [0:3] - burn vector expressed in ref_frame coords. 
                    [3] - time duration to execute burn in seconds
            ref_frame : int
                    designates the reference frame the burn vector is expressed in
                    0: rhvbody - right-handed body frame of agent vessel (forward, right, down)
                        see https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
                    1: rhcbci - right-handed celestial-body-centered inertial frame 
                        see https://github.com/mit-ll/spacegym-kspdg/tree/main#code-notation
                    2: rhntw - right-handed NTW frame (y-axis velocity aligned, z-axis is orbit normal, x-axis completes
                                right-handed system)
                        see https://github.com/mit-ll/spacegym-kspdg/tree/main#code-notation
            vesAgent : krpc.Vessel
                krpc Vessel object for vessel that is acting as the agent (as opposed to pre-scripted bot vessels)

        Returns:
            burn__rhvbody : arraylike
                3-vector representing the burn vector in the vessel's right-handed body frame
            burn_dur : float
                duration of burn to be executed
        """
        assert len(burn_vec) == 4
            
        # extract burn vector components and map to
        # right-handed body coords of vessel (rhvbody)
        burn_dur = burn_vec[3]
        burn__rhvbody = None
        if ref_frame == 0:
            # rh-vessel-body coords
            # burn vector given in agent vessel's body coords
            burn__rhvbody = burn_vec[0:3]

        elif ref_frame == 1:
            # rhcbci coords
            # burn vector given in inertial celestial body centered coords
            burn__rhcbci = burn_vec[0:3]

            # convert to agent vessel body frame
            burn__rhvbody = self.convert_rhcbci_to_rhvbody(burn__rhcbci, vessel=vesAgent)

        elif ref_frame == 2:
            # rhntw coords
            burn__rhntw =  burn_vec[0:3]
            burn__rhvbody = self.convert_rhntw_to_rhvbody(burn__rhntw, vessel=vesAgent)

        else:
            raise ValueError(f"Unrecognized burn reference frame: {ref_frame}")
        
        return burn__rhvbody, burn_dur
