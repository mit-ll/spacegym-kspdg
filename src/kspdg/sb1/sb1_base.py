# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import gymnasium as gym
import numpy as np

from typing import List, Dict
from copy import deepcopy

import kspdg.utils.utils as U
from kspdg.pe1.pe1_base import PursuitEvadeGroup1Env

DEFAULT_EPISODE_TIMEOUT = 240.0 # [sec]
DEFAULT_TARGET_VIEWING_DISTANCE = 100.0 # [m]
DEFAULT_REWARD_DECAY_COEF = 1e-5 # [1/m^2] 

class SunBlockingGroup1Env(PursuitEvadeGroup1Env):
    '''
    Base environment for 1v1 sun-blocking orbital scenarios, which
    are direct variants of the pursuit-evasion environments
    
    All inherited classes share the following
        - Agent controls the pursuer and evader has a scripted policy (although specific policy varies)
        - Pursuer and evader vehicles are identical and are meant to be the same through all
        inherited scenarios (although there is not a current enforcement of this)
        - Observation and Action spaces shared between all child envs
    '''

    # mission loadfile names for variou initial condition
    LOADFILE_I1 = "sb1_i1_init"
    LOADFILE_I2 = "sb1_i2_init"
    LOADFILE_I3 = "sb1_i3_init"
    LOADFILE_I4 = "sb1_i4_init"
    LOADFILE_I5 = "sb1_i5_init"

    PARAMS = deepcopy(PursuitEvadeGroup1Env.PARAMS)

    # info metric parameters
    # PARAMS.INFO.K_CUM_REWARD = "cumulative_reward"
    PARAMS.INFO.K_MAX_REWARD = "max_reward"
    PARAMS.INFO.K_MIN_REWARD = "min_reward"

    def __init__(self, loadfile:str, 
        episode_timeout:float = DEFAULT_EPISODE_TIMEOUT, 
        target_viewing_distance:float = DEFAULT_TARGET_VIEWING_DISTANCE,
        reward_decay_coef:float = DEFAULT_REWARD_DECAY_COEF,
        **kwargs):
        """
        Args:
            episode_timeout : float
                max length of episode [sec]
            capture_dist : float
                distance at which evader is considered captured [m]
        """

        super().__init__(
            loadfile=loadfile,
            episode_timeout=episode_timeout,
            capture_dist=None,
            **kwargs
        )

        assert target_viewing_distance > 0.0
        assert reward_decay_coef > 0.0
        self.target_viewing_distance = target_viewing_distance
        self.reward_decay_coef = reward_decay_coef

        # overwrite the pe1 observation space 
        # to include sun position information
        # (see get_observation for mapping)
        self.observation_space = gym.spaces.Box(
            low = np.concatenate((np.zeros(3), -np.inf*np.ones(15))),
            high = np.inf * np.ones(18)
        )
        
        # don't call reset. This allows instantiation and partial testing
        # without connecting to krpc server

    def _reset_episode_metrics(self) -> None:
        """ Reset attributes that track proximity, timing, and propellant use metrics
        """
        super()._reset_episode_metrics()

        self.cum_reward = 0.0
        self.prev_reward = 0.0
        self.prev_time = self.vesPursue.met
        self.min_reward = np.inf
        self.max_reward = -np.inf
    
    def get_reward(self, *args) -> float:
        """ Compute reward value

        Reward is a function of evader-pursuer-sun angle and pursuer-evader distance
        
        Returns:
            rew : float
                reward at current step
            args
                unused dummy var included to match signature of calls to get_reward in parent class
        """

        # get evader position, distance, and unit vector relative to pursuer
        p_vesE_vesP__lhpbody = self.vesEvade.position(self.vesPursue.reference_frame)
        d_vesE_vesP = np.linalg.norm(p_vesE_vesP__lhpbody)
        u_vesE_vesP__lhpbody = p_vesE_vesP__lhpbody/d_vesE_vesP

        # get sun unit vector relative to pursuer
        p_sun_vesP__lhpbody = self.conn.space_center.bodies['Sun'].position(
            self.vesPursue.reference_frame)
        d_sun_vesP = np.linalg.norm(p_sun_vesP__lhpbody)
        u_sun_vesP__lhpbody = p_sun_vesP__lhpbody/d_sun_vesP

        # compute reward. See sb_objective_plot.py for intuition
        # about reward surface shape
        rew = -np.dot(u_vesE_vesP__lhpbody, u_sun_vesP__lhpbody)
        rew *= np.exp(-self.reward_decay_coef * (d_vesE_vesP - self.target_viewing_distance)**2)

        return rew

    def get_info(self, observation: List, done: bool) -> Dict:
        """compute performance metrics
        Args:
            observation : List
                current / most recent observation of environment
            done : bool
                True if last step of episode
        """

        # call the "grandparent" info method, skipping the 
        # parent info in PursuitEvadeGroup1Env because
        # there is a lot of bloat in the parent method that is not needed
        info = super(PursuitEvadeGroup1Env, self).get_info(observation=observation, done=done)

        # sun-blocking reward metrics
        cur_reward = self.get_reward()
        cur_time = observation[self.PARAMS.OBSERVATION.I_MET]
        if cur_reward < self.min_reward:
            self.min_reward = cur_reward
        if cur_reward > self.max_reward:
            self.max_reward = cur_reward
        info[self.PARAMS.INFO.K_MIN_REWARD] = self.min_reward
        info[self.PARAMS.INFO.K_MAX_REWARD] = self.max_reward

        # fuel usage 
        info[self.PARAMS.INFO.K_PURSUER_FUEL_USAGE] = self.pursuer_init_mass - self.vesPursue.mass
        info[self.PARAMS.INFO.K_EVADER_FUEL_USAGE] = self.evader_init_mass - self.vesEvade.mass

        # weighted score is trapezoid approximation of integral of reward function
        self.cum_reward += 0.5*(cur_reward + self.prev_reward)*(cur_time - self.prev_time)
        info[self.PARAMS.INFO.K_WEIGHTED_SCORE] = self.cum_reward

        # store reward and time value for trapezoid approx in next step
        self.prev_reward = cur_reward
        self.prev_time = cur_time

        return info

    def get_observation(self):
        """ return observation of pursuit and evader vessels from referee ref frame

        Returns:
            obs : list
                [0] : mission elapsed time [s]
                [1] : current vehicle (pursuer) mass [kg]
                [2] : current vehicle (pursuer) propellant  (mono prop) [kg]
                [3:6] : pursuer position wrt CB in right-hand CBCI coords [m]
                [6:9] : pursuer velocity wrt CB in right-hand CBCI coords [m/s]
                [9:12] : evader position wrt CB in right-hand CBCI coords [m]
                [12:15] : evader velocity wrt CB in right-hand CBCI coords [m/s]
                [15:18] : pursuer's sun-pointing unit vector in right-hand CBCI coords [-]

        Ref: 
            - CBCI stands for celestial-body-centered inertial which is a coralary to ECI coords
            (see notation: https://github.com/mit-ll/spacegym-kspdg#code-notation)
            - KSP's body-centered inertial reference frame is left-handed
            (see https://krpc.github.io/krpc/python/api/space-center/celestial-body.html#SpaceCenter.CelestialBody.non_rotating_reference_frame)

        """

        obs = super().get_observation()

        # get sun position relative to pursuer
        p_sun_vesP__lhcbci = np.array(self.conn.space_center.bodies['Sun'].position(
            self.vesPursue.orbit.body.non_rotating_reference_frame))
        
        # convert to right-handed CBCI coords
        p_sun_vesP__rhcbci = U.convert_lhcbci_to_rhcbci(p_sun_vesP__lhcbci)
        d_sun_vesP = np.linalg.norm(p_sun_vesP__rhcbci)
        u_sun_vesP__rhcbci = p_sun_vesP__rhcbci/d_sun_vesP

        # encode into observation
        obs.extend(u_sun_vesP__rhcbci)

        return obs
    
    @classmethod
    def observation_list_to_dict(cls, obs_list):
        """convert observation from list to dict"""
        raise NotImplementedError
    
    @classmethod
    def observation_dict_to_list(cls, obs_dict):
        """convert observation from list to dict"""
        raise NotImplementedError


