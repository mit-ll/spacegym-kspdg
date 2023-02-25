# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import gymnasium as gym
import numpy as np

from typing import List, Dict

import kspdg.utils.utils as U
from kspdg.pe1.pe1_base import PursuitEvadeGroup1Env

DEFAULT_EPISODE_TIMEOUT = 240.0 # [sec]
DEFAULT_TARGET_VIEWING_DISTANCE = 50.0 # [m]
DEFAULT_REWARD_DECAY_COEF = 0.001 # [1/m^2] 

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

        # TODO: customize this for sun-blocking beyond pure pursuit-evade
        super()._reset_episode_metrics()
    
    def get_reward(self) -> float:
        """ Compute reward value

        Reward is a function of evader-pursuer-sun angle and pursuer-evader distance
        
        Returns:
            rew : float
                reward at current step
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

        # TODO: customize this for sun-blocking beyond pure pursuit-evade
        return super().get_info(observation=observation, done=done)

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



