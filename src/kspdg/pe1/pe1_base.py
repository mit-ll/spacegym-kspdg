# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import time
import math
import gymnasium as gym
import numpy as np
import dearpygui.dearpygui as dpg

from types import SimpleNamespace
from typing import List, Dict
from collections import deque
from copy import deepcopy

import kspdg.utils.constants as C
import kspdg.utils.utils as U
from kspdg.base_envs import Group1BaseEnv

DEFAULT_EPISODE_TIMEOUT = 240.0 # [sec]
DEFAULT_CAPTURE_DIST = 5.0      # [m]

class PursuitEvadeGroup1Env(Group1BaseEnv):
    '''
    Base environment for 1v1 pursuit-evasion orbital scenarios
    
    All inherited classes share the following
        - Agent controls the pursuer and evader has a scripted policy (although specific policy varies)
        - Pursuer and evader vehicles are identical and are meant to be the same through all
        inherited scenarios (although there is not a current enforcement of this)
        - Observation and Action spaces shared between all child envs
    '''

    # mission loadfile names for variou initial condition
    LOADFILE_I1 = "pe1_i1_init"
    LOADFILE_I2 = "pe1_i2_init"
    LOADFILE_I3 = "pe1_i3_init"
    LOADFILE_I4 = "pe1_i4_init"

    # hard-coded, static parameters for pursuer vehicle
    # that are accessible yet constant (so shouldn't be
    # in observation which should really only be variable values)
    # Need for hard-coding rsc properties comes from the errors in 
    # krpc's handling of thruster objects.
    PARAMS = deepcopy(Group1BaseEnv.PARAMS)
    PARAMS.PURSUER = SimpleNamespace()
    PARAMS.EVADER = SimpleNamespace()
    PARAMS.PURSUER.RCS = SimpleNamespace()

    # observation space paramterization
    # [0] : mission elapsed time [s]
    # [1] : current vehicle (pursuer) mass [kg]
    # [2] : current vehicle (pursuer) propellant  (mono prop) [kg]
    # [3:6] : pursuer position wrt CB in right-hand CBCI coords [m]
    # [6:9] : pursuer velocity wrt CB in right-hand CBCI coords [m/s]
    # [9:12] : evader position wrt CB in right-hand CBCI coords [m]
    # [12:15] : evader velocity wrt CB in right-hand CBCI coords [m/s]
    PARAMS.OBSERVATION.LEN = 15
    PARAMS.OBSERVATION.K_MET = "mission_elapsed_time"
    PARAMS.OBSERVATION.I_MET = 0
    PARAMS.OBSERVATION.K_PURSUER_MASS = "pursuer_mass"
    PARAMS.OBSERVATION.I_PURSUER_MASS = 1
    PARAMS.OBSERVATION.K_PURSUER_PROP_MASS = "pursuer_propellant_mass" 
    PARAMS.OBSERVATION.I_PURSUER_PROP_MASS = 2
    PARAMS.OBSERVATION.K_PURSUER_PX = "posx_p_cb__rhcbci"
    PARAMS.OBSERVATION.I_PURSUER_PX = 3
    PARAMS.OBSERVATION.K_PURSUER_PY = "posy_p_cb__rhcbci"
    PARAMS.OBSERVATION.I_PURSUER_PY = 4
    PARAMS.OBSERVATION.K_PURSUER_PZ = "posz_p_cb__rhcbci"
    PARAMS.OBSERVATION.I_PURSUER_PZ = 5
    PARAMS.OBSERVATION.K_PURSUER_VX = "velx_p_cb__rhcbci"
    PARAMS.OBSERVATION.I_PURSUER_VX = 6
    PARAMS.OBSERVATION.K_PURSUER_VY = "vely_p_cb__rhcbci"
    PARAMS.OBSERVATION.I_PURSUER_VY = 7
    PARAMS.OBSERVATION.K_PURSUER_VZ = "velz_p_cb__rhcbci"
    PARAMS.OBSERVATION.I_PURSUER_VZ = 8
    PARAMS.OBSERVATION.K_EVADER_PX = "posx_e_cb__rhcbci" 
    PARAMS.OBSERVATION.I_EVADER_PX = 9
    PARAMS.OBSERVATION.K_EVADER_PY = "posy_e_cb__rhcbci" 
    PARAMS.OBSERVATION.I_EVADER_PY = 10
    PARAMS.OBSERVATION.K_EVADER_PZ = "posz_e_cb__rhcbci" 
    PARAMS.OBSERVATION.I_EVADER_PZ = 11
    PARAMS.OBSERVATION.K_EVADER_VX = "velx_e_cb__rhcbci"
    PARAMS.OBSERVATION.I_EVADER_VX = 12
    PARAMS.OBSERVATION.K_EVADER_VY = "vely_e_cb__rhcbci"
    PARAMS.OBSERVATION.I_EVADER_VY = 13
    PARAMS.OBSERVATION.K_EVADER_VZ = "velz_e_cb__rhcbci"
    PARAMS.OBSERVATION.I_EVADER_VZ = 14

    # info metric parameters
    PARAMS.INFO.K_CLOSEST_APPROACH = "closest_approach"
    PARAMS.INFO.K_CLOSEST_APPROACH_SPEED = "closest_approach_speed"
    PARAMS.INFO.K_CLOSEST_APPROACH_TIME = "closest_approach_time"
    PARAMS.INFO.K_CLOSEST_APPROACH_PURSUER_FUEL_USAGE = "closest_approach_pursuer_fuel_usage"
    # PARAMS.INFO.K_MIN_POSVEL_PRODUCT = "minimum_position_velocity_product"
    PARAMS.INFO.K_PURSUER_FUEL_USAGE = "pursuer_fuel_usage"
    PARAMS.INFO.K_EVADER_FUEL_USAGE = "evader_fuel_usage"

    # PARAMS.INFO.K_WEIGHTED_SCORE = "weighted_score"
    PARAMS.INFO.V_SCORE_DIST_SCALE = 0.1
    PARAMS.INFO.V_SCORE_DIST_WEIGHT = 2.0
    PARAMS.INFO.V_SCORE_SPEED_SCALE = 0.5
    PARAMS.INFO.V_SCORE_SPEED_WEIGHT = 1.5
    PARAMS.INFO.V_SCORE_PURSUER_FUEL_USAGE_SCALE = 0.1
    PARAMS.INFO.V_SCORE_PURSUER_FUEL_USAGE_WEIGHT = 1.25
    PARAMS.INFO.V_SCORE_TIME_SCALE = 0.01
    PARAMS.INFO.V_SCORE_TIME_WEIGHT = 1.0

    # Relative range at which evader should be controllable
    PARAMS.EVADER.CONTROL_RANGE = 2200  # [m]

    # Specific impulse assumes all RCS thrusters are identical RV-105
    # parts operating in vacuum
    PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE = 240 # [s]
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE = 1000 # [N]

    # Assumed number of thrusters creating thrust in each direction
    PARAMS.PURSUER.RCS.N_THRUSTERS_FORWARD = 8
    PARAMS.PURSUER.RCS.N_THRUSTERS_REVERSE = 8
    PARAMS.PURSUER.RCS.N_THRUSTERS_RIGHT = 4
    PARAMS.PURSUER.RCS.N_THRUSTERS_LEFT = 4
    PARAMS.PURSUER.RCS.N_THRUSTERS_UP = 4
    PARAMS.PURSUER.RCS.N_THRUSTERS_DOWN = 4

    # computed max thrust in each direction [N]
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_FORWARD = \
        PARAMS.PURSUER.RCS.N_THRUSTERS_FORWARD * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_REVERSE = \
        PARAMS.PURSUER.RCS.N_THRUSTERS_REVERSE * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_RIGHT = \
        PARAMS.PURSUER.RCS.N_THRUSTERS_RIGHT * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_LEFT = \
        PARAMS.PURSUER.RCS.N_THRUSTERS_LEFT * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_UP = \
        PARAMS.PURSUER.RCS.N_THRUSTERS_UP * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_DOWN = \
        PARAMS.PURSUER.RCS.N_THRUSTERS_DOWN * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE

    # computed maximum fuel consumption rate in each direction [kg/s]
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_FORWARD = \
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_FORWARD / (C.G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_REVERSE = \
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_REVERSE / (C.G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_RIGHT = \
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_RIGHT / (C.G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_LEFT = \
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_LEFT / (C.G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_UP = \
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_UP / (C.G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_DOWN = \
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_DOWN / (C.G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)


    def __init__(self, loadfile:str, 
        episode_timeout:float = DEFAULT_EPISODE_TIMEOUT, 
        capture_dist:float = DEFAULT_CAPTURE_DIST,
        **kwargs):
        """
        Args:
            episode_timeout : float
                max length of episode [sec]
            capture_dist : float
                distance at which evader is considered captured [m]
        """

        super().__init__(**kwargs)

        assert episode_timeout > 0
        if capture_dist is not None:
            assert capture_dist > 0
        self.episode_timeout = episode_timeout
        self.capture_dist = capture_dist

        # establish load file for environment resets
        self.loadfile = loadfile

        # establish observation space (see get_observation() for mapping)
        self.observation_space = gym.spaces.Box(
            low = np.concatenate((np.zeros(3), -np.inf*np.ones(12))),
            high = np.inf * np.ones(15)
        )

        # define max thrust along each axes
        self.agent_max_thrust_forward = self.PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_FORWARD
        self.agent_max_thrust_reverse = self.PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_REVERSE
        self.agent_max_thrust_right = self.PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_RIGHT
        self.agent_max_thrust_left = self.PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_LEFT
        self.agent_max_thrust_down = self.PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_DOWN
        self.agent_max_thrust_up =  self.PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_UP
        
        # don't call reset. This allows instantiation and partial testing
        # without connecting to krpc server

    def _reset_vessels(self):
        """"""
        # get vessel objects
        self.vesEvade, self.vesPursue = self.conn.space_center.vessels[:3]

        # Set the pursuer as the the active (i.e. human-controlled) vessel
        # and target evader
        self.conn.space_center.active_vessel = self.vesPursue
        self.conn.space_center.target_vessel = self.vesEvade
        self.logger.info("Changing active vehicle...")
        time.sleep(1)   # give time to re-orient

        # set the evader to stability assist and orient in orbit-normal direction
        # orient pursuer in target-pointing direction

        self.logger.info("Activating Pursuer SAS, RCS and orienting to Evader...")
        self.vesPursue.control.sas = True
        time.sleep(0.1)   # give time to re-orient
        self.vesPursue.control.sas_mode = self.vesPursue.control.sas_mode.target
        time.sleep(2)   # give time to re-orient

        # activate RCS thrusters
        self.vesPursue.control.rcs = True

    def _reset_episode_metrics(self) -> None:
        """ Reset attributes that track proximity, timing, and propellant use metrics
        """

        self.min_dist = np.inf
        self.min_dist_time = 0.0
        self.min_dist_speed = np.inf
        self.min_dist_pursuer_fuel_usage = np.inf
        # self.min_posvel_prod = np.inf
        self.pursuer_init_mass = self.vesPursue.mass
        self.evader_init_mass = self.vesEvade.mass

    def step(self, action):
        """Apply thrust for specified time duration"""
        return self.vessel_step(action=action, vesAgent=self.vesPursue)
    
    def get_weighted_score(self, dist: float, speed: float, time: float, fuel: float):
        """ Compute a scaled, weighted sum of scoring metrics
        Args:
            dist : float
                relative distance between pursuer and evader [m]
            speed : float
                relative speed between pursuer and evader [m/s]
            time : float
                elapsed time [s]
            fuel : kg
                fuel used by pursuer spacecraft [kg]
        Returns:
            score : float
                weighted score
        """

        score = (
            (self.PARAMS.INFO.V_SCORE_DIST_SCALE * dist) ** self.PARAMS.INFO.V_SCORE_DIST_WEIGHT + 
            (self.PARAMS.INFO.V_SCORE_SPEED_SCALE * speed) ** self.PARAMS.INFO.V_SCORE_SPEED_WEIGHT + 
            (self.PARAMS.INFO.V_SCORE_PURSUER_FUEL_USAGE_SCALE * fuel) ** self.PARAMS.INFO.V_SCORE_PURSUER_FUEL_USAGE_WEIGHT + 
            (self.PARAMS.INFO.V_SCORE_TIME_SCALE * time) ** self.PARAMS.INFO.V_SCORE_TIME_WEIGHT 
        )

        return score
    
    def get_reward(self, info: Dict, done: bool) -> float:
        """ Compute reward value as negative of weighted score
        Args:
            info : Dict
                information dictionary of performance metrics
            done : bool
                True if last step of episode
        Returns:
            rew : float
                reward at current step
        """
        if done:
            return -info[self.PARAMS.INFO.K_WEIGHTED_SCORE]
        else:
            return 0.0

    def get_info(self, observation: List, done: bool) -> Dict:
        """compute performance metrics
        Args:
            observation : List
                current / most recent observation of environment
            done : bool
                True if last step of episode
        """

        obs = observation
        info = super().get_info(observation=obs, done=done)

        # parse pursuer and evader current states
        p0_p_cb__rhcbci = np.array([
            obs[self.PARAMS.OBSERVATION.I_PURSUER_PX],
            obs[self.PARAMS.OBSERVATION.I_PURSUER_PY],
            obs[self.PARAMS.OBSERVATION.I_PURSUER_PZ],
        ])
        v0_p_cb__rhcbci = np.array([
            obs[self.PARAMS.OBSERVATION.I_PURSUER_VX],
            obs[self.PARAMS.OBSERVATION.I_PURSUER_VY],
            obs[self.PARAMS.OBSERVATION.I_PURSUER_VZ],
        ])
        p0_e_cb__rhcbci = np.array([
            obs[self.PARAMS.OBSERVATION.I_EVADER_PX],
            obs[self.PARAMS.OBSERVATION.I_EVADER_PY],
            obs[self.PARAMS.OBSERVATION.I_EVADER_PZ],
        ])
        v0_e_cb__rhcbci = np.array([
            obs[self.PARAMS.OBSERVATION.I_EVADER_VX],
            obs[self.PARAMS.OBSERVATION.I_EVADER_VY],
            obs[self.PARAMS.OBSERVATION.I_EVADER_VZ],
        ])
        
        # nearest approach metrics
        d_vesE_vesP = np.linalg.norm(p0_p_cb__rhcbci-p0_e_cb__rhcbci)
        if d_vesE_vesP < self.min_dist:

            # update records of closest approach, time, relative speed, and fuel
            self.min_dist = d_vesE_vesP
            self.min_dist_time = obs[self.PARAMS.OBSERVATION.I_MET]
            self.min_dist_speed = np.linalg.norm(v0_p_cb__rhcbci-v0_e_cb__rhcbci)
            self.min_dist_pursuer_fuel_usage = self.pursuer_init_mass - self.vesPursue.mass

        info[self.PARAMS.INFO.K_CLOSEST_APPROACH] = self.min_dist
        info[self.PARAMS.INFO.K_CLOSEST_APPROACH_TIME] = self.min_dist_time
        info[self.PARAMS.INFO.K_CLOSEST_APPROACH_SPEED] = self.min_dist_speed
        info[self.PARAMS.INFO.K_CLOSEST_APPROACH_PURSUER_FUEL_USAGE] = self.min_dist_pursuer_fuel_usage

        # position-velocity product metric
        # s_vesE_vesP = np.linalg.norm(v0_p_cb__rhcbci-v0_e_cb__rhcbci)
        # pv_prod = d_vesE_vesP * s_vesE_vesP
        # if pv_prod < self.min_posvel_prod:
        #     self.min_posvel_prod = pv_prod
        # info[self.PARAMS.INFO.K_MIN_POSVEL_PRODUCT] = self.min_posvel_prod

        # fuel usage 
        info[self.PARAMS.INFO.K_PURSUER_FUEL_USAGE] = self.pursuer_init_mass - self.vesPursue.mass
        info[self.PARAMS.INFO.K_EVADER_FUEL_USAGE] = self.evader_init_mass - self.vesEvade.mass

        # weighted score
        info[self.PARAMS.INFO.K_WEIGHTED_SCORE] = self.get_weighted_score(
            dist=self.min_dist,
            speed=self.min_dist_speed,
            time=self.min_dist_time,
            fuel=self.min_dist_pursuer_fuel_usage
        )

        return info

    def get_observation(self):
        ''' return observation of pursuit and evader vessels from referee ref frame

        Returns:
            obs : list
                [0] : mission elapsed time [s]
                [1] : current vehicle (pursuer) mass [kg]
                [2] : current vehicle (pursuer) propellant  (mono prop) [kg]
                [3:6] : pursuer position wrt CB in right-hand CBCI coords [m]
                [6:9] : pursuer velocity wrt CB in right-hand CBCI coords [m/s]
                [9:12] : evader position wrt CB in right-hand CBCI coords [m]
                [12:15] : evader velocity wrt CB in right-hand CBCI coords [m/s]

        Ref: 
            - CBCI stands for celestial-body-centered inertial which is a coralary to ECI coords
            (see notation: https://github.com/mit-ll/spacegym-kspdg#code-notation)
            - KSP's body-centered inertial reference frame is left-handed
            (see https://krpc.github.io/krpc/python/api/space-center/celestial-body.html#SpaceCenter.CelestialBody.non_rotating_reference_frame)

        '''

        rf = self.vesPursue.orbit.body.non_rotating_reference_frame

        obs = [None] * self.PARAMS.OBSERVATION.LEN

        # get pursuer mass properties
        obs[self.PARAMS.OBSERVATION.I_MET] = self.vesPursue.met
        obs[self.PARAMS.OBSERVATION.I_PURSUER_MASS] = self.vesPursue.mass
        obs[self.PARAMS.OBSERVATION.I_PURSUER_PROP_MASS] = self.vesPursue.resources.amount('MonoPropellant')

        # got pursuer and evader position and velocity in 
        # left-handed celestial-body-centered-inertial frame
        p_p_cb__lhcbci = list(self.vesPursue.position(rf))
        v_p_cb__lhcbci = list(self.vesPursue.velocity(rf))
        p_e_cb__lhcbci = list(self.vesEvade.position(rf))
        v_e_cb__lhcbci = list(self.vesEvade.velocity(rf))

        # convert to right-hand system and add to observation
        p_p_cb__rhcbci = U.convert_lhcbci_to_rhcbci(p_p_cb__lhcbci)
        v_p_cb__rhcbci = U.convert_lhcbci_to_rhcbci(v_p_cb__lhcbci)
        p_e_cb__rhcbci = U.convert_lhcbci_to_rhcbci(p_e_cb__lhcbci)
        v_e_cb__rhcbci = U.convert_lhcbci_to_rhcbci(v_e_cb__lhcbci)

        # store observation of pursuer and evader position and velocity
        obs[self.PARAMS.OBSERVATION.I_PURSUER_PX], \
            obs[self.PARAMS.OBSERVATION.I_PURSUER_PY], \
            obs[self.PARAMS.OBSERVATION.I_PURSUER_PZ]  = \
            p_p_cb__rhcbci 
        
        obs[self.PARAMS.OBSERVATION.I_PURSUER_VX], \
            obs[self.PARAMS.OBSERVATION.I_PURSUER_VY], \
            obs[self.PARAMS.OBSERVATION.I_PURSUER_VZ]  = \
            v_p_cb__rhcbci

        obs[self.PARAMS.OBSERVATION.I_EVADER_PX], \
            obs[self.PARAMS.OBSERVATION.I_EVADER_PY], \
            obs[self.PARAMS.OBSERVATION.I_EVADER_PZ]  = \
            p_e_cb__rhcbci

        obs[self.PARAMS.OBSERVATION.I_EVADER_VX], \
            obs[self.PARAMS.OBSERVATION.I_EVADER_VY], \
            obs[self.PARAMS.OBSERVATION.I_EVADER_VZ]  = \
            v_e_cb__rhcbci

        return obs

    @classmethod
    def observation_list_to_dict(cls, obs_list):
        """convert observation from list to dict"""
        assert len(obs_list) == cls.PARAMS.OBSERVATION.LEN
        obs_dict = dict()
        obs_dict[cls.PARAMS.OBSERVATION.K_MET] = obs_list[cls.PARAMS.OBSERVATION.I_MET]
        obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_MASS] = obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_MASS]
        obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_PROP_MASS] = obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_PROP_MASS]
        obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_PX] = obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_PX]
        obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_PY] = obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_PY]
        obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_PZ] = obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_PZ]
        obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_VX] = obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_VX]
        obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_VY] = obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_VY]
        obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_VZ] = obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_VZ]
        obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_PX] = obs_list[cls.PARAMS.OBSERVATION.I_EVADER_PX]
        obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_PY] = obs_list[cls.PARAMS.OBSERVATION.I_EVADER_PY]
        obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_PZ] = obs_list[cls.PARAMS.OBSERVATION.I_EVADER_PZ]
        obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_VX] = obs_list[cls.PARAMS.OBSERVATION.I_EVADER_VX]
        obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_VY] = obs_list[cls.PARAMS.OBSERVATION.I_EVADER_VY]
        obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_VZ] = obs_list[cls.PARAMS.OBSERVATION.I_EVADER_VZ]

        return obs_dict

    @classmethod
    def observation_dict_to_list(cls, obs_dict):
        """convert observation from list to dict"""
        obs_list = cls.PARAMS.OBSERVATION.LEN * [None]
        obs_list[cls.PARAMS.OBSERVATION.I_MET] = obs_dict[cls.PARAMS.OBSERVATION.K_MET]
        obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_MASS] = obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_MASS]
        obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_PROP_MASS] = obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_PROP_MASS]
        obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_PX] = obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_PX]
        obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_PY] = obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_PY]
        obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_PZ] = obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_PZ]
        obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_VX] = obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_VX]
        obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_VY] = obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_VY]
        obs_list[cls.PARAMS.OBSERVATION.I_PURSUER_VZ] = obs_dict[cls.PARAMS.OBSERVATION.K_PURSUER_VZ]
        obs_list[cls.PARAMS.OBSERVATION.I_EVADER_PX] = obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_PX]
        obs_list[cls.PARAMS.OBSERVATION.I_EVADER_PY] = obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_PY]
        obs_list[cls.PARAMS.OBSERVATION.I_EVADER_PZ] = obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_PZ]
        obs_list[cls.PARAMS.OBSERVATION.I_EVADER_VX] = obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_VX]
        obs_list[cls.PARAMS.OBSERVATION.I_EVADER_VY] = obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_VY]
        obs_list[cls.PARAMS.OBSERVATION.I_EVADER_VZ] = obs_dict[cls.PARAMS.OBSERVATION.K_EVADER_VZ]

        return obs_list
        
    def get_pe_relative_distance(self):
        '''compute relative distance between pursuer and evader'''
        p_vesE_vesP__lhpbody = self.vesEvade.position(self.vesPursue.reference_frame)
        return np.linalg.norm(p_vesE_vesP__lhpbody)

    def get_pe_relative_speed(self):
        '''compute relative speed between pursuer and evader'''
        v_vesE_vesP__lhpbody = self.vesEvade.velocity(self.vesPursue.reference_frame)
        return np.linalg.norm(v_vesE_vesP__lhpbody)
    
    def bot_policy(self):
        """ Re-direct function to rename generic func to lbg1-specific func
        """
        self.evasive_maneuvers()

    def evasive_maneuvers(self):
        ''' evasive maneuvers algorithm
        '''
        raise NotImplementedError("Must be implemented by child class")

    def enforce_episode_termination(self):
        '''determine if distance or timeout episode termination conditions are met
        '''
        
        while not self.stop_episode_termination_thread:
            # get distance to pursuer
            d_vesE_vesP = self.get_pe_relative_distance()
            is_captured = False
            if self.capture_dist is not None:
                is_captured = d_vesE_vesP < self.capture_dist
                if is_captured:
                    self.logger.info("\n~~~SUCCESSFUL CAPTURE!~~~\n")

            # check for episode timeout
            is_timeout = self.vesPursue.met > self.episode_timeout
            if is_timeout:
                self.logger.info("\n~~~EPISODE TIMEOUT~~~\n")

            if is_captured or is_timeout:
                self.logger.info("Terminating episode...\n")
                self.is_episode_done = True
                self.stop_bot_thread = True
                self.stop_episode_termination_thread = True

    @classmethod
    def dpg_telem_setup(cls, history_sec: float):
        """
        ## Description
        Initializes DearPyGui telemetry plots and state for this environment. Called once
        at startup by `run_dpg_plotter` inside the DPG window context.

        ## Parameters
        - **history_sec** (`float`): Duration of the rolling time window to display.

        ## Returns
        - **state** (`dict`): Container holding plot tags, axis handles, and any
        internal buffers needed for subsequent updates.

        ## Notes
        - Responsible for creating all DPG widgets (plots, axes, series, legends).
        - Should not perform any per-frame updates or data ingestion.
        """
        ingest_cap_hz = 240
        max_points = int(history_sec * ingest_cap_hz)
        def rb(): return (deque(maxlen=max_points), deque(maxlen=max_points))  # (t,y)

        # state of plotter
        state = {
            "H": history_sec,
            "P": cls.PARAMS.OBSERVATION,    # subset of class params, used in state for simplification in update function
            "buf": {"dist_ep": rb(), "speed_ep": rb()},
            "tags": {}, "axes": {}
        }

        # UI
        with dpg.plot(label="Relative Distance (m)", height=280, width=860):
            x1 = dpg.add_plot_axis(dpg.mvXAxis, label="time since present (s)")
            y1 = dpg.add_plot_axis(dpg.mvYAxis, label="distance (m)")
            dpg.add_plot_legend(location=dpg.mvPlot_Location_SouthWest)
            t1 = dpg.add_line_series([], [], parent=y1, label="Evader-Pursuer")
        with dpg.plot(label="Relative Speed (m/s)", height=280, width=860):
            x2 = dpg.add_plot_axis(dpg.mvXAxis, label="time since present (s)")
            y2 = dpg.add_plot_axis(dpg.mvYAxis, label="speed (m/s)")
            dpg.add_plot_legend(location=dpg.mvPlot_Location_SouthWest)
            t2 = dpg.add_line_series([], [], parent=y2, label="Evader-Pursuer")

        dpg.set_axis_limits(x1, -history_sec, 0.0)
        dpg.set_axis_limits(x2, -history_sec, 0.0)

        state["tags"].update({"dist_ep": t1, "speed_ep": t2})
        state["axes"].update({"x1": x1, "y1": y1, "x2": x2, "y2": y2})
        return state
    
    @classmethod
    def dpg_telem_update(cls, state, t, obs, *, do_draw: bool):
        """
        ## Description
        Processes new observations and refreshes telemetry plot data as needed.
        Called repeatedly by `run_dpg_plotter` at the GUI frame rate.

        ## Parameters
        - **state** (`dict`): The plotting state returned from `dpg_telem_setup()`.
        - **t** (`float` or `None`): Timestamp in seconds for the current observation.
        - **obs** (array-like or `None`): Observation vector from the environment.
        - **do_draw** (`bool`): `True` when the plot should be redrawn this frame;
        `False` if only ingestion should occur.

        ## Notes
        - When `t` and `obs` are `None`, may still update visual elements (e.g.,
        moving axes or annotations).
        - Should be lightweight; avoid long computations in this loop.
        """
        P = state["P"]; H = state["H"]; tags = state["tags"]; axes = state["axes"]

        # Ingest only when we have new data
        if t is not None and obs is not None:
            px,py,pz = obs[P.I_PURSUER_PX], obs[P.I_PURSUER_PY], obs[P.I_PURSUER_PZ]
            pvx,pvy,pvz = obs[P.I_PURSUER_VX], obs[P.I_PURSUER_VY], obs[P.I_PURSUER_VZ]
            ex,ey,ez = obs[P.I_EVADER_PX], obs[P.I_EVADER_PY], obs[P.I_EVADER_PZ]
            evx,evy,evz = obs[P.I_EVADER_VX], obs[P.I_EVADER_VY], obs[P.I_EVADER_VZ]

            dist_ep  = math.dist((ex,ey,ez), (px,py,pz))
            speed_ep = math.dist((evx,evy,evz), (pvx,pvy,pvz))

            for k,v in (("dist_ep",dist_ep), ("speed_ep",speed_ep)):
                xs, ys = state["buf"][k]
                xs.append(float(t)); ys.append(float(v))

        if not do_draw:
            return

        # Draw (use latest t across series)
        latest_t = None
        for xs,_ in state["buf"].values():
            if xs:
                latest_t = xs[-1] if latest_t is None else max(latest_t, xs[-1])
        if latest_t is None:
            return

        def push(k, tag):
            xs, ys = state["buf"][k]
            if not xs: dpg.set_value(tag, [[], []]); return
            xl, yl = list(xs), list(ys)
            tmin = latest_t - H
            start = 0
            for i in range(len(xl)-1, -1, -1):
                if xl[i] < tmin: start = i+1; break
            x_rel = [xi - latest_t for xi in xl[start:]]
            dpg.set_value(tag, [x_rel, yl[start:]])

        push("dist_ep",  tags["dist_ep"])
        push("speed_ep", tags["speed_ep"])

        # Light autoscale
        for valname, yaxis in (("dist_ep", axes["y1"]),
                            ("speed_ep", axes["y2"])):
            ys = list(state["buf"][valname][1])
            if ys:
                ymin, ymax = min(ys), max(ys); pad = 0.05 * max(1e-6, ymax - ymin)
                dpg.set_axis_limits(yaxis, ymin - pad, ymax + pad)

