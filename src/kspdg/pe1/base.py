# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import time
import gymnasium as gym
import numpy as np

from types import SimpleNamespace
from typing import List, Dict
from threading import Thread

import kspdg.utils.constants as C
import kspdg.utils.utils as U
from kspdg.base_envs import KSPDGBaseEnv

DEFAULT_EPISODE_TIMEOUT = 240.0 # [sec]
DEFAULT_CAPTURE_DIST = 5.0      # [m]

class PursuitEnv(KSPDGBaseEnv):
    '''
    Base environment for 1v1 pursuit-evasion orbital scenarios
    
    All inherited classes share the following
        - Agent controls the pursuer and evader has a scripted policy (although specific policy varies)
        - Pursuer and evader vehicles are identical and are meant to be the same through all
        inherited scenarios (although there is not a current enforcement of this)
        - Observation and Action spaces shared between all child envs
    '''

    # hard-coded, static parameters for pursuer vehicle
    # that are accessible yet constant (so shouldn't be
    # in observation which should really only be variable values)
    # Need for hard-coding rsc properties comes from the errors in 
    # krpc's handling of thruster objects.
    PARAMS = SimpleNamespace()
    PARAMS.PURSUER = SimpleNamespace()
    PARAMS.EVADER = SimpleNamespace()
    PARAMS.PURSUER.RCS = SimpleNamespace()
    PARAMS.OBSERVATION = SimpleNamespace()
    PARAMS.INFO = SimpleNamespace()

    # observation space paramterization
    # [0] : mission elapsed time [s]
    # [1] : current vehicle (pursuer) mass [kg]
    # [2] : current vehicle (pursuer) propellant  (mono prop) [kg]
    # [3:6] : pursuer position in reference orbit right-hand CBCI coords [m]
    # [6:9] : pursuer velocity in reference orbit right-hand CBCI coords [m/s]
    # [9:12] : evader position in reference orbit right-hand CBCI coords [m]
    # [12:15] : evader velocity in reference orbit right-hand CBCI coords [m/s]
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
    PARAMS.INFO.K_CLOSEST_APPROACH_TIME = "closest_approach_time"
    PARAMS.INFO.K_MIN_POSVEL_PRODUCT = "minimum_position_velocity_product"
    PARAMS.INFO.K_PURSUER_FUEL_USAGE = "pursuer_fuel_usage"
    PARAMS.INFO.K_EVADER_FUEL_USAGE = "evader_fuel_usage"
    PARAMS.INFO.K_DV_AT_TF = "expected_deltav_at_final_time"

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
        capture_dist:float = DEFAULT_CAPTURE_DIST):
        """
        Args:
            episode_timeout : float
                max length of episode [sec]
            capture_dist : float
                distance at which evader is considered captured [m]
        """

        assert episode_timeout > 0
        assert capture_dist > 0
        self.episode_timeout = episode_timeout
        self.capture_dist = capture_dist

        # establish load file for environment resets
        self.loadfile = loadfile

        # establish observation space (see get_observation for mapping)
        self.observation_space = gym.spaces.Box(
            low = np.concatenate((np.zeros(3), -np.inf*np.ones(12))),
            high = np.inf * np.ones(15)
        )

        # establish action space (forward, right, down, time)
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, 0.0]), 
            high=np.array([1.0, 1.0, 1.0, 10.0])
        )
        
        # don't call reset. This allows instantiation and partial testing
        # without connecting to krpc server
        
    def reset(self):

        # connect to KRPC server and load mission save file 
        self.connect_and_load_on_reset()

        # get vessel objects
        self.vesEvade, self.vesPursue = self.conn.space_center.vessels[:3]

        # Set the pursuer as the the active (i.e. human-controlled) vessel
        # and target evader
        self.conn.space_center.active_vessel = self.vesPursue
        self.conn.space_center.target_vessel = self.vesEvade
        print("Changing active vehicle...")
        time.sleep(1)   # give time to re-orient

        # set the evader to stability assist and orient in orbit-normal direction
        # orient pursuer in target-pointing direction

        print("Activating Pursuer SAS, RCS and orienting to Evader...")
        self.vesPursue.control.sas = True
        time.sleep(0.1)   # give time to re-orient
        self.vesPursue.control.sas_mode = self.vesPursue.control.sas_mode.target
        time.sleep(2)   # give time to re-orient

        # activate RCS thrusters
        self.vesPursue.control.rcs = True

        # reset performance metrics
        self.min_dist = np.inf
        self.min_dist_time = 0.0
        self.min_posvel_prod = np.inf
        self.pursuer_init_mass = self.vesPursue.mass
        self.evader_init_mass = self.vesEvade.mass

        # start process for evader maneuvers
        self.stop_evade_thread = False
        self.evade_thread = Thread(target=self.evasive_maneuvers)
        self.evade_thread.start()

        # start process for checking episode termination
        self.is_episode_done = False
        self.stop_episode_termination_thread = False
        self.episode_termination_thread = Thread(target=self.enforce_episode_termination)
        self.episode_termination_thread.start()

        # package observation and performance metric info for return
        obs = self.get_observation()
        info = self.get_info(obs, False)

        return obs, info

    def step(self, action):
        ''' Apply thrust and torque actuation for specified time duration
        Args:
            action : np.ndarray
                4-tuple of throttle values in 3D and timestep (forward, right, down, tstep)

        Ref: 
            Actions are in forward, right, down to align with the right-handed version of the
            Vessel Surface Reference Frame 
            https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
        '''

        # parse and apply action
        self.vesPursue.control.forward = action[0]
        self.vesPursue.control.right = action[1]
        self.vesPursue.control.up = -action[2]

        # execute maneuver for specified time, checking for end
        # conditions while you do
        timeout = time.time() + action[3]
        while True: 
            if self.is_episode_done or time.time() > timeout:
                break

        # zero out thrusts
        self.vesPursue.control.forward = 0.0
        self.vesPursue.control.up = 0.0
        self.vesPursue.control.right = 0.0

        # get observation
        obs = self.get_observation()

        # compute reward
        rew = -self.get_pe_relative_distance()

        # compute performance metrics
        info = self.get_info(obs, self.is_episode_done)

        return obs, rew, self.is_episode_done, info

    def get_info(self, observation: List, done: bool) -> Dict:
        """compute performance metrics
        Args:
            observation : List
                current / most recent observation of environment
            done : bool
                True if last step of episode
        """

        obs = observation
        info = dict()

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
            self.min_dist = d_vesE_vesP
            self.min_dist_time = obs[self.PARAMS.OBSERVATION.I_MET]
        info[self.PARAMS.INFO.K_CLOSEST_APPROACH] = self.min_dist
        info[self.PARAMS.INFO.K_CLOSEST_APPROACH_TIME] = self.min_dist_time

        # position-velocity product metric
        s_vesE_vesP = np.linalg.norm(v0_p_cb__rhcbci-v0_e_cb__rhcbci)
        pv_prod = d_vesE_vesP * s_vesE_vesP
        if pv_prod < self.min_posvel_prod:
            self.min_posvel_prod = pv_prod
        info[self.PARAMS.INFO.K_MIN_POSVEL_PRODUCT] = self.min_posvel_prod

        # fuel usage 
        info[self.PARAMS.INFO.K_PURSUER_FUEL_USAGE] = self.pursuer_init_mass - self.vesPursue.mass
        info[self.PARAMS.INFO.K_EVADER_FUEL_USAGE] = self.evader_init_mass - self.vesEvade.mass

        # compute approximate delta-v need to intercept non-maneuvering
        # evader
        if done:

            # call capture dv estimator
            dv0, dvf = U.estimate_capture_dv(
                p0_prs=p0_p_cb__rhcbci,
                v0_prs=v0_p_cb__rhcbci,
                p0_evd=p0_e_cb__rhcbci,
                v0_evd=v0_e_cb__rhcbci,
                tof=self.episode_timeout
            )

            info[self.PARAMS.INFO.K_DV_AT_TF] = dv0 + dvf

        else:
            info[self.PARAMS.INFO.K_DV_AT_TF] = None


        return info

    def get_observation(self):
        ''' return observation of pursuit and evader vessels from referee ref frame

        Returns:
            obs : list
                [0] : mission elapsed time [s]
                [1] : current vehicle (pursuer) mass [kg]
                [2] : current vehicle (pursuer) propellant  (mono prop) [kg]
                [3:6] : pursuer position in reference orbit right-hand CBCI coords [m]
                [6:9] : pursuer velocity in reference orbit right-hand CBCI coords [m/s]
                [9:12] : evader position in reference orbit right-hand CBCI coords [m]
                [12:15] : evader velocity in reference orbit right-hand CBCI coords [m/s]

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
            is_captured = d_vesE_vesP < self.capture_dist
            if is_captured:
                print("\n~~~SUCCESSFUL CAPTURE!~~~\n")

            # check for episode timeout
            is_timeout = self.vesPursue.met > self.episode_timeout
            if is_timeout:
                print("\n~~~EPISODE TIMEOUT~~~\n")

            if is_captured or is_timeout:
                print("Terminating episode...\n")
                self.is_episode_done = True
                self.stop_episode_termination_thread = True

    def close(self):

        # handle evasive maneuvering thread
        self.stop_evade_thread = True
        self.evade_thread.join()

        # handle episode termination thread
        self.stop_episode_termination_thread = True
        self.episode_termination_thread.join()

        # close connection to krpc server
        self.conn.close()

    def convert_rhntw_to_rhpbody(self, v__rhntw: List[float]) -> List[float]:
        '''Converts vector in right-handed NTW frame to pursuer vessel right-oriented body frame
        Args:
            v__ntw : List[float]
                3-vector represented in orbital NTW coords
        
        Returns
            v__rhpbody : List[float]
                3-vector vector represented in pursuer's right-hadded body coords (forward, right, down)
        
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
        v__lhpbody = list(self.conn.space_center.transform_direction(
            direction = tuple(v__lhntw),
            from_ = self.vesPursue.orbital_reference_frame,
            to = self.vesPursue.reference_frame
        ))

        # convert left-handed body coords (right, forward, down) to right-handed body coords (forward, right, down)
        v__rhpbody = U.convert_lhbody_to_rhbody(v__lhbody=v__lhpbody)

        return v__rhpbody

    def convert_rhcbci_to_rhpbody(self, v__rhcbci: List[float]) -> List[float]:
        '''Converts vector in right-handed celestial-body-centered-inertial frame to 
        pursuer vessel right-oriented body frame

        Args:
            v__rhcbci : List[float]
                3-vector represented in celestial-body-centered-inertial fram 
                (similar to ECI coords, but we aren't necessarily orbitiing Earth)
        
        Returns
            v__rhpbody : List[float]
                3-vector vector represented in pursuer's right-hadded body coords (forward, right, down)
        
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
        v__lhpbody = list(self.conn.space_center.transform_direction(
            direction = tuple(v__lhcbci),
            from_ = self.vesPursue.orbit.body.non_rotating_reference_frame,
            to = self.vesPursue.reference_frame
        ))

        # convert left-handed body coords (right, forward, down) to right-handed body coords (forward, right, down)
        v__rhpbody = U.convert_lhbody_to_rhbody(v__lhbody=v__lhpbody)

        return v__rhpbody
