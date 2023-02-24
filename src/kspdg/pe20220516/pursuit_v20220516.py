# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import time
import gymnasium as gym
import numpy as np

from types import SimpleNamespace
from typing import List
from threading import Thread

import kspdg.utils.constants as C
import kspdg.utils.utils as U

from kspdg.base_envs import KSPDGBaseEnv

_INIT_LOADFILE = "20220516_PursuitEvade_init"
_EVADE_DIST_THRESHOLD = 100.0
_MISSION_DONE_DIST_THRESHOLD = 20.0

class PursuitEnvV20220516(KSPDGBaseEnv):
    '''
     A simple pursuit-evasion orbital scenario
    
     Agent controls the pursuer and evader has a scripted policy
    
     The evasion algorithm is very simplistic:
     if the pursuer comes within a specified radius, the 
     evader simply burns orbit-normal direction to evade
     
     Objective is to safely grapple/dock with evader without crashing
    '''

    # hard-coded, static parameters for pursuer vehicle
    # that are accessible yet constant (so shouldn't be
    # in observation which should really only be variable values)
    # Need for hard-coding rsc properties comes from the errors in 
    # krpc's handling of thruster objects.
    PARAMS = SimpleNamespace()
    PARAMS.PURSUER = SimpleNamespace()
    PARAMS.PURSUER.RCS = SimpleNamespace()

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


    def __init__(self):

        # establish load file for environment resets
        self.loadfile = _INIT_LOADFILE

        # establish observation space (see get_observation for mapping)
        self.observation_space = gym.spaces.Box(
            low = np.concatenate((np.zeros(2), -np.inf*np.ones(12))),
            high = np.inf * np.ones(14)
        )

        # establish action space (forward, right, down, time)
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, 0.0]), 
            high=np.array([1.0, 1.0, 1.0, 10.0])
        )
        
        # call reset function
        self.reset()
        
    
    def _reset_vessels(self) -> None:

        # get vessel objects
        self.vesReferee, self.vesEvade, self.vesPursue = self.conn.space_center.vessels[:3]

        # Set the pursuer as the the active (i.e. human-controlled) vessel
        # and target evader
        self.conn.space_center.active_vessel = self.vesPursue
        self.conn.space_center.target_vessel = self.vesEvade
        print("Changing active vehicle...")
        time.sleep(1)   # give time to re-orient

        # set the evader to stability assist and orient in orbit-normal direction
        # orient pursuer in target-pointing direction

        self.vesEvade.control.sas = True
        self.vesPursue.control.sas = True
        print("Activating stability assist...")
        time.sleep(0.1)   # give time to re-orient
        self.vesEvade.control.sas_mode = self.vesEvade.control.sas_mode.normal
        self.vesPursue.control.sas_mode = self.vesPursue.control.sas_mode.target
        print("Re-orienting Pursuer and Evader...")
        time.sleep(2)   # give time to re-orient

        # actuate RCS thrusters
        print("Activating reaction control systems...")
        self.vesEvade.control.rcs = True
        self.vesPursue.control.rcs = True

    def _reset_episode_metrics(self) -> None:
        pass

    def _start_bot_threads(self) -> None:
        """ Start parallel thread to execute Evader's evasive maneuvers
        """

        self.stop_evade_thread = False

        # check that thread does not exist or is not running
        if hasattr(self, "evade_thread"):
            if self.evade_thread.is_alive():
                raise ConnectionError("evade_thread is already running."+ 
                    " Close and join evade_thread before restarting")

        self.evade_thread = Thread(target=self.evasive_maneuvers)
        self.evade_thread.start()

    def step(self, action):
        ''' Apply thrust and torque actuation for speciefied time
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
        rew = -self.pursue_evade_relative_distance()

        info = {}

        return obs, rew, self.is_episode_done, info
    
    def get_info(self, observation: List, done: bool):
        return {}

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

    def get_observation(self):
        ''' return observation of pursuit and evader vessels from referee ref frame

        Returns:
            obs : list
                [0] : current vehicle (pursuer) mass [kg]
                [1] : current vehicle (pursuer) propellant  (mono prop) [kg]
                [2:5] : pursuer position in reference orbit right-hand NTW coords 
                [5:8] : pursuer velocity in reference orbit right-hand NTW coords
                [8:11] : evader position in reference orbit right-hand NTW coords 
                [11:14] : evader velocity in reference orbit right-hand NTW coords

        Ref: 
            reference orbit coords from krpc are left-handed NTW frame(anti-radial, prograde, orbit-normal)
            https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-orbital-reference-frame
        '''

        rf = self.vesReferee.orbital_reference_frame

        obs = []

        # get pursuer mass properties
        obs.append(self.vesPursue.mass)
        obs.append(self.vesPursue.resources.amount('MonoPropellant'))

        # got pursuer and evader position and velocity in 
        # left-handed NTW frame relative to "referee" satellite
        p_p_r__lhntw = list(self.vesPursue.position(rf))
        v_p_r__lhntw = list(self.vesPursue.velocity(rf))
        p_e_r__lhntw = list(self.vesEvade.position(rf))
        v_e_r__lhntw = list(self.vesEvade.velocity(rf))

        # convert to right-hand system and add to observation
        p_p_r__rhntw = U.convert_lhntw_to_rhntw(p_p_r__lhntw)
        v_p_r__rhntw = U.convert_lhntw_to_rhntw(v_p_r__lhntw)
        p_e_r__rhntw = U.convert_lhntw_to_rhntw(p_e_r__lhntw)
        v_e_r__rhntw = U.convert_lhntw_to_rhntw(v_e_r__lhntw)

        # store observation of pursuer and evader position and velocity
        obs.extend(p_p_r__rhntw)
        obs.extend(v_p_r__rhntw)
        obs.extend(p_e_r__rhntw)
        obs.extend(v_e_r__rhntw)

        return obs
        
    def pursue_evade_relative_distance(self):
        '''compute relative distance between pursuer and evader'''
        p_vesE_vesP__lhpbody = self.vesEvade.position(self.vesPursue.reference_frame)
        return np.linalg.norm(p_vesE_vesP__lhpbody)

    def enforce_episode_termination(self) -> bool:
        '''determine if episode termination conditions are met
        
        Returns:
            bool
                true if episode termination criteria is met
        '''

        while not self.stop_episode_termination_thread:
            # get distance to pursuer
            d_vesE_vesP = self.pursue_evade_relative_distance()
            self.is_episode_done = d_vesE_vesP < _MISSION_DONE_DIST_THRESHOLD
            if self.is_episode_done:
                print("Successful Capture!")
                self.stop_episode_termination_thread = True

    def evasive_maneuvers(self):
        '''use relative position of pursuer to execute simple evasive maneuver
        Args:
            vesE : krpc.Vessel
                krpc vessel object for evader
            vesP : krpc.Vessel
                krpc vessel object for pursuer
        '''
        was_evading = False
        while not self.stop_evade_thread:

            # get distance to pursuer
            d_vesE_vesP = self.pursue_evade_relative_distance()

            # if pursuer is too close, evade in orbit-normal direction
            if d_vesE_vesP < _EVADE_DIST_THRESHOLD:
                self.vesEvade.control.forward = 1.0
                if not was_evading:
                    print("Pursuer detected! Executing evasive maneuvers")
                was_evading = True
            else:
                self.vesEvade.control.forward = 0.0
                if was_evading:
                    print("No pursuer in range. Zeroing thrust")
                was_evading = False

    def close(self):

        # handle evasive maneuvering thread
        self.stop_evade_thread = True
        self.evade_thread.join()

        # handle episode termination thread
        self.stop_episode_termination_thread = True
        self.episode_termination_thread.join()

        # close connection to krpc server
        self.conn.close()
