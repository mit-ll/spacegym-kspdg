# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Parent and subclasses of all LBG1 environments that use a 
# a passive lady vessel and heuristic guard that pursues
# the bandit

import time
import numpy as np

from typing import Tuple

import kspdg.utils.utils as U
from kspdg.lbg1.lbg1_base import LadyBanditGuardGroup1Env

class LBG1_LG1_ParentEnv(LadyBanditGuardGroup1Env):

    BG_PURSUIT_THROTTLE = 1.0   # throttle level for guard to apply to pursue bandit
    BG_MIN_REL_SPEED_THESHOLD = 2.0 # [m/s] threshold at which relative velocity is considered zero
    BG_MAX_REL_SPEED_THESHOLD = 10.0    # [m/s] velocity at which targeting cycle is switched from direct
                                        # pursuit to zeroing
    BG_VEL_POS_ANG_THRESHOLD = 30*np.pi/180.0 # [rad] angular threshold between relative velocity and position at which
                                                # coast phase will terminate
    LOOP_TIMEOUT = 30 # [s] max time for a maneuver loop to prevent endless loop

    def __init__(self, loadfile: str, **kwargs):
        super().__init__(loadfile=loadfile, **kwargs)

    def lady_guard_policy(self):
        """lady is passive, guard applies a heuristic pursuit of bandit"""
        
        # Set and engage guard auto-pilot reference frame so 
        # left-handed NTW frame centered on guard
        self.vesGuard.auto_pilot.reference_frame = self.vesGuard.orbital_reference_frame
        self.vesGuard.auto_pilot.engage()

        # delay to give time for evader to re-orient
        time.sleep(0.5)

        # turn on low-thrust maneuver
        self.vesGuard.control.rcs = True

        while not self.stop_bot_thread: 

            print("DEBUG: GUARD: Performing direct pursuit of Bandit...")
            loop_start_time = time.time()
            while True:

                # get velocity and speed of bandit relative to guard
                vel_vesB_vesG__lhgntw = self.vesBandit.velocity(self.vesGuard.orbital_reference_frame)
                spd_vesB_vesG = np.linalg.norm(vel_vesB_vesG__lhgntw)
                print("DEBUG: Bandit-Guard relative speed = ",spd_vesB_vesG)

                # check zero-ing vel breakout conditions to move to next step
                if (spd_vesB_vesG < LBG1_LG1_ParentEnv.BG_MIN_REL_SPEED_THESHOLD) or \
                    time.time() - loop_start_time > LBG1_LG1_ParentEnv.LOOP_TIMEOUT:
                    break

                # point at negative relative velocity vector and burn for fixed time
                self.vesGuard.auto_pilot.target_direction = -np.array(vel_vesB_vesG__lhgntw)
                self.vesGuard.control.forward = -LBG1_LG1_ParentEnv.BG_PURSUIT_THROTTLE
                time.sleep(0.2)

            print("DEBUG: GUARD: Performing direct pursuit of Bandit...")
            loop_start_time = time.time()
            while True:

                # get velocity and speed of bandit relative to guard
                vel_vesB_vesG__lhgntw = self.vesBandit.velocity(self.vesGuard.orbital_reference_frame)
                spd_vesB_vesG = np.linalg.norm(vel_vesB_vesG__lhgntw)
                print("DEBUG: Bandit-Guard relative speed = ",spd_vesB_vesG)

                # check direct pursuit burn breakout conditions to move to next step
                if (spd_vesB_vesG > LBG1_LG1_ParentEnv.BG_MAX_REL_SPEED_THESHOLD) or \
                    time.time() - loop_start_time > LBG1_LG1_ParentEnv.LOOP_TIMEOUT:
                    break

                # Point at Bandint and burn for fixed time
                pos_vesB_vesG__lhgntw = self.vesBandit.position(self.vesGuard.orbital_reference_frame)
                self.vesGuard.auto_pilot.target_direction = pos_vesB_vesG__lhgntw
                self.vesGuard.control.forward = LBG1_LG1_ParentEnv.BG_PURSUIT_THROTTLE
                time.sleep(0.2)

            self.vesGuard.control.forward = 0
            print("DEBUG: GUARD: Coasting toward Bandit...")
            loop_start_time = time.time()
            while True:

                # compute angle between relative velocity and relative position
                # vectors for the Bandit-Guard system
                vel_vesB_vesG__lhgntw = self.vesBandit.velocity(self.vesGuard.orbital_reference_frame)
                pos_vesB_vesG__lhgntw = self.vesBandit.position(self.vesGuard.orbital_reference_frame)
                ang_vel_pos_rad = U.angle_between(
                    -np.array(vel_vesB_vesG__lhgntw), 
                    np.array(pos_vesB_vesG__lhgntw))

                print("DEBUG: Bandit-Guard vel-pos vector angle = ",ang_vel_pos_rad)

                # check for coast breakout condition to repeat process
                if ang_vel_pos_rad > LBG1_LG1_ParentEnv.BG_VEL_POS_ANG_THRESHOLD or \
                    time.time() - loop_start_time > LBG1_LG1_ParentEnv.LOOP_TIMEOUT:
                    break
        
        # terminate throttle
        self.vesGuard.control.forward = 0.0
        self.vesGuard.control.right = 0.0
        self.vesGuard.control.up = 0.0
        self.vesGuard.auto_pilot.disengage()

    @staticmethod
    def compute_target_pointing_angles(vesEgo, vesTarg) -> Tuple[float, float]:
        """compute autopilot pitch and heading angles to point at a target vessel

        Args:
            vesEgo : krpc.types.Vessel
                the vessel performing the targeting. Note this is potentially 
                different from krpc's active_vessel which is the vessel on which
                the gui is focused
            vesTarg : krpc.types.Vessel
                the vessel being targeted. Note that this is potentially
                different from krpc's target_vessel. Changing the target_vessel
                in krpc would affect behavior of active_vessel


        Returns:
            target_pitch : float
                The target pitch, in degrees, between -90° and +90°
            target_heading : 
                The target heading, in degrees, between 0° and 360°.
        """

        # get position of target in ego's reference frame
        p_vesTarg_vesEgo__lhEgoBody = vesTarg.position(vesEgo.reference_frame)

class LBG1_LG1_I1_Env(LBG1_LG1_ParentEnv):
    INIT_LOADFILE = "lbg1_i1_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG1_I1_Env.INIT_LOADFILE, **kwargs)

class LBG1_LG1_I2_Env(LBG1_LG1_ParentEnv):
    INIT_LOADFILE = "lbg1_i2_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG1_I2_Env.INIT_LOADFILE, **kwargs)