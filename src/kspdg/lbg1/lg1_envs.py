# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Parent and subclasses of all LBG1 environments that use a 
# a passive lady vessel and heuristic guard that pursues
# the bandit

import time
import numpy as np

from types import SimpleNamespace

import kspdg.utils.utils as U
from kspdg.lbg1.lbg1_base import LadyBanditGuardGroup1Env

class LBG1_LG1_ParentEnv(LadyBanditGuardGroup1Env):

    def __init__(self, 
        loadfile: str, 
        pursuit_throttle: float = 1.0,
        min_speed_thresh: float = 0.2,
        max_speed_thresh: float = 10.0,
        ang_vel_pos_thresh: float = 15*np.pi/180.0,
        pointing_thresh: float = 30.0,
        loop_timeout: float = 30.0,
        reorient_time: float = 1.0,
        min_burn_time: float = 1.0,
        **kwargs):
        """
        Args:
            pursuit_throttle : float
                throttle level for guard to apply to pursue bandit
            min_speed_thresh : float
                [m/s] threshold at which relative velocity is considered zero
            max_speed_thresh : float
                [m/s] velocity at which targeting cycle is switched from direct
                pursuit to zeroing
            ang_vel_pos_thresh : float
                [rad] angular threshold between relative velocity and position at which
                coast phase will terminate
            pointing_thresh : float
                [deg] pointing error threshold at which zeroing-out maneuver will pause
                burn to wait until pointing re-aligns
            loop_timeout : float
                [s] max time for a maneuver loop to prevent endless loop
            reorient_time : float
                [s] time given to re-orient spacecraft
            min_burn_time
                [s] minimum burn time to ensure some progress in made toward bandit and that
                vel-pos angle trigger doesn't immediately kick us out of direct burn phase"""
        super().__init__(loadfile=loadfile, **kwargs)

        self.pursuit_throttle = pursuit_throttle
        self.min_speed_thresh = min_speed_thresh
        self.max_speed_thresh = max_speed_thresh
        self.ang_vel_pos_thresh = ang_vel_pos_thresh
        self.pointing_thresh = pointing_thresh
        self.loop_timeout = loop_timeout
        self.reorient_time = reorient_time
        self.min_burn_time = min_burn_time

    def lady_guard_policy(self):
        """lady is passive, guard applies a heuristic pursuit of bandit"""
        
        # Set and engage guard auto-pilot reference frame so 
        # left-handed NTW frame centered on guard
        self.vesGuard.auto_pilot.reference_frame = self.vesGuard.orbital_reference_frame
        self.vesGuard.auto_pilot.engage()

        # delay to give time for evader to re-orient
        time.sleep(self.reorient_time)

        # turn on low-thrust maneuver
        self.vesGuard.control.rcs = True

        while not self.stop_bot_thread: 

            # Step 1: Zero-out relative velocity between bandit and guard
            self.zeroout_bandit_guard_velocity()

            if self.stop_bot_thread:
                break

            # Step 2: Direct-pointing pursuit burn of guard toward bandit
            self.direct_burn_guard_toward_bandit()

            if self.stop_bot_thread:
                break

            # Step 3: Coast until relative velocity and position come out of alignment
            self.coast_until_pos_vel_misalign()
        
        # terminate throttle
        self.vesGuard.control.forward = 0.0
        self.vesGuard.control.right = 0.0
        self.vesGuard.control.up = 0.0
        self.vesGuard.auto_pilot.disengage()

    def zeroout_bandit_guard_velocity(self):
        """Perform burn to zero-out relative velocity between bandit and guard"""

        self.logger.debug("Zeroing: Zeroing Out Relative Velocities")

        loop_start_time = time.time()
        while True:

            # get velocity and speed of bandit relative to guard
            vel_vesB_vesG__lhgntw = self.vesBandit.velocity(self.vesGuard.orbital_reference_frame)
            spd_vesB_vesG = np.linalg.norm(vel_vesB_vesG__lhgntw)
            self.logger.debug("Zeroing: Bandit-Guard relative speed = {}".format(spd_vesB_vesG))

            # check zero-ing vel breakout conditions to move to next step
            if (spd_vesB_vesG < self.min_speed_thresh) or \
                time.time() - loop_start_time > self.loop_timeout or \
                self.stop_bot_thread:
                break

            # point at negative relative velocity vector and burn
            self.vesGuard.auto_pilot.target_direction = -np.array(vel_vesB_vesG__lhgntw)

            # if pointing direction too far off, give a some time to reorient to avoid 
            # "death spin"
            if self.vesGuard.auto_pilot.error > self.pointing_thresh:
                self.vesGuard.control.forward = 0
                self.logger.debug("Zeroing-Reorientation: Pointing error = {}".format(self.vesGuard.auto_pilot.error))
                continue

            self.vesGuard.control.forward = -self.pursuit_throttle

        # stop burn
        self.vesGuard.control.forward = 0

    def direct_burn_guard_toward_bandit(self):
        """Point Guard directly at Bandit and burn until desired relative velocity met"""

        self.logger.debug("DirectBurn: Performing direct pursuit of Bandit...")

        # point toward bandit and give some time to re-orient
        pos_vesB_vesG__lhgntw = self.vesBandit.position(self.vesGuard.orbital_reference_frame)
        self.vesGuard.auto_pilot.target_direction = pos_vesB_vesG__lhgntw
        time.sleep(self.reorient_time)

        loop_start_time = time.time()
        while True:

            # get velocity and speed of bandit relative to guard
            vel_vesB_vesG__lhgntw = self.vesBandit.velocity(self.vesGuard.orbital_reference_frame)
            spd_vesB_vesG = np.linalg.norm(vel_vesB_vesG__lhgntw)
            self.logger.debug("DirectBurn: Bandit-Guard relative speed = {}".format(spd_vesB_vesG))

            # compute velocity-position angle
            pos_vesB_vesG__lhgntw = self.vesBandit.position(self.vesGuard.orbital_reference_frame)
            ang_vel_pos_rad = U.angle_between(
                -np.array(vel_vesB_vesG__lhgntw), 
                np.array(pos_vesB_vesG__lhgntw))

            # check direct pursuit burn breakout conditions to move to next step
            # Break out conditions:
            # relative speed threshold is met OR
            # top-level stop flag is thrown OR
            # angular deviation between vel and pos vectors exceeds threshold AFTER a min burn time is achieved
            loop_time = time.time() - loop_start_time
            if spd_vesB_vesG > self.max_speed_thresh or \
                loop_time > self.loop_timeout or\
                self.stop_bot_thread or \
                (ang_vel_pos_rad > self.ang_vel_pos_thresh and loop_time > self.min_burn_time ):
                break

            # Point at Bandint and burn for fixed time
            self.vesGuard.auto_pilot.target_direction = pos_vesB_vesG__lhgntw
            self.vesGuard.control.forward = self.pursuit_throttle

        # stop burn
        self.vesGuard.control.forward = 0

    def coast_until_pos_vel_misalign(self):
        """Coast until the relative position and relative velocity of Bandit-Guard misalign"""

        self.vesGuard.control.forward = 0
        self.logger.debug("Coast: Coasting toward Bandit...")
        # loop_start_time = time.time()
        while True:

            # compute angle between relative velocity and relative position
            # vectors for the Bandit-Guard system
            vel_vesB_vesG__lhgntw = self.vesBandit.velocity(self.vesGuard.orbital_reference_frame)
            pos_vesB_vesG__lhgntw = self.vesBandit.position(self.vesGuard.orbital_reference_frame)
            ang_vel_pos_rad = U.angle_between(
                -np.array(vel_vesB_vesG__lhgntw), 
                np.array(pos_vesB_vesG__lhgntw))

            self.logger.debug("Coast: Bandit-Guard vel-pos vector angle = {}".format(ang_vel_pos_rad))

            # check for coast breakout condition to repeat process
            if ang_vel_pos_rad > self.ang_vel_pos_thresh or \
                self.stop_bot_thread:
                # time.time() - loop_start_time > LBG1_LG1_ParentEnv.LOOP_TIMEOUT:
                break

            # re-orient along velocity vector to make smoother transition to zeroing phase
            self.vesGuard.auto_pilot.target_direction = -np.array(vel_vesB_vesG__lhgntw)

    # @staticmethod
    # def compute_target_pointing_angles(vesEgo, vesTarg) -> Tuple[float, float]:
    #     """compute autopilot pitch and heading angles to point at a target vessel

    #     Args:
    #         vesEgo : krpc.types.Vessel
    #             the vessel performing the targeting. Note this is potentially 
    #             different from krpc's active_vessel which is the vessel on which
    #             the gui is focused
    #         vesTarg : krpc.types.Vessel
    #             the vessel being targeted. Note that this is potentially
    #             different from krpc's target_vessel. Changing the target_vessel
    #             in krpc would affect behavior of active_vessel


    #     Returns:
    #         target_pitch : float
    #             The target pitch, in degrees, between -90° and +90°
    #         target_heading : 
    #             The target heading, in degrees, between 0° and 360°.
    #     """

    #     # get position of target in ego's reference frame
    #     p_vesTarg_vesEgo__lhEgoBody = vesTarg.position(vesEgo.reference_frame)

class LBG1_LG1_I1_Env(LBG1_LG1_ParentEnv):
    INIT_LOADFILE = "lbg1_i1_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG1_I1_Env.INIT_LOADFILE, **kwargs)

class LBG1_LG1_I2_Env(LBG1_LG1_ParentEnv):
    INIT_LOADFILE = "lbg1_i2_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG1_I2_Env.INIT_LOADFILE, **kwargs)