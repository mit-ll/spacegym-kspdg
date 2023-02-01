# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Parent and subclasses of all LBG1 environments that use a 
# a passive lady vessel and heuristic guard that pursues
# the bandit

import time
import numpy as np

from typing import Tuple

from kspdg.lbg1.lbg1_base import LadyBanditGuardGroup1Env

class LBG1_LG1_ParentEnv(LadyBanditGuardGroup1Env):

    GUARD_BANDIT_PURSUIT_THROTTLE = 0.5

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
        self.vesGuard.control.forward = LBG1_LG1_ParentEnv.GUARD_BANDIT_PURSUIT_THROTTLE

        while not self.stop_bot_thread: 

            # get position of Bandit in Guard's NTW reference frame
            p_vesB_vesG__lhgbody = np.array(self.vesBandit.position(self.vesGuard.orbital_reference_frame))
            u_vesB_vesG__lhgbody = p_vesB_vesG__lhgbody/np.linalg.norm(p_vesB_vesG__lhgbody)

            # set autopilot target direction to point at bandit
            self.vesGuard.auto_pilot.target_direction = u_vesB_vesG__lhgbody

            # throttle is set at startup, just wait for stop thread flag
            time.sleep(5.0)
        
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

class LBG1_LG0_I1_Env(LBG1_LG1_ParentEnv):
    INIT_LOADFILE = "lbg1_i1_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG0_I1_Env.INIT_LOADFILE, **kwargs)

class LBG1_LG0_I2_Env(LBG1_LG1_ParentEnv):
    INIT_LOADFILE = "lbg1_i2_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG0_I2_Env.INIT_LOADFILE, **kwargs)