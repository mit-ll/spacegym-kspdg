# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Parent and subclasses of all LBG1 environments that use an active
# Lady (constant prograde thrust) and heuristic guard that pursues
# the bandit

import time

from kspdg.lbg1.lg1_envs import LBG1_LG1_ParentEnv

class LBG1_LG2_ParentEnv(LBG1_LG1_ParentEnv):

    PROGRADE_EVASION_THROTTLE = 0.2 # constant throttle Lady applies

    def __init__(self, loadfile: str, **kwargs):
        super().__init__(loadfile=loadfile, **kwargs)

    def lady_guard_policy(self):
        """Lady const thrust, Guard heuristic pursuit of Bandit"""
        
        # Set and engage lady auto-pilot reference frame so 
        # that it points in it's own prograde direction
        self.vesLady.auto_pilot.reference_frame = self.vesLady.orbital_reference_frame
        self.vesLady.auto_pilot.target_pitch = 0.0
        self.vesLady.auto_pilot.target_heading = 0.0
        self.vesLady.auto_pilot.target_roll = 0.0
        self.vesLady.auto_pilot.engage()

        # delay to give time for evader to re-orient
        time.sleep(0.5)

        # turn on low-thrust maneuver
        self.vesLady.control.rcs = True
        self.vesLady.control.forward = LBG1_LG2_ParentEnv.PROGRADE_EVASION_THROTTLE

        # run loop for heuristic for guard
        super().lady_guard_policy()

        # throttle down lady at end
        self.vesLady.control.forward = 0.0
        self.vesLady.control.right = 0.0
        self.vesLady.control.up = 0.0
        self.vesLady.auto_pilot.disengage()


class LBG1_LG2_I1_Env(LBG1_LG2_ParentEnv):
    INIT_LOADFILE = "lbg1_i1_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG2_I1_Env.INIT_LOADFILE, **kwargs)

class LBG1_LG2_I2_Env(LBG1_LG2_ParentEnv):
    INIT_LOADFILE = "lbg1_i2_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG2_I2_Env.INIT_LOADFILE, **kwargs)