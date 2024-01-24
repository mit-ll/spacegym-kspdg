# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Parent and subclasses of all LBG1 environments that use an active
# Lady (constant prograde thrust) and heuristic guard that pursues
# the bandit

import time
import numpy as np

from kspdg.lbg1.lg1_envs import LBG1_LG1_ParentEnv

EVASION_THROTTLE = 1.0 # constant throttle Lady applies during evasive maneuver
EVASION_BURN_DURATION = 0.5 # [s] duration of evasive maneuver
EVASION_DIST_THRESHOLD = 500.0  # [m] L-B distance at which evasion executes
EVASION_BURN_DELAY = 10 # [s]

class LBG1_LG2_ParentEnv(LBG1_LG1_ParentEnv):

    def __init__(self, loadfile: str, **kwargs):
        super().__init__(loadfile=loadfile, **kwargs)

    def lady_guard_policy(self):
        """Lady const thrust, Guard heuristic pursuit of Bandit"""

        # Do nothing until, and unless, bandit gets too close
        while True:

            # break out and do nothing if bot thread is stopped
            if self.stop_bot_thread:
                return
            
            bandit_dist = min(self.get_lb_relative_distance(), self.get_bg_relative_distance())
            if bandit_dist < EVASION_DIST_THRESHOLD:
                self.logger.info("\n~~~BANDIT DETECTED!\nExecuting evasive maneuvers in {} sec~~~\n".format(EVASION_BURN_DELAY))
                break
            else:
                time.sleep(0.1)
        
        # Set and engage lady auto-pilot reference frame so 
        # that it points in it's own prograde direction
        self.vesLady.auto_pilot.reference_frame = self.vesLady.orbital_reference_frame
        self.vesLady.auto_pilot.target_pitch = 0.0
        self.vesLady.auto_pilot.target_heading = 0.0
        self.vesLady.auto_pilot.target_roll = 0.0
        self.vesLady.auto_pilot.engage()
        time.sleep(1.0)

        # randomize which direction to burn
        self.vesLady.auto_pilot.target_heading = np.random.choice([90.0, 270.0])

        # delay to give time for lady to re-orient
        time.sleep(EVASION_BURN_DELAY-1.0)

        # execute an impulsive maneuver
        self.logger.info("\n~~~Executing evasive maneuvers now!~~~\n")
        self.vesLady.parts.engines[0].active = True
        self.vesLady.control.throttle = EVASION_THROTTLE
        time.sleep(EVASION_BURN_DURATION)
        self.vesLady.control.throttle = 0.0

        # run loop for heuristic for guard
        super().lady_guard_policy()

        # throttle down lady at end
        self.vesLady.auto_pilot.disengage()


class LBG1_LG2_I1_Env(LBG1_LG2_ParentEnv):
    INIT_LOADFILE = "lbg1_i1_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG2_I1_Env.INIT_LOADFILE, **kwargs)

class LBG1_LG2_I2_Env(LBG1_LG2_ParentEnv):
    INIT_LOADFILE = "lbg1_i2_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG2_I2_Env.INIT_LOADFILE, **kwargs)