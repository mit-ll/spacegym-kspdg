# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import time
from kspdg.pe1.base import PursuitEnv

_PROGRADE_EVASION_THROTTLE = 0.2

class PursuitEnv_e4(PursuitEnv):
    def __init__(self, loadfile: str, **kwargs):
        super().__init__(loadfile=loadfile, **kwargs)

    def evasive_maneuvers(self):
        '''Constant low-thrust burn in prograde direction, regardless of pursuer
        '''

        # Set and engage evader auto-pilot reference frame so 
        # that it points in it's own prograde direction
        self.vesEvade.auto_pilot.reference_frame = self.vesEvade.orbital_reference_frame
        self.vesEvade.auto_pilot.target_pitch = 0.0
        self.vesEvade.auto_pilot.target_heading = 0.0
        self.vesEvade.auto_pilot.target_roll = 0.0
        self.vesEvade.auto_pilot.engage()

        # delay to give time for evader to re-orient
        time.sleep(0.5)

        # turn on low-thrust maneuver
        self.vesEvade.control.rcs = True
        self.vesEvade.control.forward = _PROGRADE_EVASION_THROTTLE

        while not self.stop_evade_thread:

            # throttle is set at startup, just wait for stop thread flag
            time.sleep(0.1)
        
        # terminate throttle
        self.vesEvade.control.forward = 0.0
        self.vesEvade.control.right = 0.0
        self.vesEvade.control.up = 0.0
        self.vesEvade.auto_pilot.disengage()
