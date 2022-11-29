# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import time
from kspdg.pe1.base import PursuitEnv

_EVADE_DIST_THRESHOLD = 200.0   # [m]
_PROGRADE_EVASION_THROTTLE = 0.75

class PursuitEnv_e4(PursuitEnv):
    def __init__(self, loadfile: str, **kwargs):
        super().__init__(loadfile=loadfile, **kwargs)

    def evasive_maneuvers(self):
        '''use relative position of pursuer to execute simple evasive maneuver
        '''
        was_evading = False
        while not self.stop_evade_thread:


            # get distance to pursuer
            d_vesE_vesP = self.get_pe_relative_distance()

            # check for control range
            if d_vesE_vesP < self.PARAMS.EVADER.CONTROL_RANGE:

                # if pursuer is too close, evade in orbit-normal direction
                if d_vesE_vesP < _EVADE_DIST_THRESHOLD:
                    if not was_evading:
                        print("Pursuer detected! Executing evasive maneuvers")
                        cur_speed_mode = self.vesPursue.control.speed_mode
                        self.vesEvade.control.sas = True
                        time.sleep(0.25) # give time to commands to go through
                        self.vesPursue.control.speed_mode = self.vesPursue.control.speed_mode.orbit
                        time.sleep(0.25) # give time to commands to go through
                        self.vesEvade.control.sas_mode = self.vesEvade.control.sas_mode.prograde
                        time.sleep(2.0) # give time to commands to go through
                        self.vesEvade.control.rcs = True
                        self.vesPursue.control.speed_mode = cur_speed_mode # reset speed mode
                    was_evading = True
                    self.vesEvade.control.forward = _PROGRADE_EVASION_THROTTLE
                else:
                    self.vesEvade.control.forward = 0.0
                    if was_evading:
                        print("No pursuer in range. Zeroing thrust")
                    was_evading = False
        
        # terminate throttle
        if self.get_pe_relative_distance() < self.PARAMS.EVADER.CONTROL_RANGE:
            self.vesEvade.control.forward = 0.0
            self.vesEvade.control.right = 0.0
            self.vesEvade.control.up = 0.0
