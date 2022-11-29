# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.pe1.base import PursuitEnv

DEFAULT_EVADE_DIST = 200.0   # [m]
DEFAULT_EVADE_THROTTLE = 1.0

class PursuitEnv_e3(PursuitEnv):
    def __init__(self, loadfile: str, 
        evade_dist:float = DEFAULT_EVADE_DIST, 
        evade_throttle:float = DEFAULT_EVADE_THROTTLE,
        **kwargs):
        """
        Args
            evade_dist : float
                distance from pursuer in which evader starts 
                evasive maneuvers [m]
            evade_throttle : float
                throttle value to use for evasion [0-1]
        """
        super().__init__(loadfile=loadfile, **kwargs)
        assert evade_dist > 0.0
        assert evade_throttle >= 0.0
        assert evade_throttle <= 1.0
        self.evade_dist = evade_dist
        self.evade_throttle = evade_throttle

    def evasive_maneuvers(self):
        '''use relative position of pursuer to execute simple evasive maneuver
        '''
        was_evading = False
        is_control_set = False
        while not self.stop_evade_thread:


            # get distance to pursuer
            d_vesE_vesP = self.get_pe_relative_distance()

            # check for control range
            if d_vesE_vesP < self.PARAMS.EVADER.CONTROL_RANGE:

                # turn on evader sas if not yet active
                if not is_control_set:
                    print("Activating Evader SAS and RCS...")
                    self.vesEvade.control.sas = True
                    self.vesEvade.control.sas_mode = self.vesEvade.control.sas_mode.normal
                    self.vesEvade.control.rcs = True
                    is_control_set = True

                # if pursuer is too close, evade in orbit-normal direction
                if d_vesE_vesP < self.evade_dist:
                    self.vesEvade.control.forward = self.evade_throttle
                    if not was_evading:
                        print("\n~~~PURSUER DETECTED! Executing evasive maneuvers~~~\n")
                    was_evading = True
                else:
                    self.vesEvade.control.forward = 0.0
                    if was_evading:
                        print("\n~~~NO PURSUER IN RANGE! Zeroing thrust...~~~\n")
                    was_evading = False
        
        # terminate throttle
        if self.get_pe_relative_distance() < self.PARAMS.EVADER.CONTROL_RANGE:
            self.vesEvade.control.forward = 0.0
            self.vesEvade.control.right = 0.0
            self.vesEvade.control.up = 0.0
