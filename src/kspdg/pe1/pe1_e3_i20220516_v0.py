# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.pe1.e3 import PursuitEnv_e3

_INIT_LOADFILE = "20220516_PursuitEvade_init"
_MISSION_DONE_DIST_THRESHOLD = 20.0     # [m]

class PursuitEnv_e3_i20220516_v0(PursuitEnv_e3):
    def __init__(self):
        super().__init__(loadfile=_INIT_LOADFILE)

    def check_episode_termination(self) -> bool:
        '''determine if episode termination conditions are met
        
        Returns:
            bool
                true if episode termination criteria is met
        '''

        while not self.stop_episode_termination_thread:
            # get distance to pursuer
            d_vesE_vesP = self.get_pe_relative_distance()
            self.is_episode_done = d_vesE_vesP < _MISSION_DONE_DIST_THRESHOLD
            if self.is_episode_done:
                print("Successful Capture!")
                self.stop_episode_termination_thread = True
