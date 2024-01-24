# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import time
from kspdg.pe1.pe1_base import PursuitEvadeGroup1Env

_PROGRADE_EVASION_THROTTLE = 0.2

class PE1_E4_ParentEnv(PursuitEvadeGroup1Env):
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

        while not self.stop_bot_thread:

            # throttle is set at startup, just wait for stop thread flag
            time.sleep(0.1)
        
        # terminate throttle
        self.vesEvade.control.forward = 0.0
        self.vesEvade.control.right = 0.0
        self.vesEvade.control.up = 0.0
        self.vesEvade.auto_pilot.disengage()

class PE1_E4_I1_Env(PE1_E4_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I1, **kwargs)
    
class PE1_E4_I2_Env(PE1_E4_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I2, **kwargs)

class PE1_E4_I3_Env(PE1_E4_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I3, **kwargs)

class PE1_E4_I4_Env(PE1_E4_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I4, **kwargs)