# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import time

from numpy.random import rand
from kspdg.pe1.pe1_base import PursuitEvadeGroup1Env

class PE1_E2_ParentEnv(PursuitEvadeGroup1Env):
    def __init__(self, loadfile: str, **kwargs):
        super().__init__(loadfile=loadfile, **kwargs)

    def evasive_maneuvers(self):
        '''Randomized evasive manuevers
        '''

        # variable to set sas on once
        is_control_set = False

        while not self.stop_bot_thread:

            # check for control range
            if self.get_pe_relative_distance() < self.PARAMS.EVADER.CONTROL_RANGE:

                # turn on evader sas if not yet active
                if not is_control_set:
                    self.logger.info("Activating Evader SAS and RCS...")
                    self.vesEvade.control.sas = True
                    self.vesEvade.control.sas_mode = self.vesEvade.control.sas_mode.prograde
                    self.vesEvade.control.rcs = True
                    is_control_set = True

                # randomly select duration
                dur = 2*rand()

                # randomize if throttle and attitude control will be executed
                thr_on = rand() < 0.5
                att_on = rand() < 0.5

                # randomly select throttle and attitude ctrl values
                thr = thr_on * (2*rand(3) - 1)
                att = att_on * (2*rand(3) - 1)

                # actuate throttle and attitude ctrl
                self.vesEvade.control.forward = thr[0]
                self.vesEvade.control.right = thr[1]
                self.vesEvade.control.up = -thr[2]
                self.vesEvade.control.pitch = att[0]
                self.vesEvade.control.yaw = att[1]
                self.vesEvade.control.roll = att[2]

                # execute throttle for randomized amount of time
                time.sleep(dur)

        # terminate throttle
        if self.get_pe_relative_distance() < self.PARAMS.EVADER.CONTROL_RANGE:
            self.vesEvade.control.forward = 0.0
            self.vesEvade.control.right = 0.0
            self.vesEvade.control.up = 0.0

class PE1_E2_I1_Env(PE1_E2_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I1, **kwargs)
    
class PE1_E2_I2_Env(PE1_E2_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I2, **kwargs)

class PE1_E2_I3_Env(PE1_E2_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I3, **kwargs)

class PE1_E2_I4_Env(PE1_E2_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I4, **kwargs)