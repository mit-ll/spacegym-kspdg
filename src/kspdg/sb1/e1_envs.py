# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.sb1.sb1_base import SunBlockingGroup1Env

class SB1_E1_ParentEnv(SunBlockingGroup1Env):
    def __init__(self, loadfile: str, **kwargs):
        super().__init__(loadfile=loadfile, **kwargs)

    def evasive_maneuvers(self):
        '''Do not perform evasive maneuvers
        '''
        pass

class SB1_E1_I1_Env(SB1_E1_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=SunBlockingGroup1Env.LOADFILE_I1, **kwargs)

class SB1_E1_I2_Env(SB1_E1_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=SunBlockingGroup1Env.LOADFILE_I2, **kwargs)

class SB1_E1_I3_Env(SB1_E1_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=SunBlockingGroup1Env.LOADFILE_I3, **kwargs)

class SB1_E1_I4_Env(SB1_E1_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=SunBlockingGroup1Env.LOADFILE_I4, **kwargs)

class SB1_E1_I5_Env(SB1_E1_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=SunBlockingGroup1Env.LOADFILE_I5, **kwargs)