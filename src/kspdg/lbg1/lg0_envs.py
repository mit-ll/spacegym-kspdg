# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Parent and subclasses of all LBG1 environments that use a passive 
# (non-maneuvering) policy for the Lady and Guard vessels

from kspdg.lbg1.lbg1_base import LadyBanditGuardGroup1Env

class LBG1_LG0_ParentEnv(LadyBanditGuardGroup1Env):

    def __init__(self, loadfile: str, **kwargs):
        super().__init__(loadfile=loadfile, **kwargs)

    def lady_guard_policy(self):
        """Lady and guard are passive and do not maneuver"""
        pass

class LBG1_LG0_I1_Env(LBG1_LG0_ParentEnv):
    INIT_LOADFILE = "lbg1_i1_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG0_I1_Env.INIT_LOADFILE, **kwargs)

class LBG1_LG0_I2_Env(LBG1_LG0_ParentEnv):
    INIT_LOADFILE = "lbg1_i2_init"
    def __init__(self, **kwargs):
        super().__init__(loadfile=LBG1_LG0_I2_Env.INIT_LOADFILE, **kwargs)