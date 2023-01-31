# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Lady-Bandit-Guard environment with passive Lady and Guard and
# "i1" initial orbital conditions

from kspdg.lbg1.lg0 import LBG1_LG0_ParentEnv

_INIT_LOADFILE = "lbg1_i1_init"

class LBG1_LG0_I1_Env(LBG1_LG0_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=_INIT_LOADFILE, **kwargs)