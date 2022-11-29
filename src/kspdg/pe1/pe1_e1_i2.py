# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.pe1.e1 import PursuitEnv_ePassive

_INIT_LOADFILE = "pe1_i2_init"

class PursuitEnv_ePassive_i2_v0(PursuitEnv_ePassive):
    def __init__(self, **kwargs):
        super().__init__(loadfile=_INIT_LOADFILE, **kwargs)

