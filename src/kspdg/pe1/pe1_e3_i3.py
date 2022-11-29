# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.pe1.e3 import PursuitEnv_e3

_INIT_LOADFILE = "pe1_i3_init"

class PursuitEnv_e3_i3_v0(PursuitEnv_e3):
    def __init__(self, **kwargs):
        super().__init__(loadfile=_INIT_LOADFILE, **kwargs)
