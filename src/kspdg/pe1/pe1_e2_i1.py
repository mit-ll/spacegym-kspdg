# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.pe1.e2 import PursuitEnv_eRandom

_INIT_LOADFILE = "pe1_i1_init"

class PursuitEnv_eRandom_i1_v0(PursuitEnv_eRandom):
    def __init__(self, **kwargs):
        super().__init__(loadfile=_INIT_LOADFILE, **kwargs)

