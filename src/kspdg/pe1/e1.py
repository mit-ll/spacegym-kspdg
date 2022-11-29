# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.pe1.base import PursuitEnv

class PursuitEnv_ePassive(PursuitEnv):
    def __init__(self, loadfile: str, **kwargs):
        super().__init__(loadfile=loadfile, **kwargs)

    def evasive_maneuvers(self):
        '''Do not perform evasive maneuvers
        '''
        pass
