# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Parent class of all LBG1 environments that use a passive (non-maneuvering)
# policy for the Lady and Guard vessels

from kspdg.lbg1.lbg1_base import LadyBanditGuardGroup1Env

class LBG1_LG0_ParentEnv(LadyBanditGuardGroup1Env):

    def __init__(self, loadfile: str, **kwargs):
        super().__init__(loadfile=loadfile, **kwargs)

    def lady_guard_policy(self):
        """Lady and guard are passive and do not maneuver"""
        pass