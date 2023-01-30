# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Abstract base classes for KSPDG environment classes

import krpc
import gymnasium as gym

from abc import ABC

class KSPDGBaseEnv(ABC, gym.Env):
    """ Abstract base class for all KSPDG gym environments
    """

    def connect_and_load_on_reset(self):
        """Connect to KRPC server and load mission save file 
        """

        # remove prior connection if it exists
        if hasattr(self, 'conn'):
            self.close()

        # establish krpc connect to send remote commands
        self.conn = krpc.connect(name='kspdg_env')
        print("Connected to kRPC server")

        # Load save file from start of mission scenario
        self.conn.space_center.load(self.loadfile)

