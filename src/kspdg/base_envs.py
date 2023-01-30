# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Abstract base classes for KSPDG environment classes

import gymnasium as gym

from abc import ABC

class KSPDGBaseEnv(ABC, gym.Env):
    """ Abstract base class for all KSPDG gym environments
    """

    