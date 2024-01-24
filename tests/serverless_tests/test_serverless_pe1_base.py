# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import pytest
import numpy as np

import kspdg.utils.constants as C
from kspdg.pe1.pe1_base import PursuitEvadeGroup1Env


@pytest.fixture
def pe1_base_env():
    """setup and teardown of the PursuitEnvV20220516 object connected to kRPC server"""
    env = PursuitEvadeGroup1Env(None)
    yield env
