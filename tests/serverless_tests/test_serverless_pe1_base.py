# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import pytest
import numpy as np

import kspdg.utils.constants as C
from kspdg.pe1.base import PursuitEnv


@pytest.fixture
def pe1_base_env():
    """setup and teardown of the PursuitEnvV20220516 object connected to kRPC server"""
    env = PursuitEnv(None)
    yield env

def test_estimate_capture_dv_0(pe1_base_env):
    """check capture dv of the same circular orbit is zero """

    # ~~ ARRANGE ~~
    # two points on circular orbit with quarter-period separation
    p0_prs = np.array([1.5e5+C.KERBIN.RADIUS, 0, 0])
    p0_evd = np.array([1.5e5+C.KERBIN.RADIUS, 0, 0])
    pf__rhcbci = np.array([0, 1.5e5+C.KERBIN.RADIUS, 0])
    r0 = np.linalg.norm(p0_prs)
    T = 2 * np.pi * np.sqrt(r0**3/C.KERBIN.MU) / 4
    v0_prs = np.array([0, 2170.0, 0])
    v0_evd = np.array([0, 2170.0, 0])

    # ~~ ACT ~~
    # solve lambert's problem
    dv0, dvf = pe1_base_env.estimate_capture_dv(p0_prs, v0_prs, p0_evd, v0_evd, T)

    # ~~ ASSERT ~~
    assert np.isclose(dv0, 0.0)
    assert np.isclose(dvf, 0.0)

def test_estimate_capture_dv_1(pe1_base_env):
    """check capture dv of the same circular orbit, relatively inclined by 90 deg, is zero """

    # ~~ ARRANGE ~~
    # two points on circular orbit with quarter-period separation
    p0_prs = np.array([1.5e5+C.KERBIN.RADIUS, 0, 0])
    p0_evd = np.array([0, 0, 1.5e5+C.KERBIN.RADIUS])
    pf__rhcbci = np.array([0, 1.5e5+C.KERBIN.RADIUS, 0])
    r0 = np.linalg.norm(p0_prs)
    T = 2 * np.pi * np.sqrt(r0**3/C.KERBIN.MU) / 4
    v0_prs = np.array([0, 2170.0, 0])
    v0_evd = np.array([0, 2170.0, 0])

    # ~~ ACT ~~
    # solve lambert's problem
    dv0, dvf = pe1_base_env.estimate_capture_dv(p0_prs, v0_prs, p0_evd, v0_evd, T)

    # ~~ ASSERT ~~
    assert np.isclose(dv0, 0.0, atol=1.0)

    vfexp_prs = np.array([-2170.0, 0, 0])
    vfexp_evd = np.array([0, 0, -2170.0])
    dvfexp = np.linalg.norm(vfexp_prs-vfexp_evd)
    assert np.isclose(dvf, dvfexp, atol=1.0)
