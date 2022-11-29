# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import pytest
import numpy as np
from kspdg.pe1.base import PursuitEnv

@pytest.fixture
def pe1_base_env():
    """setup and teardown of the PursuitEnvV20220516 object connected to kRPC server"""
    env = PursuitEnv(None)
    yield env


def test_propagate_orbit_0(pe1_base_env):
    """check circular orbit propagates back to init state in one period"""

    # ~~ ARRANGE ~~
    p0__rhcbci = np.array([1.5e5+pe1_base_env.PARAMS.KERBIN.RADIUS, 0, 0])
    r0 = np.linalg.norm(p0__rhcbci)
    v0__rhcbci = np.array([0, 2170.0, 0])
    T = 2 * np.pi * np.sqrt(r0**3/pe1_base_env.PARAMS.KERBIN.MU)

    # ~~ ACT ~~
    # propagate orbit
    pf__rhcbci, vf__rhcbci = pe1_base_env.propagate_orbit(p0__rhcbci, v0__rhcbci, T)

    # ~~ ASSERT ~~
    assert np.allclose(p0__rhcbci, pf__rhcbci, rtol=1e-2, atol=1e3)
    assert np.allclose(v0__rhcbci, vf__rhcbci, rtol=1e-2, atol=1e1)

def test_solve_lambert_0(pe1_base_env):
    """check circular orbit propagation and lambert target solution match"""

    # ~~ ARRANGE ~~
    # two points on circular orbit with quarter-period separation
    p0__rhcbci = np.array([1.5e5+pe1_base_env.PARAMS.KERBIN.RADIUS, 0, 0])
    pf__rhcbci = np.array([0, 1.5e5+pe1_base_env.PARAMS.KERBIN.RADIUS, 0])
    r0 = np.linalg.norm(p0__rhcbci)
    T = 2 * np.pi * np.sqrt(r0**3/pe1_base_env.PARAMS.KERBIN.MU) / 4
    v0exp__rhcbci = np.array([0, 2170.0, 0])
    vfexp__rhcbci = np.array([-2170.0, 0, 0])

    # ~~ ACT ~~
    # solve lambert's problem
    v0__rhcbci, vf__rhcbci = pe1_base_env.solve_lambert(p0__rhcbci, pf__rhcbci, T)

    # ~~ ASSERT ~~
    assert np.allclose(v0exp__rhcbci, v0__rhcbci, rtol=1e-3)
    assert np.allclose(vfexp__rhcbci, vf__rhcbci, rtol=1e-3)

def test_estimate_capture_dv_0(pe1_base_env):
    """check capture dv of the same circular orbit is zero """

    # ~~ ARRANGE ~~
    # two points on circular orbit with quarter-period separation
    p0_prs = np.array([1.5e5+pe1_base_env.PARAMS.KERBIN.RADIUS, 0, 0])
    p0_evd = np.array([1.5e5+pe1_base_env.PARAMS.KERBIN.RADIUS, 0, 0])
    pf__rhcbci = np.array([0, 1.5e5+pe1_base_env.PARAMS.KERBIN.RADIUS, 0])
    r0 = np.linalg.norm(p0_prs)
    T = 2 * np.pi * np.sqrt(r0**3/pe1_base_env.PARAMS.KERBIN.MU) / 4
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
    p0_prs = np.array([1.5e5+pe1_base_env.PARAMS.KERBIN.RADIUS, 0, 0])
    p0_evd = np.array([0, 0, 1.5e5+pe1_base_env.PARAMS.KERBIN.RADIUS])
    pf__rhcbci = np.array([0, 1.5e5+pe1_base_env.PARAMS.KERBIN.RADIUS, 0])
    r0 = np.linalg.norm(p0_prs)
    T = 2 * np.pi * np.sqrt(r0**3/pe1_base_env.PARAMS.KERBIN.MU) / 4
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
