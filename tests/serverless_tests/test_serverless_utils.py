# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# pytest for utils function that can be run without a KSP server running
import numpy as np

import kspdg.utils.constants as C
import kspdg.utils.utils as U

def test_convert_lhntw_and_rhntw_0():
    '''converting from left and right NTW frames result in same vector'''

    # ~~ ARRANGE ~~
    # a random but fixed vector
    v0__rhntw = [ 66.26946452, -24.70725379, -97.0466488 ]

    # ~~ ACT ~~
    v1__lhntw = U.convert_rhntw_to_lhntw(v0__rhntw)
    v2__rhntw = U.convert_lhntw_to_rhntw(v1__lhntw)

    # ~~ ASSERT ~~
    assert np.allclose(v0__rhntw, v2__rhntw)

def test_convert_lhbody_and_rhbody_0():
    '''converting between left and right body frames result in same vector'''

    # ~~ ARRANGE ~~
    # a random but fixed vector
    v0__rhbody = [ 66.26946452, -24.70725379, -97.0466488 ]

    # ~~ ACT ~~
    v1__lhbody = U.convert_rhbody_to_lhbody(v0__rhbody)
    v2__rhbody = U.convert_lhbody_to_rhbody(v1__lhbody)

    # ~~ ASSERT ~~
    assert np.allclose(v0__rhbody, v2__rhbody)

def test_convert_lhcbci_and_rhcbci_0():
    '''converting between left and right body frames result in same vector'''

    # ~~ ARRANGE ~~
    # a random but fixed vector
    v0__rhcbci = [-43.99569368, -74.06025274,  84.84426997]

    # ~~ ACT ~~
    v1__lhcbci = U.convert_rhcbci_to_lhcbci(v0__rhcbci)
    v2__rhcbci = U.convert_lhcbci_to_rhcbci(v1__lhcbci)

    # ~~ ASSERT ~~
    assert np.allclose(v0__rhcbci, v2__rhcbci)

def test_propagate_orbit_0():
    """check circular orbit propagates back to init state in one period"""

    # ~~ ARRANGE ~~
    p0__rhcbci = np.array([1.5e5+C.KERBIN.RADIUS, 0, 0])
    r0 = np.linalg.norm(p0__rhcbci)
    v0__rhcbci = np.array([0, 2170.0, 0])
    T = 2 * np.pi * np.sqrt(r0**3/C.KERBIN.MU)

    # ~~ ACT ~~
    # propagate orbit
    pf__rhcbci, vf__rhcbci = U.propagate_orbit_tof(p0__rhcbci, v0__rhcbci, T)

    # ~~ ASSERT ~~
    assert np.allclose(p0__rhcbci, pf__rhcbci, rtol=1e-2, atol=1e3)
    assert np.allclose(v0__rhcbci, vf__rhcbci, rtol=1e-2, atol=1e1)

def test_solve_lambert_0():
    """check circular orbit propagation and lambert target solution match"""

    # ~~ ARRANGE ~~
    # two points on circular orbit with quarter-period separation
    p0__rhcbci = np.array([1.5e5+C.KERBIN.RADIUS, 0, 0])
    pf__rhcbci = np.array([0, 1.5e5+C.KERBIN.RADIUS, 0])
    r0 = np.linalg.norm(p0__rhcbci)
    T = 2 * np.pi * np.sqrt(r0**3/C.KERBIN.MU) / 4
    v0exp__rhcbci = np.array([0, 2170.0, 0])
    vfexp__rhcbci = np.array([-2170.0, 0, 0])

    # ~~ ACT ~~
    # solve lambert's problem
    v0__rhcbci, vf__rhcbci = U.solve_lambert(p0__rhcbci, pf__rhcbci, T)

    # ~~ ASSERT ~~
    assert np.allclose(v0exp__rhcbci, v0__rhcbci, rtol=1e-3)
    assert np.allclose(vfexp__rhcbci, vf__rhcbci, rtol=1e-3)

def test_estimate_capture_dv_0():
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
    dv0, dvf = U.estimate_capture_dv(p0_prs, v0_prs, p0_evd, v0_evd, T)

    # ~~ ASSERT ~~
    assert np.isclose(dv0, 0.0)
    assert np.isclose(dvf, 0.0)

def test_estimate_capture_dv_1():
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
    dv0, dvf = U.estimate_capture_dv(p0_prs, v0_prs, p0_evd, v0_evd, T)

    # ~~ ASSERT ~~
    assert np.isclose(dv0, 0.0, atol=1.0)

    vfexp_prs = np.array([-2170.0, 0, 0])
    vfexp_evd = np.array([0, 0, -2170.0])
    dvfexp = np.linalg.norm(vfexp_prs-vfexp_evd)
    assert np.isclose(dvf, dvfexp, atol=1.0)

if __name__ == "__main__":
    test_convert_lhcbci_and_rhcbci_0()
