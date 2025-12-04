# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# pytest for utils function that can be run without a KSP server running
import pytest
import numpy as np

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

def test_circular_case_basis_alignment():
    # Circular-like: position +x, velocity +y, so:
    # T = +y, W = +z, N = T×W = +x
    r = 7000e3
    v = 7.5e3
    p_ref = np.array([r, 0.0, 0.0])
    v_ref = np.array([0.0, v, 0.0])

    # e_x in inertial should map to +N
    e_x = np.array([1.0, 0.0, 0.0])
    ntw_x = U.convert_rhcbci_to_rhntw(e_x, p_ref, v_ref)
    # e_y → +T, e_z → +W
    e_y = np.array([0.0, 1.0, 0.0])
    ntw_y = U.convert_rhcbci_to_rhntw(e_y, p_ref, v_ref)
    e_z = np.array([0.0, 0.0, 1.0])
    ntw_z = U.convert_rhcbci_to_rhntw(e_z, p_ref, v_ref)

    # Expect identity mapping of basis: ex→[1,0,0], ey→[0,1,0], ez→[0,0,1]
    assert ntw_x.shape == (3,)
    assert np.allclose(ntw_x, [1, 0, 0], atol=1e-12)
    assert np.allclose(ntw_y, [0, 1, 0], atol=1e-12)
    assert np.allclose(ntw_z, [0, 0, 1], atol=1e-12)

def test_non_circular_orthonormal_right_handed():
    # Non-circular: velocity not perpendicular to position
    p_ref = np.array([7000e3, 0.0, 0.0])
    v_ref = np.array([1.0e3, 7.5e3, 0.0])  # has radial + along-track components

    # Recover basis by converting standard basis vectors and stacking rows inverted
    ex_ntw = U.convert_rhcbci_to_rhntw(np.array([1,0,0]), p_ref, v_ref)
    ey_ntw = U.convert_rhcbci_to_rhntw(np.array([0,1,0]), p_ref, v_ref)
    ez_ntw = U.convert_rhcbci_to_rhntw(np.array([0,0,1]), p_ref, v_ref)

    # The conversion is: r_ntw = R * r_inertial, where rows of R are [N; T; W].
    # To inspect R in inertial space, we can solve a small system: R * I = [ex_ntw ey_ntw ez_ntw]
    R = np.column_stack((ex_ntw, ey_ntw, ez_ntw))  # R @ e_i = column i

    # Check orthonormal rows (N,T,W) -> equivalently columns of R^T
    Rt = R.T
    N, T, W = Rt[0], Rt[1], Rt[2]
    assert np.isclose(np.linalg.norm(N), 1.0, atol=1e-12)
    assert np.isclose(np.linalg.norm(T), 1.0, atol=1e-12)
    assert np.isclose(np.linalg.norm(W), 1.0, atol=1e-12)
    assert np.isclose(np.dot(N, T), 0.0, atol=1e-12)
    assert np.isclose(np.dot(N, W), 0.0, atol=1e-12)
    assert np.isclose(np.dot(T, W), 0.0, atol=1e-12)

    # Right-handed: det(R) should be +1
    detR = np.linalg.det(R)
    assert np.isclose(detR, 1.0, atol=1e-12)

def test_vector_projection_consistency():
    # Compare direct projection with function output
    p_ref = np.array([7000e3, 0.0, 0.0])
    v_ref = np.array([0.5e3, 7.5e3, 0.2e3])

    # Choose an arbitrary vector
    r_tar = np.array([2.0, -3.0, 5.0])

    # Function output
    r_ntw = U.convert_rhcbci_to_rhntw(r_tar, p_ref, v_ref)

    # Manually reconstruct basis and project to check numerics
    t_hat = v_ref / np.linalg.norm(v_ref)
    w_hat = np.cross(p_ref, v_ref); w_hat /= np.linalg.norm(w_hat)
    n_hat = np.cross(t_hat, w_hat)
    R = np.vstack((n_hat, t_hat, w_hat))
    r_ntw_manual = R @ r_tar

    assert np.allclose(r_ntw, r_ntw_manual, atol=1e-12)

def test_raises_on_zero_position():
    p_ref = np.array([0.0, 0.0, 0.0])
    v_ref = np.array([0.0, 7.5, 0.0])
    with pytest.raises(ValueError):
        U.convert_rhcbci_to_rhntw(np.array([1,2,3]), p_ref, v_ref)

    with pytest.raises(ValueError):
        U.convert_rhntw_to_rhcbci(np.array([1,2,3]), p_ref, v_ref)

def test_raises_on_zero_velocity():
    p_ref = np.array([7000e3, 0.0, 0.0])
    v_ref = np.array([0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        U.convert_rhcbci_to_rhntw(np.array([1,2,3]), p_ref, v_ref)

    with pytest.raises(ValueError):
        U.convert_rhntw_to_rhcbci(np.array([1,2,3]), p_ref, v_ref)

def test_raises_on_colinear_r_and_v():
    # v parallel to r => cross(r,v)=0 => cannot define W
    p_ref = np.array([7000e3, 0.0, 0.0])
    v_ref = np.array([1.0, 0.0, 0.0])  # colinear with p_ref
    with pytest.raises(ValueError):
        U.convert_rhcbci_to_rhntw(np.array([1,2,3]), p_ref, v_ref)

    with pytest.raises(ValueError):
        U.convert_rhntw_to_rhcbci(np.array([1,2,3]), p_ref, v_ref)

@pytest.mark.parametrize(
    "r_vec, p_ref, v_ref",
    [
        # 1) Simple circular-ish orbit in xy-plane
        (
            [1.2, -3.4, 5.6],
            [7000e3, 0.0, 0.0],          # position along +x
            [0.0, 7.5e3, 0.0],           # velocity along +y
        ),
        # 2) Inclined orbit, generic target vector
        (
            [-1200.0, 3400.0, 5600.0],
            [6500e3, 1000e3, 500e3],     # off-equatorial
            [-100.0, 7450.0, 1500.0],    # velocity not orthogonal to position
        ),
        # 3) Highly non-circular reference, target with mixed components
        (
            [10.0, -20.0, 30.0],
            [8000e3, -2000e3, 1000e3],
            [2000.0, 6500.0, -500.0],
        ),
        # 4) Another generic case with non-zero z velocity
        (
            [-5.0, 2.0, 11.0],
            [7200e3, 500e3, -300e3],
            [500.0, 7600.0, 900.0],
        ),
    ],
)
def test_rhcbci_rhntw_roundtrip(r_vec, p_ref, v_ref):
    r_vec = np.array(r_vec, dtype=float)
    p_ref = np.array(p_ref, dtype=float)
    v_ref = np.array(v_ref, dtype=float)

    r_ntw = U.convert_rhcbci_to_rhntw(r_vec, p_ref, v_ref)
    r_back = U.convert_rhntw_to_rhcbci(r_ntw, p_ref, v_ref)

    # Round-trip should recover original vector
    assert np.allclose(r_back, r_vec, atol=1e-9)

if __name__ == "__main__":
    test_convert_lhcbci_and_rhcbci_0()
