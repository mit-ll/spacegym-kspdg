# Copyright (c) 2025, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# krpc-serverless pytests for orbital euqations of motion written in julia

import numpy as np
import pytest

from importlib.resources import files

from kspdg.utils.private_src_utils import get_private_src_module_str
import kspdg.utils.constants as C

# suppress warnings from mismatch between numpy and juliacall array copy
pytestmark = pytest.mark.filterwarnings(
    "ignore:.*__array__ implementation doesn't accept a copy keyword.*:DeprecationWarning"
)

# PosixPath to kspdg installation point
KSPDG_INSTALL_PATH = files('kspdg')

# Path to dynamics.jl relative to kspdg install point, accounting for
# python version and OS-specific directories
DYNAMICS_JL_PATH = get_private_src_module_str("kspdg_envs.lbg1")
DYNAMICS_JL_PATH = DYNAMICS_JL_PATH.replace('.','/')
DYNAMICS_JL_PATH = DYNAMICS_JL_PATH.partition('/')[2]

# Join the solve_lbg1.jl relative path to kspdg absolute path
DYNAMICS_JL_PATH = KSPDG_INSTALL_PATH / DYNAMICS_JL_PATH / "dynamics.jl"

# mean motion of 100K circular orbit around kerbing
N_KERBIN_100K = np.round(np.sqrt(C.KERBIN.MU/((C.KERBIN.RADIUS + 1e5)**3)), 4) 

from juliacall import Main as jl

def test_dynamics_jl_include():
    """check include of dynamics.jl does not error"""

    # ~~ ACT ~~
    jl.include(str(DYNAMICS_JL_PATH))

def _ensure_dynamics_loaded():
    """Include the Julia file once into Main."""
    if hasattr(jl, "cw_discrete__rhntw") and hasattr(jl, "lbg_cw_discrete_eom__rhntw"):
        return

    jl.include(str(DYNAMICS_JL_PATH))


def cw_discrete_rhntw_py(dt: float, n: float):
    """
    Python wrapper around Julia cw_discrete__rhntw(dt, n).
    Returns A, B as NumPy arrays.
    """
    _ensure_dynamics_loaded()
    A_jl, B_jl = jl.cw_discrete__rhntw(dt, n)
    A = np.array(A_jl)
    B = np.array(B_jl)
    return A, B

def lbg_cw_discrete_eom_rhntw_py(dt: float, n: float):
    """Python wrapper around Julia lbg_cw_discrete_eom__rhntw(dt, n)."""
    _ensure_dynamics_loaded()
    A_jl, B_jl = jl.lbg_cw_discrete_eom__rhntw(dt, n)
    return np.array(A_jl), np.array(B_jl)

def test_cw_discrete_shapes_and_real_values():
    dt = 10.0       # seconds
    n = 0.0011      # rad/s in LEO (not Kerbin)

    A, B = cw_discrete_rhntw_py(dt, n)

    assert A.shape == (6, 6)
    assert B.shape == (6, 3)
    assert np.isrealobj(A)
    assert np.isrealobj(B)

@pytest.mark.parametrize(
    "dt,n",
    [
        (1e-6, 0.0011),         # Earth LEO orbital rate
        (1e-6, N_KERBIN_100K)   # Kerbin 100K orbital rate
    ],
)
def test_cw_discrete_small_dt_approx_identity_and_dtB(dt, n):
    # For small dt, A ≈ I, B ≈ dt * Bc
    A, B = cw_discrete_rhntw_py(dt, n)

    I = np.eye(6)
    np.testing.assert_allclose(A, I, atol=1e-5)

    # Continuous-time Bc bottom 3x3 = I, top = 0
    B_bottom = B[3:, :]
    np.testing.assert_allclose(B_bottom, dt * np.eye(3), atol=1e-5)

    B_top = B[:3, :]
    np.testing.assert_allclose(B_top, 0.0, atol=1e-6)

def _analytic_z_solution(t, n, z0, zdot0):
    """
    Unforced analytic solution to z-axis motion
    
    :param t: time [s]
    :param n: mean motion [rad/sec]
    :param z0: initial z-position of deputy spacecraft [m]
    :param zdot0: initial z-velociity of deputy spacecraft [m/s]
    """
    z = z0 * np.cos(n * t) + (zdot0 / n) * np.sin(n * t)
    zdot = -n * z0 * np.sin(n * t) + zdot0 * np.cos(n * t)
    return z, zdot

def test_cw_z_axis_matches_analytic_sho():
    # CW equations should have z-axis motion as a decoupled simple harmonic oscillator
    # check this against an analytical model of a SHO
    n = 0.0011 # rad/s in LEO (not Kerbin)
    dt = 10.0  # seconds
    num_steps = 50

    A, B = cw_discrete_rhntw_py(dt, n)

    # Initial condition: pure z/zdot, no x/y components, zero input
    z0 = 100.0     # meters
    zdot0 = 0.5    # m/s
    x = np.array([0.0, 0.0, z0, 0.0, 0.0, zdot0])
    u = np.zeros(3)

    for k in range(num_steps + 1):
        t = k * dt
        z_analytic, zdot_analytic = _analytic_z_solution(t, n, z0, zdot0)
        z_num = x[2]
        zdot_num = x[5]

        assert np.allclose(z_num, z_analytic, atol=1e-6)
        assert np.allclose(zdot_num, zdot_analytic, atol=1e-6)

        # Step forward
        x = A @ x + B @ u

def _analytic_z_forced(t, n, az):
    """
    Analytic solution of motion on z-axis under constant acceleration
    
    :param t: time [s]
    :param n: mean motion [rad/sec]
    :param az: constant acceleration in z-axis [m/s/s]
    """
    z_p = az / n**2
    z = z_p * (1 - np.cos(n * t))
    zdot = (az / n) * np.sin(n * t)
    return z, zdot

@pytest.mark.parametrize(
    "dt,n,az",
    [
        (10.0, 0.0011, 1e-4),
        (60.0, 0.0011, 1e-4),
        (30.0, 0.0005, 5e-5),
        (5.0, N_KERBIN_100K, 1.0)
    ],
)
def test_z_axis_forced_response_matches_analytic(dt, n, az):
    """Constant a_z input should match analytic z(t), zdot(t) after one step."""
    A, B = cw_discrete_rhntw_py(dt, n)

    # initial state: only z,ż potentially nonzero; but here both zero
    x = np.zeros(6)
    u = np.array([0.0, 0.0, az])

    x_next = A @ x + B @ u

    z_num = x_next[2]
    zdot_num = x_next[5]

    z_analytic, zdot_analytic = _analytic_z_forced(dt, n, az)

    assert np.allclose(z_num, z_analytic, atol=1e-8, rtol=1e-8)
    assert np.allclose(zdot_num, zdot_analytic, atol=1e-8, rtol=1e-8)

def test_lbg_cw_shapes_and_real_values():
    dt = 10.0
    n = 0.0011

    A, B = lbg_cw_discrete_eom_rhntw_py(dt, n)

    # A: 12x12, B: 12x6
    assert A.shape == (12, 12)
    assert B.shape == (12, 6)

    # Real-valued
    assert np.isrealobj(A)
    assert np.isrealobj(B)

@pytest.mark.parametrize(
    "dt,n",
    [
        (1.0, 0.0011),
        (10.0, 0.0011),
        (30.0, 0.0011),
        (5.0, N_KERBIN_100K)
    ],
)
def test_lbg_cw_block_structure_matches_single_satellite(dt, n):
    A_cw, B_cw = cw_discrete_rhntw_py(dt, n)
    A, B = lbg_cw_discrete_eom_rhntw_py(dt, n)

    # Top-left 6x6 = A_cw (bandit)
    assert np.allclose(A[0:6, 0:6], A_cw)

    # Bottom-right 6x6 = A_cw (guard)
    assert np.allclose(A[6:12, 6:12], A_cw)

    # Off-diagonal blocks ≈ 0
    assert np.allclose(A[0:6, 6:12], 0.0)
    assert np.allclose(A[6:12, 0:6], 0.0)

    # B: bandit part (rows 0:6, cols 0:3) = B_cw
    assert np.allclose(B[0:6, 0:3], B_cw)

    # B: guard part (rows 6:12, cols 3:6) = B_cw
    assert np.allclose(B[6:12, 3:6], B_cw)

    # The "cross" blocks of B should be zero
    assert np.allclose(B[0:6, 3:6], 0.0)  # bandit unaffected by guard input
    assert np.allclose(B[6:12, 0:3], 0.0)  # guard unaffected by bandit input

def test_lbg_cw_bandit_guard_decoupled_dynamics():
    dt = 10.0
    n = N_KERBIN_100K

    A_cw, B_cw = cw_discrete_rhntw_py(dt, n)
    A, B = lbg_cw_discrete_eom_rhntw_py(dt, n)

    # Simple deterministic initial states
    x_b = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])   # bandit
    x_g = np.array([-1.0, -2.0, -3.0, -0.1, -0.2, -0.3])  # guard

    # Stack into full state x ∈ R^12
    x = np.concatenate([x_b, x_g])

    # Inputs: only guard has control, bandit input = 0
    u_b = np.zeros(3)
    u_g = np.array([0.01, -0.02, 0.03])
    u = np.concatenate([u_b, u_g])  # [a_x_b, a_y_b, a_z_b, a_x_g, a_y_g, a_z_g]

    # One-step propagation
    x_next = A @ x + B @ u
    x_b_next = x_next[0:6]
    x_g_next = x_next[6:12]

    # Expected bandit next-state: purely A_cw * x_b (no effect from u_g)
    x_b_expected = A_cw @ x_b

    # Expected guard next-state: A_cw * x_g + B_cw * u_g
    x_g_expected = A_cw @ x_g + B_cw @ u_g

    assert np.allclose(x_b_next, x_b_expected, atol=1e-10, rtol=1e-10)
    assert np.allclose(x_g_next, x_g_expected, atol=1e-10, rtol=1e-10)