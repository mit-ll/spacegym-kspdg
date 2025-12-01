# Copyright (c) 2025, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# krpc-serverless pytests for orbital euqations of motion written in julia

import numpy as np

from importlib.resources import files

from kspdg.utils.private_src_utils import get_private_src_module_str

# PosixPath to kspdg installation point
KSPDG_INSTALL_PATH = files('kspdg')

# Path to dynamics.jl relative to kspdg install point, accounting for
# python version and OS-specific directories
DYNAMICS_JL_PATH = get_private_src_module_str("kspdg_envs.lbg1")
DYNAMICS_JL_PATH = DYNAMICS_JL_PATH.replace('.','/')
DYNAMICS_JL_PATH = DYNAMICS_JL_PATH.partition('/')[2]

# Join the solve_lbg1.jl relative path to kspdg absolute path
DYNAMICS_JL_PATH = KSPDG_INSTALL_PATH / DYNAMICS_JL_PATH / "dynamics.jl"

from juliacall import Main as jl

def test_dynamics_jl_include():
    """check include of dynamics.jl does not error"""

    # ~~ ACT ~~
    jl.include(str(DYNAMICS_JL_PATH))

def _ensure_dynamics_loaded():
    """Include the Julia file once into Main."""
    if hasattr(jl, "cw_discrete__rhntw"):
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

def test_cw_discrete_shapes_and_real_values():
    dt = 10.0       # seconds
    n = 0.0011      # rad/s in LEO (not Kerbin)

    A, B = cw_discrete_rhntw_py(dt, n)

    assert A.shape == (6, 6)
    assert B.shape == (6, 3)
    assert np.isrealobj(A)
    assert np.isrealobj(B)

def test_cw_discrete_small_dt_approx_identity_and_dtB():
    # For small dt, A ≈ I, B ≈ dt * Bc
    n = 0.0011  # rad/s in LEO (not Kerbin)
    dt = 1e-6   # sec

    A, B = cw_discrete_rhntw_py(dt, n)

    I = np.eye(6)
    np.testing.assert_allclose(A, I, atol=1e-5)

    # Continuous-time Bc bottom 3x3 = I, top = 0
    B_bottom = B[3:, :]
    np.testing.assert_allclose(B_bottom, dt * np.eye(3), atol=1e-5)

    B_top = B[:3, :]
    np.testing.assert_allclose(B_top, 0.0, atol=1e-6)

def _analytic_z_solution(t, n, z0, zdot0):
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