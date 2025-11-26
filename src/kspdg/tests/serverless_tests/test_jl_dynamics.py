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
    n = 0.0011      # rad/s

    A, B = cw_discrete_rhntw_py(dt, n)

    assert A.shape == (6, 6)
    assert B.shape == (6, 3)
    assert np.isrealobj(A)
    assert np.isrealobj(B)