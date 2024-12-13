# pytest for julia calls that can be run without a KSP server running

import pytest
import numpy as np

from pathlib import Path

def test_juliacall_import():
    """check if juliacall has been installed correctly"""
    from juliacall import Main as jl

def test_using_ilqgames_svector():
    """check ilqgames.jl has been installed correctly, basic SVector usage"""

    # ~~ ARRANGE ~~
    from juliacall import Main as jl
    from juliacall import JuliaError

    # ~~ ACT ~~
    jl.seval("using iLQGames: SVector")

    # create a static arrary in 
    x = jl.seval("SVector(1.1,2.2,3.3)")

    # ~~ ASSERT ~~
    with pytest.raises(JuliaError):
        # x is a static array in julia, i.e. immutable, so this should error
        x[0] = 10

    with pytest.raises(IndexError):
        # Julia is 1-indexed, but converted to 0-index for python object x, so this should error
        x[3]

def test_julia_script_and_functions():
    """check that julia scripts and functions are callable from python"""

    # ~~ ARRANGE ~~
    from juliacall import Main as jl

    # ~~ ACT ~~
    example_script_jl_path = Path(__file__).with_name("example_script.jl")
    s = jl.include(str(example_script_jl_path))

    # call the foo function
    x2 = jl.foo(4)

    # call the array sum with python ndarray
    xsum = jl.foo_array_sum(np.array([2, 3, 5]))

    # ~~ ASSERT ~~
    assert s == "Hi! I'm a Julia script (even though you might be calling me from python)!"
    assert np.isclose(x2, 8)
    assert np.isclose(xsum, 10)

