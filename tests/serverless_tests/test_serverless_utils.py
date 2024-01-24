# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
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


if __name__ == "__main__":
    test_convert_lhcbci_and_rhcbci_0()
