# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# kspdg package-internal scripts for creating commmand line entry points for unit tests
# so as not to need a full download/clone of the repository

import pytest
import os

from importlib.resources import files

KSPDG_INSTALL_PATH = files('kspdg')

def serverless_tests():
    """Run tests that do not require connection to krpc server; i.e. ksp does not need to be running."""
    # test_path = os.path.join(os.path.dirname(__file__), "../../../tests/serverless_tests")
    test_path = os.path.join(KSPDG_INSTALL_PATH, "../../tests/serverless_tests")
    pytest.main([test_path])

def lbg1_i2_tests():
    """Run tests that do not require connection to krpc server; i.e. ksp does not need to be running."""
    # test_path = os.path.join(os.path.dirname(__file__), "../../../tests/serverless_tests")
    test_path = os.path.join(KSPDG_INSTALL_PATH, "../../tests/ksp_ingame_tests/test_lbg1_i2.py")
    pytest.main([test_path])