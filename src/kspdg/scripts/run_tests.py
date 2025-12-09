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
    """Run tests that do not require connection to krpc server; i.e. ksp does not need to be running"""
    test_path = os.path.join(KSPDG_INSTALL_PATH, "tests/serverless_tests")
    pytest.main([test_path])

def lbg1_i2_tests():
    """Run tests in lbg1_i2 mission"""
    test_path = os.path.join(KSPDG_INSTALL_PATH, "tests/ksp_ingame_tests/test_lbg1_i2.py")
    pytest.main([test_path])

def pe1_i3_tests():
    """Run tests in pe1_i3 mission"""
    test_path = os.path.join(KSPDG_INSTALL_PATH, "tests/ksp_ingame_tests/test_pe1_i3.py")
    pytest.main([test_path])

def sb1_i5_tests():
    """Run tests in sb1_i5 mission"""
    test_path = os.path.join(KSPDG_INSTALL_PATH, "tests/ksp_ingame_tests/test_sb1_i5.py")
    pytest.main([test_path])

def aiaa_competition_tests():
    """Run small subset of tests for AIAA competition events"""
    lbg1_test_path = os.path.join(KSPDG_INSTALL_PATH, "tests/ksp_ingame_tests/test_lbg1_i2.py")
    pytest.main([
        lbg1_test_path+"::test_physics_range_extender_1",
        lbg1_test_path+"::test_smoketest_lg6"
        ])