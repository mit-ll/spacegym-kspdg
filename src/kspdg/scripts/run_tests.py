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
    """Run the tests for your package."""
    # test_path = os.path.join(os.path.dirname(__file__), "../../../tests/serverless_tests")
    test_path = os.path.join(KSPDG_INSTALL_PATH, "../../tests/serverless_tests")
    pytest.main([test_path])