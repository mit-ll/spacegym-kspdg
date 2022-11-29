# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import pytest
import time
import krpc
import numpy as np

from types import SimpleNamespace

import kspdg.utils.utils as U
from kspdg.pursuit_v20220516 import PursuitEnvV20220516

_INIT_LOADFILE = '20220629_PyTest_init'

@pytest.fixture
def pytest_20220629_env():
    '''setup and teardown connection to krpc server running in 20220629_PyTest game file'''
    env = SimpleNamespace()

    # create connection
    env.conn = krpc.connect(name='pytest_client')
    print("Connected to kRPC server")

    # Load save file from start of mission scenario
    env.conn.space_center.load(_INIT_LOADFILE)

    # get vessel objects
    env.vesTest = env.conn.space_center.vessels[0]

    # hand over environment to test function
    yield env

    # close environment connection after test complete
    env.conn.close()

# def test_get_combined_rcs_properties_0(pytest_20220629_env):
#     '''check thrust results in expected deltaV'''
#     # ~~ ARRANGE ~~
#     env = pytest_20220629_env

#     # set burn time and body-relative burn vector for test
#     delta_t = 10.0
#     burn_vec__rhbody = [1,0,0]

#     # get information about celestial body for ease of use
#     cb = env.vesTest.orbit.body

#     # orient craft in prograde direction
#     env.conn.space_center.target_vessel = None
#     env.vesTest.control.sas = True
#     time.sleep(0.1)
#     env.vesTest.control.sas_mode = env.vesTest.control.sas_mode.prograde
#     time.sleep(2.0) # give time for craft to settle

#     # ~~ ACT ~~ 
#     # get rcs properties
#     max_thrust, max_fuel_consumption, specific_impulse = \
#         U.get_rcs_net_directional_properties(env.vesTest, burn_vec__rhbody)

#     # get initial mass and speed of pursuer with respect to inertial (non-rotating) 
#     # celestial body frame (not expressed in any coords because not a vector)
#     m0_test = env.vesTest.mass
#     s0_test_cbci = env.vesTest.flight(cb.non_rotating_reference_frame).speed

#     # activate rcs and 
#     # apply max thrust in forward body direction (prograde because of sas mode) for one second
#     env.vesTest.control.rcs = True
#     env.vesTest.control.forward = burn_vec__rhbody[0]
#     env.vesTest.control.right = burn_vec__rhbody[1]
#     env.vesTest.control.up = -burn_vec__rhbody[2]
#     time.sleep(delta_t)
#     env.vesTest.control.forward = 0.0
#     env.vesTest.control.right = 0.0
#     env.vesTest.control.up = 0.0

#     # measure new mass and speed of pursuer 
#     m1_test = env.vesTest.mass
#     s1_test_cbci = env.vesTest.flight(cb.non_rotating_reference_frame).speed

#     # ~~ ASSERT ~~

#     # check mass delta aligns with fuel consumption
#     delta_m = m0_test - m1_test
#     delta_m_exp = max_fuel_consumption*delta_t
#     assert np.isclose(delta_m, delta_m_exp, rtol=1e-2)



