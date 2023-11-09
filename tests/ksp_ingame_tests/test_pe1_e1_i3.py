# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

'''
Test that require:
1. KSP to be running
2. Play Mission mode -> pe1_i3
3. kRPC server to be running
'''
import pytest
import time
import numpy as np

import kspdg.utils.constants as C
from kspdg.pe1.e1_envs import PE1_E1_I3_Env

@pytest.fixture
def pe1_e1_i3_env():
    '''setup and teardown of the pursuit-evade env object connected to kRPC server'''
    env = PE1_E1_I3_Env()
    env.reset()
    yield env
    env.close()

def test_observation_dict_list_convert_0(pe1_e1_i3_env):
    '''check that converting between lists and dict to not alter observation'''
    # ~~ ARRANGE ~~
    env = pe1_e1_i3_env

    # collect current observation as list
    obs_list = env.get_observation()

    # ~~ ACT ~~ 
    # convert obs to dict
    obs_dict = env.observation_list_to_dict(obs_list)

    # convert obs back to list
    obs_list_new = env.observation_dict_to_list(obs_dict)

    # ~~ ASSERT ~~
    # check that all observations are equal
    assert np.allclose(obs_list, obs_list_new)

def test_get_info_0(pe1_e1_i3_env):
    '''check that init state get info is as expected'''
    # ~~ ARRANGE ~~

    if pe1_e1_i3_env is None:
        env = PE1_E1_I3_Env()
        env.reset()
    else:
        env = pe1_e1_i3_env

    # ~~ ACT ~~ 
    # time.sleep(1.0)
    # for i in range(10):
        # get info dict
    obs_list = env.get_observation()
    info1 = env.get_info(obs_list, False)
    obs_list = env.get_observation()
    info2 = env.get_info(obs_list, True)

    # ~~ ASSERT ~~
    # check that all observations are equal
    min_closest_approach = 2500
    max_closest_approach = 2800
    assert info1[env.PARAMS.INFO.K_CLOSEST_APPROACH] > min_closest_approach
    assert info1[env.PARAMS.INFO.K_CLOSEST_APPROACH] < max_closest_approach
    min_closest_approach_speed = 7.0
    max_closest_approach_speed = 10.0
    assert info1[env.PARAMS.INFO.K_CLOSEST_APPROACH_SPEED] > min_closest_approach_speed
    assert info1[env.PARAMS.INFO.K_CLOSEST_APPROACH_SPEED] < max_closest_approach_speed
    pursuer_fuel_usage = 0.0
    evader_fuel_usage = 0.0
    assert np.isclose(info1[env.PARAMS.INFO.K_PURSUER_FUEL_USAGE], pursuer_fuel_usage, atol=1.0)
    assert np.isclose(info1[env.PARAMS.INFO.K_EVADER_FUEL_USAGE], evader_fuel_usage, atol=1.0)
    min_closest_approach_time = 0.0
    max_closest_approach_time = 10.0
    assert info1[env.PARAMS.INFO.K_CLOSEST_APPROACH_TIME] > min_closest_approach_time
    assert info1[env.PARAMS.INFO.K_CLOSEST_APPROACH_TIME] < max_closest_approach_time
    # assert info1[env.PARAMS.INFO.K_MIN_POSVEL_PRODUCT] > 7*2500
    # assert info1[env.PARAMS.INFO.K_MIN_POSVEL_PRODUCT] < 8*2800
    assert info1[env.PARAMS.INFO.K_WEIGHTED_SCORE] > (
        (0.1 * min_closest_approach)**2 + 
        (0.5 * min_closest_approach_speed)**1.5 + 
        (0.1 * pursuer_fuel_usage)**1.25 + 
        (0.01 * min_closest_approach_time)**1.0
    )
    assert info1[env.PARAMS.INFO.K_WEIGHTED_SCORE] < (
        (0.1 * max_closest_approach)**2 + 
        (0.5 * max_closest_approach_speed)**1.5 + 
        (0.1 * pursuer_fuel_usage)**1.25 + 
        (0.01 * max_closest_approach_time)**1.0
    )

    env.close()

def test_get_combined_rcs_properties_0(pe1_e1_i3_env):
    '''check thrust maneuvers results in expected deltaV, fuel depletion, and score improvement'''
    # ~~ ARRANGE ~~
    env = pe1_e1_i3_env

    # set burn time and body-relative burn vector for test
    delta_t = 10.0
    burn_vec__rhbody = [1,0,0]

    # get information about celestial body for ease of use
    cb = env.vesPursue.orbit.body

    # orient craft in prograde direction
    env.conn.space_center.target_vessel = None
    env.vesPursue.control.sas = True
    time.sleep(0.1)
    env.vesPursue.control.sas_mode = env.vesPursue.control.sas_mode.prograde
    time.sleep(2.0) # give time for craft to settle

    # ~~ ACT ~~ 
    # get initial mass and speed of pursuer with respect to inertial (non-rotating) 
    # celestial body frame (not expressed in any coords because not a vector)
    # get initial score
    obs0 = env.get_observation()
    info0 = env.get_info(obs0, False)
    m0_pur = env.vesPursue.mass
    s0_pur_cbci = env.vesPursue.flight(cb.non_rotating_reference_frame).speed

    # activate rcs and 
    # apply max thrust in forward body direction (prograde because of sas mode) for one second
    env.vesPursue.control.rcs = True
    env.vesPursue.control.forward = burn_vec__rhbody[0]
    env.vesPursue.control.right = burn_vec__rhbody[1]
    env.vesPursue.control.up = -burn_vec__rhbody[2]
    time.sleep(delta_t)
    env.vesPursue.control.forward = 0.0
    env.vesPursue.control.right = 0.0
    env.vesPursue.control.up = 0.0

    # measure new mass and speed of pursuer 
    obs1 = env.get_observation()
    info1 = env.get_info(obs1, False)
    m1_pur = env.vesPursue.mass
    s1_pur_cbci = env.vesPursue.flight(cb.non_rotating_reference_frame).speed

    # ~~ ASSERT ~~

    # check mass delta aligns with fuel consumption
    delta_m = m0_pur - m1_pur
    delta_m_exp = env.PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_FORWARD * delta_t
    assert np.isclose(delta_m, delta_m_exp, rtol=5e-2)

    # check speed change aligns with expected delta_v
    delta_v = s1_pur_cbci - s0_pur_cbci
    delta_v_exp = env.PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE * C.G0 * np.log(m0_pur/m1_pur)
    assert np.isclose(delta_v, delta_v_exp, rtol=1e-2)

    # check that weighted score has improved by approaching the other agent
    assert info1[env.PARAMS.INFO.K_WEIGHTED_SCORE] < info0[env.PARAMS.INFO.K_WEIGHTED_SCORE] 

def test_convert_rhntw_to_rhpbody_0(pe1_e1_i3_env):
    '''check along-track vec in right-hand NTW frame transforms to forward in right-hand pursuer body coords'''

    # rename for ease of use
    env = pe1_e1_i3_env
    env.conn.space_center.target_vessel = None
    env.vesPursue.control.rcs = False
    time.sleep(0.5)
    v_exp__rhpbody = [1, 0, 0]

    # vector pointing along track
    v__rhntw = [0, 1, 0]
    env.vesPursue.control.sas_mode = env.vesPursue.control.sas_mode.prograde
    time.sleep(0.5)   # give time to re-orient
    v__rhpbody = pe1_e1_i3_env.convert_rhntw_to_rhpbody(v__rhntw)
    assert np.allclose(v__rhpbody, v_exp__rhpbody, atol=1e-2)

    # vector pointing radial out
    v__rhntw = [1, 0, 0]
    env.vesPursue.control.sas_mode = env.vesPursue.control.sas_mode.radial
    time.sleep(2.0)   # give time to re-orient
    v__rhpbody = pe1_e1_i3_env.convert_rhntw_to_rhpbody(v__rhntw)
    assert np.allclose(v__rhpbody, v_exp__rhpbody, atol=1e-2)

    # vector pointing normal
    v__rhntw = [0, 0, 1]
    env.vesPursue.control.sas_mode = env.vesPursue.control.sas_mode.normal
    time.sleep(2.0)   # give time to re-orient
    v__rhpbody = pe1_e1_i3_env.convert_rhntw_to_rhpbody(v__rhntw)
    assert np.allclose(v__rhpbody, v_exp__rhpbody, atol=1e-2)

    # vector pointing retrograde
    v__rhntw = [0, -1, 0]
    env.vesPursue.control.sas_mode = env.vesPursue.control.sas_mode.retrograde
    time.sleep(2.0)   # give time to re-orient
    v__rhpbody = pe1_e1_i3_env.convert_rhntw_to_rhpbody(v__rhntw)
    assert np.allclose(v__rhpbody, v_exp__rhpbody, atol=1e-2)

    # vector pointing in-radial
    v__rhntw = [-1, 0, 0]
    env.vesPursue.control.sas_mode = env.vesPursue.control.sas_mode.anti_radial
    time.sleep(2.0)   # give time to re-orient
    v__rhpbody = pe1_e1_i3_env.convert_rhntw_to_rhpbody(v__rhntw)
    assert np.allclose(v__rhpbody, v_exp__rhpbody, atol=1e-2)

    # vector pointing retrograde
    v__rhntw = [0, 0, -1]
    env.vesPursue.control.sas_mode = env.vesPursue.control.sas_mode.anti_normal
    time.sleep(2.0)   # give time to re-orient
    v__rhpbody = pe1_e1_i3_env.convert_rhntw_to_rhpbody(v__rhntw)
    assert np.allclose(v__rhpbody, v_exp__rhpbody, atol=1e-2)

if __name__ == "__main__":
    test_get_info_0(None)
