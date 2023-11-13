# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

"""
Test that require:
1. KSP to be running
2. Play Mission mode -> lbg_i2
3. kRPC server to be running
"""
import pytest
import time
import numpy as np

import kspdg.utils.constants as C
from kspdg.lbg1.lg0_envs import LBG1_LG0_I2_Env

@pytest.fixture
def lbg1_lg0_i2_env():
    """setup and teardown of the PursuitEnvV20220516 object connected to kRPC server"""
    env = LBG1_LG0_I2_Env()
    env.reset()
    yield env
    env.close()

def test_smoketest_0(lbg1_lg0_i2_env):
    """Simply ensure no errors are thrown from starting environment"""
    pass

def test_observation_0(lbg1_lg0_i2_env):
    """Check that observation produces reasonable values"""

    # ~~~ ARRANGE ~~~

    if lbg1_lg0_i2_env is None:
        env = LBG1_LG0_I2_Env()
        env.reset()
    else:
        env = lbg1_lg0_i2_env

    # ~~~ ACT ~~~

    # Wait a little while to let any reset maneuvers to settle
    time.sleep(0.2)

    # query observation
    obs = env.get_observation()

    # ~~~ ASSERT ~~~

    # check consistency of size of observation
    assert len(obs) == env.PARAMS.OBSERVATION.LEN
    assert obs.shape == env.observation_space.shape

    # mission time: startup should take less than 5 seconds
    exp_max_startup = 10.0  # [sec]
    assert obs[0] < exp_max_startup

    # bandit mass
    mass_eps = 10.0 # [kg]
    exp_mass = 6.685e3  # [kg] expected mass of bandit vehicle
    exp_prop_mass = 1.2e3   # [kg] expect propellant mass in bandit
    assert obs[1] > exp_mass - mass_eps
    assert obs[1] < exp_mass + mass_eps
    assert obs[2] > exp_prop_mass - mass_eps
    assert obs[2] < exp_prop_mass + mass_eps

    # bandit position checks
    exp_dist_b_cb = 7.5e5   # [m] expected distance of bandit from center of central body
    dist_b_cb_eps = 1e2     # [m] margin of error on bandit position
    mes_pos_b_cb__rhcbci = obs[3:6] # [m] position of bandit wrt central body expressed in rhcbci coords as measured
    mes_dist_b_cb = np.linalg.norm(mes_pos_b_cb__rhcbci)    # [m] distance of bandit from center of body as measured
    assert mes_dist_b_cb > exp_dist_b_cb - dist_b_cb_eps
    assert mes_dist_b_cb < exp_dist_b_cb + dist_b_cb_eps

    # bandit velocity checks
    exp_spd_b_cb = np.sqrt(C.KERBIN.MU/exp_dist_b_cb)  # [m/s] expected speed of bandit wrt central body (circular orbit)
    spd_b_cb_eps = 10.0 # [m/s] margin of error on bandit speed
    mes_vel_b_cb__rhcbci = obs[6:9] # [m/s] velocity of bandit wrt central body expressed in rhcbci coords as measured
    mes_spd_b_cb = np.linalg.norm(mes_vel_b_cb__rhcbci)  # [m/s] speed of bandit wrt central body as measured
    assert mes_spd_b_cb > exp_spd_b_cb - spd_b_cb_eps
    assert mes_spd_b_cb < exp_spd_b_cb + spd_b_cb_eps

    # lady position checks
    exp_dist_l_cb = 7.5e5   # [m] expected distance of lady from center of central body
    dist_l_cb_eps = 1e2     # [m] margin of error on lady position
    mes_pos_l_cb__rhcbci = obs[9:12]    # [m] position of lady wrt central body expressed in rhcbci coords as measured
    mes_dist_l_cb = np.linalg.norm(mes_pos_l_cb__rhcbci)    # [m] distance of lady from center of body as measured
    assert mes_dist_l_cb > exp_dist_l_cb - dist_l_cb_eps
    assert mes_dist_l_cb < exp_dist_l_cb + dist_l_cb_eps

    # lady velocity checks
    exp_spd_l_cb = np.sqrt(C.KERBIN.MU/exp_dist_l_cb)  # [m/s] expected speed of guard wrt central body (circular orbit)
    spd_l_cb_eps = 10.0 # [m/s] margin of error on guard speed
    mes_vel_l_cb__rhcbci = obs[12:15] # [m/s] velocity of guard wrt central body expressed in rhcbci coords as measured
    mes_spd_l_cb = np.linalg.norm(mes_vel_l_cb__rhcbci)  # [m/s] speed of guard wrt central body as measured
    assert mes_spd_l_cb > exp_spd_l_cb - spd_l_cb_eps
    assert mes_spd_l_cb < exp_spd_l_cb + spd_l_cb_eps

    # guard position checks
    exp_dist_g_cb = 7.5e5   # [m] expected distance of guard from center of central body
    dist_g_cb_eps = 1e2     # [m] margin of error on guard position
    mes_pos_g_cb__rhcbci = obs[15:18]    # [m] position of guard wrt central body expressed in rhcbci coords as measured
    mes_dist_g_cb = np.linalg.norm(mes_pos_g_cb__rhcbci)    # [m] distance of lady from center of body as measured
    assert mes_dist_g_cb > exp_dist_g_cb - dist_g_cb_eps
    assert mes_dist_g_cb < exp_dist_g_cb + dist_g_cb_eps

    # guard velocity checks
    exp_spd_g_cb = np.sqrt(C.KERBIN.MU/exp_dist_g_cb)  # [m/s] expected speed of guard wrt central body (circular orbit)
    spd_g_cb_eps = 10.0 # [m/s] margin of error on guard speed
    mes_vel_g_cb__rhcbci = obs[18:] # [m/s] velocity of guard wrt central body expressed in rhcbci coords as measured
    mes_spd_g_cb = np.linalg.norm(mes_vel_g_cb__rhcbci)  # [m/s] speed of guard wrt central body as measured
    assert mes_spd_g_cb > exp_spd_g_cb - spd_g_cb_eps
    assert mes_spd_g_cb < exp_spd_g_cb + spd_g_cb_eps

def test_step_action_space_0(lbg1_lg0_i2_env):
    '''check that step accepts various action input formats without error'''
    # ~~~ ARRANGE ~~~

    if lbg1_lg0_i2_env is None:
        env = LBG1_LG0_I2_Env()
        env.reset()
    else:
        env = lbg1_lg0_i2_env

    # ~~ ACT ~~ 
    env.step([1.0, 0, 0, 1.0])
    env.step({
        "burn_vec":[1.0, 0, 0, 1.0],
        "ref_frame":0
    })
    env.step({
        "burn_vec":[1.0, 0, 0, 1.0],
        "ref_frame":1
    })
    env.step({
        "burn_vec":[1.0, 0, 0, 1.0],
        "ref_frame":2
    })

    with pytest.raises(ValueError):
        env.step({
            "burn_vec":[1.0, 0, 0, 1.0],
            "ref_frame":3
        })

 