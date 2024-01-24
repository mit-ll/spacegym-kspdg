# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

'''
Test that require:
1. KSP to be running
2. Play Mission mode -> sb1_i5
3. kRPC server to be running
'''
import pytest
import time
import numpy as np

import kspdg.utils.constants as C
from kspdg.sb1.e1_envs import SB1_E1_I5_Env

@pytest.fixture
def sb1_e1_i5_env():
    '''setup and teardown of the PursuitEnvV20220516 object connected to kRPC server'''
    env = SB1_E1_I5_Env()
    env.reset()
    yield env
    env.close()

def test_get_reward_and_info_0(sb1_e1_i5_env):
    '''check that init state get info is as expected'''
    # ~~ ARRANGE ~~

    if sb1_e1_i5_env is None:
        env = SB1_E1_I5_Env()
        env.reset()
    else:
        env = sb1_e1_i5_env

    t0 = env.vesPursue.met


    # ~~ ACT ~~ 
    obs = env.get_observation()
    info = env.get_info(obs, False)
    rew = env.get_reward()

    # ~~ ASSERT ~~
    exp_rew = 1.0
    assert np.isclose(rew, 1.0, atol=1e-3)
    assert np.isclose(info[env.PARAMS.INFO.K_MAX_REWARD], exp_rew, atol=1e-3)
    assert np.isclose(info[env.PARAMS.INFO.K_MIN_REWARD], exp_rew, atol=1e-3)

    # ~~ ACT & ASSERT ~~ 
    # time.sleep(tstep) # fixed sleep time to establish expected score
    time.sleep(2.0)
    obs = env.get_observation()
    info = env.get_info(obs, False)
    dt = env.vesPursue.met - t0
    assert np.isclose(info[env.PARAMS.INFO.K_WEIGHTED_SCORE], dt*exp_rew, rtol=1e-1)   # get info is called twice

    env.close()

def test_step_action_space_0(sb1_e1_i5_env):
    '''check that step accepts various action input formats without error'''
    # ~~ ARRANGE ~~

    if sb1_e1_i5_env is None:
        env = SB1_E1_I5_Env()
        env.reset()
    else:
        env = sb1_e1_i5_env

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
