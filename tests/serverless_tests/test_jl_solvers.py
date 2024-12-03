# krpc-serverless pytests for differential game solvers written in julia

import pytest
import numpy as np

from pathlib import Path
from importlib.resources import files

from kspdg.utils.private_src_utils import get_private_src_module_str


#--- Get path to solve_lbg1.jl from kspdg installation point
# (e.g. may be local to this file if pip installed editably or within a 
# conda environment somewhere else in the file system)

# PosixPath to kspdg installation point
KSPDG_INSTALL_PATH = files('kspdg')

# Path to solve_lbg1.jl relative to kspdg install point, accounting for
# python version and OS-specific directories
SOLVE_LBG1_JL_PATH = get_private_src_module_str("kspdg_envs.lbg1")
SOLVE_LBG1_JL_PATH = SOLVE_LBG1_JL_PATH.replace('.','/')
SOLVE_LBG1_JL_PATH = SOLVE_LBG1_JL_PATH.partition('/')[2]

# Join the solve_lbg1.jl relative path to kspdg absolute path
SOLVE_LBG1_JL_PATH = KSPDG_INSTALL_PATH / SOLVE_LBG1_JL_PATH / "solve_lbg1.jl"

def test_solve_lbg1_jl_import():
    """check import of solve_lbg1.jl does not error"""
    # ~~ ARRANGE ~~
    from juliacall import Main as jl

    # ~~ ACT ~~
    jl.include(str(SOLVE_LBG1_JL_PATH))

def test_accel_quad_penalty_1():
    """check acceleration penalty (i.e. soft control constraint) outputs as expected"""

    # ~~ ARRANGE ~~
    from juliacall import Main as jl
    a = np.ones(3)

    # ~~ ACT ~~
    jl.include(str(SOLVE_LBG1_JL_PATH))
    c_a = jl.accel_quad_penalty(a, 1, 1)

    # ~~ ASSERT ~~
    assert np.isclose(c_a, 1.5)

def get_lbg1_i2_init_conds():
    # define initial state
    R_k = 6.0e5/1e3      # [km] Kerbin radius
    mu_k = 3.5316e12/1e9   # [km^3/s^2] Kerbin gravitational parameter
    pos_l_cb__rhcbci = [7.08263725e+02, -2.46702910e+02, -5.72613803e-04]   # [km] lbg1_i2 lady init position 
    vel_l_cb__rhcbci = [7.13727102e-01,  2.04924253e+00,  3.58013683e-06] # [km/s] lbg1_i2 lady init velocity
    ladyX0 = np.concatenate((pos_l_cb__rhcbci, vel_l_cb__rhcbci))
    pos_b_cb__rhcbci = [7.07398403e+02, -2.49173762e+02, -4.35846058e-04]   # [km] lbg1_i2 bandit init position
    vel_b_cb__rhcbci = [7.20875939e-01,  2.04673868e+00, 3.60504223e-06] # [km/s] lbg1_i2 bandit init velocity
    banditX0 = np.concatenate((pos_b_cb__rhcbci, vel_b_cb__rhcbci))
    pos_g_cb__rhcbci = [7.08048206e+02, -2.47320907e+02, -4.32614294e-04]   # [km] lbg1_i2 guard init position 
    vel_g_cb__rhcbci = [7.15515133e-01,  2.04861891e+00, 3.57553846e-06] # [km/s] lbg1_i2 guard init velocity
    guardX0 = np.concatenate((pos_g_cb__rhcbci, vel_g_cb__rhcbci))

    # define accel constraints and cost weights
    bandit_acc_max = 0.0012   # max bandit acceleration [km/s/s]
    guard_acc_max = 0.0012   # max guard acceleration [km/s/s]

    return (
        R_k, mu_k, 
        ladyX0, banditX0, guardX0, 
        bandit_acc_max, 
        guard_acc_max
    )    

@pytest.fixture
def lbg1_i2_init_conds():
    return get_lbg1_i2_init_conds()

def test_solve_lady_bandit_guard_costtype_3_2(lbg1_i2_init_conds):
    """run solve_lady_bandit_guard_3 (type-3 costs) solver on KSP conditions"""

    # ~~ ARRANGE ~~
    from juliacall import Main as jl

    jl.include(str(SOLVE_LBG1_JL_PATH))

    t_step = 1.0    # [s] length of timestep
    n_steps = 100    # [-] number of timesteps

    # define solver regularization terms
    # NOTE: I do not yet have intuition on exactly how these terms affect/apply in the solver, but
    # they can make a big difference as to whether or not the solver converges. If left to their defaults
    # of 0.0 as encoded in iLQGames.jl, the solution to this problem will not converge
    sol_max_n_iter = 200
    sol_state_reg = 1e-3
    sol_control_reg = 1e-3

    # get init conds from fixture
    R_k, mu_k, ladyX0, banditX0, guardX0, bandit_acc_max, guard_acc_max = \
        lbg1_i2_init_conds if lbg1_i2_init_conds is not None else get_lbg1_i2_init_conds()
    
    bg_dist_weight = 1.0
    lb_dist_weight = 1.0
    bg_speed_weight = 1e4 # [s^2]
    lb_speed_weight = 1e4 # [s^2]
    bandit_acc_max_penalty_weight = 1e1 # [km^2]
    guard_acc_max_penalty_weight = 1e1 # [km^2]

    # ~~ ACT ~~
    conv, b_pos, b_vel, b_acc, g_pos, g_vel, g_acc, l_pos, l_vel = jl.solve_lady_bandit_guard_costtype_3(
        t_step = t_step,
        n_steps = n_steps,
        mu = mu_k,
        ladyX0 = ladyX0,
        banditX0 = banditX0,
        guardX0 = guardX0,
        bandit_acc_max = bandit_acc_max, 
        bandit_acc_max_penalty_weight = bandit_acc_max_penalty_weight, 
        guard_acc_max = guard_acc_max, 
        guard_acc_max_penalty_weight = guard_acc_max_penalty_weight, 
        bg_dist_weight = bg_dist_weight, 
        lb_dist_weight = lb_dist_weight,
        bg_speed_weight = bg_speed_weight,
        lb_speed_weight = lb_speed_weight,
        solver_max_n_iter = sol_max_n_iter,
        solver_state_regularization = sol_state_reg,
        solver_control_regularization = sol_control_reg)
    
    assert b_pos.shape == (3,n_steps)

    if __name__ == "__main__":
        
        # running test for visualization
        return conv, b_pos, b_vel, b_acc, g_pos, g_vel, g_acc, l_pos, l_vel
    
    else:

        # running test for unit testing
        assert conv
    

if __name__ == "__main__":

    conv, b_pos, b_vel, b_acc, g_pos, g_vel, g_acc, l_pos, l_vel = test_solve_lady_bandit_guard_costtype_3_2(None)

    print(f"Converged: {conv}")

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # plot position trajectory
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.plot(b_pos[0,:], b_pos[1,:], b_pos[2,:], label='Bandit', marker='o')
    ax1.plot(g_pos[0,:], g_pos[1,:], g_pos[2,:], label='Guard', marker='^')
    ax1.plot(l_pos[0,:], l_pos[1,:], l_pos[2,:], label='Lady', marker='o')

    # Adding labels and legend
    ax1.set_xlabel('X-coord [km]')
    ax1.set_ylabel('Y-coord [km]')
    ax1.set_zlabel('Z-coord [km]')
    ax1.set_box_aspect([1,1,1])
    ax1.set_title('3D Trajectories of Lady-Bandit-Guard')
    ax1.legend()

    # plot controls (acceleration)
    fig2 = plt.figure()
    ax2_1 = fig2.add_subplot(211)
    ax2_1.plot(b_acc[0,:], label='x-acc')
    ax2_1.plot(b_acc[1,:], label='y-acc')
    ax2_1.plot(b_acc[2,:], label='z-acc')
    ax2_1.set_xlabel('timestep [-]')
    ax2_1.set_ylabel('acceleration [km/s/s]')
    ax2_1.set_title('Bandit Acceleration')
    ax2_1.legend()

    ax2_2 = fig2.add_subplot(212)
    ax2_2.plot(g_acc[0,:], label='x-acc')
    ax2_2.plot(g_acc[1,:], label='y-acc')
    ax2_2.plot(g_acc[2,:], label='z-acc')
    ax2_2.set_xlabel('timestep [-]')
    ax2_2.set_ylabel('acceleration [km/s/s]')
    ax2_2.set_title('Guard Acceleration')
    ax2_2.legend()

    # plot position trajectory in 2D
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.plot(b_pos[0,:], b_pos[1,:], label='Bandit', marker='o')
    ax3.plot(g_pos[0,:], g_pos[1,:], label='Guard', marker='^')
    ax3.plot(l_pos[0,:], l_pos[1,:], label='Lady', marker='o')
    ax3.set_xlabel('X-coord [km]')
    ax3.set_ylabel('Y-coord [km]')
    ax3.set_title('2D Trajectories of Lady-Bandit-Guard')
    ax3.legend()

    # plot relative distances
    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    ax4.plot([np.linalg.norm(b_pos[:,i]-g_pos[:,i]) for i in range(b_pos.shape[1])], label='Bandit-Guard')
    ax4.plot([np.linalg.norm(b_pos[:,i]-l_pos[:,i]) for i in range(b_pos.shape[1])], label='Bandit-Lady')
    ax4.set_xlabel('timestep [-]')
    ax4.set_ylabel('relative distance [km]')
    ax4.set_title('Relative Distances')
    ax4.legend()

    # Display the plot
    plt.show()
