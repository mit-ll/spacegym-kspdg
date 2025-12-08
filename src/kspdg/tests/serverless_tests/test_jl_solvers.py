# krpc-serverless pytests for differential game solvers written in julia

import pytest
import numpy as np

from importlib.resources import files

from kspdg.utils.private_src_utils import get_private_src_module_str

# suppress warnings from mismatch between numpy and juliacall array copy
pytestmark = pytest.mark.filterwarnings(
    "ignore:.*__array__ implementation doesn't accept a copy keyword.*:DeprecationWarning"
)

#--- Get path to solve_lbg1.jl from kspdg installation point
# (e.g. may be local to this file if pip installed editably or within a 
# conda environment somewhere else in the file system)

# PosixPath to kspdg installation point
KSPDG_INSTALL_PATH = files('kspdg')

# Path to solve_lbg1.jl relative to kspdg install point, accounting for
# python version and OS-specific directories
SOLVERS_LBG1_JL_PATH = get_private_src_module_str("kspdg_envs.lbg1")
SOLVERS_LBG1_JL_PATH = SOLVERS_LBG1_JL_PATH.replace('.','/')
SOLVERS_LBG1_JL_PATH = SOLVERS_LBG1_JL_PATH.partition('/')[2]

# Join the solve_lbg1.jl and solve_lq_lbg1.jl relative path to kspdg absolute path
SOLVE_LBG1_JL_PATH = KSPDG_INSTALL_PATH / SOLVERS_LBG1_JL_PATH / "solve_lbg1.jl"
SOLVE_LQ_LBG1_JL_PATH = KSPDG_INSTALL_PATH / SOLVERS_LBG1_JL_PATH / "solve_lq_lbg1.jl"

from juliacall import Main as jl

def test_solve_lbg1_jl_import():
    """check import of solve_lbg1.jl does not error"""
    jl.include(str(SOLVE_LBG1_JL_PATH))
    jl.include(str(SOLVE_LQ_LBG1_JL_PATH))

def _ensure_solvers_loaded():
    """Include the Julia file once into Main."""
    if (
        hasattr(jl, "solve_lady_bandit_guard_costtype_3") and 
        hasattr(jl, "solve_lq_lady_bandit_guard_costtype_1") and
        hasattr(jl, "lbg_quad_costs_1")
        ):
        return

    jl.include(str(SOLVE_LBG1_JL_PATH))
    jl.include(str(SOLVE_LQ_LBG1_JL_PATH))

def test_accel_quad_penalty_1():
    """check acceleration penalty (i.e. soft control constraint) outputs as expected"""

    # ~~ ARRANGE ~~
    _ensure_solvers_loaded()
    a = np.ones(3)

    # ~~ ACT ~~
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
    _ensure_solvers_loaded()

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

def test_solve_lady_bandit_guard_costtype_3_3(lbg1_i2_init_conds):
    """run solve_lady_bandit_guard_3 (type-3 costs) solver on KSP conditions"""

    # ~~ ARRANGE ~~
    _ensure_solvers_loaded()

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
    
    bg_dist_weight = 1e4
    lb_dist_weight = 0.0
    bg_speed_weight = 0.0 # [s^2]
    lb_speed_weight = 0.0 # [s^2]
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

def lbg_quad_costs_1_py(**kwargs):
    """
    Python wrapper around Julia lbg_quad_costs_1(...).

    Returns NumPy arrays:
    Qb, qb, Rb, rb, Qg, qg, Rg, rg
    """
    _ensure_solvers_loaded()
    Qb_jl, qb_jl, Rb_jl, rb_jl, Qg_jl, qg_jl, Rg_jl, rg_jl = jl.lbg_quad_costs_1(
        **kwargs
    )
    Qb = np.array(Qb_jl)
    qb = np.array(qb_jl)
    Rb = np.array(Rb_jl)
    rb = np.array(rb_jl)
    Qg = np.array(Qg_jl)
    qg = np.array(qg_jl)
    Rg = np.array(Rg_jl)
    rg = np.array(rg_jl)
    return Qb, qb, Rb, rb, Qg, qg, Rg, rg

def test_lq_lbg_quad_costs_1_shapes_and_zero_linear_terms():
    Qb, qb, Rb, rb, Qg, qg, Rg, rg = lbg_quad_costs_1_py()

    # Shapes
    assert Qb.shape == (12, 12)
    assert Qg.shape == (12, 12)
    assert Rb.shape == (6, 6)
    assert Rg.shape == (6, 6)
    assert qb.shape == (12,)
    assert qg.shape == (12,)
    assert rb.shape == (6,)
    assert rg.shape == (6,)

    # Linear terms are zero (for now by design)
    assert np.allclose(qb, 0.0)
    assert np.allclose(qg, 0.0)
    assert np.allclose(rb, 0.0)
    assert np.allclose(rg, 0.0)

    # Q and R should be symmetric
    assert np.allclose(Qb, Qb.T)
    assert np.allclose(Qg, Qg.T)
    assert np.allclose(Rb, Rb.T)
    assert np.allclose(Rg, Rg.T)

def _stage_cost(Q, q, R, r, x, u):
    return 0.5 * x.T @ Q @ x + q.T @ x + 0.5 * u.T @ R @ u + r.T @ u

def test_lq_lbg_quad_costs_1_bandit_guard_relative_position_effect():
    # Choose weights:
    #   bandit: likes being near lady (w_blady>0), dislikes guard nearby (w_bguard<0)
    #   guard: likes being near bandit (w_gbandit>0), dislikes bandit near lady (w_glady<0)
    Qb, qb, Rb, rb, Qg, qg, Rg, rg = lbg_quad_costs_1_py(
        w_b_bl_dist=1.0,
        w_b_bg_dist=-1.0,
        w_g_bl_dist=-1.0,
        w_g_bg_dist=1.0,
        w_b_ctrl=0.0,
        w_g_ctrl=0.0,
    )

    # State with bandit at origin, guard 10 m away along +x:
    x1 = np.zeros(12)
    x1[0] = 0.0   # x_b
    x1[6] = 10.0  # x_g

    # State with bandit at origin, guard 20 m away along +x:
    x2 = np.zeros(12)
    x2[0] = 0.0
    x2[6] = 20.0

    u_zero = np.zeros(6)

    Jb1 = _stage_cost(Qb, qb, Rb, rb, x1, u_zero)
    Jb2 = _stage_cost(Qb, qb, Rb, rb, x2, u_zero)

    Jg1 = _stage_cost(Qg, qg, Rg, rg, x1, u_zero)
    Jg2 = _stage_cost(Qg, qg, Rg, rg, x2, u_zero)

    # Bandit dislikes being near guard (w_bguard<0), so distance 20 should have
    # LOWER cost than distance 10 when minimizing Jb.
    assert Jb2 < Jb1

    # Guard likes being near bandit (w_gbandit>0), so distance 20 should have
    # HIGHER cost than distance 10 when minimizing Jg.
    assert Jg2 > Jg1

def test_lq_lbg_quad_costs_1_known_costs():
    # Choose weights:
    #   bandit: likes being near lady (w_blady>0), dislikes guard nearby (w_bguard<0)
    #   guard: likes being near bandit (w_gbandit>0), dislikes bandit near lady (w_glady<0)
    Qb, qb, Rb, rb, Qg, qg, Rg, rg = lbg_quad_costs_1_py(
        w_b_bl_dist=1.0,
        w_b_bg_dist=-1.0,
        w_g_bl_dist=-1.0,
        w_g_bg_dist=1.0,
        w_b_ctrl=1.0,
        w_g_ctrl=1.0,
    )

    # State with bandit and guard at origin, no control:
    x = np.zeros(12)
    x[0] = 0.0   # x_b
    x[6] = 0.0  # x_g
    u = np.zeros(6)
    Jb = _stage_cost(Qb, qb, Rb, rb, x, u)
    Jg = _stage_cost(Qg, qg, Rg, rg, x, u)
    assert np.isclose(Jb, 0.0)
    assert np.isclose(Jg, 0.0)

    # State with bandit at origin, guard 10 m away along +x, no control:
    x = np.zeros(12)
    x[0] = 0.0   # x_b
    x[6] = 10.0  # x_g
    u = np.zeros(6)
    Jb = _stage_cost(Qb, qb, Rb, rb, x, u)
    Jg = _stage_cost(Qg, qg, Rg, rg, x, u)
    assert np.isclose(Jb, -0.5*(10.0**2))
    assert np.isclose(Jg, 0.5*(10.0**2))

    # State with bandit along +y axis, guard on -y axis, no control:
    x = np.zeros(12)
    x[1] = 10.0   # x_b
    x[7] = -10.0   # x_g
    u = np.zeros(6)
    Jb = _stage_cost(Qb, qb, Rb, rb, x, u)
    Jg = _stage_cost(Qg, qg, Rg, rg, x, u)
    assert np.isclose(Jb, 0.5*(10**2) - 0.5*(20.0**2))
    assert np.isclose(Jg, -0.5*(10**2) + 0.5*(20.0**2))

    # bandit and guard at arbitrary positions with arbitrary control
    # tests that velocity doesn't affect cost
    x = np.random.rand(12)
    p_b = x[0:3] = np.array([-0.98912135, -0.36778665,  1.28792526])
    p_g = x[6:9] = np.array([ 0.19397442,  0.92023090,  0.57710379])
    dist_bl = np.linalg.norm(p_b)
    dist_bg = np.linalg.norm(p_b - p_g)

    u = np.random.rand(6)
    u_b = u[0:3] = np.array([-0.63646365,  0.54195222, -0.31659545])
    u_g = u[3:6] = np.array([-0.32238912,  0.09716732, -1.52593041])
    ctrl_b = np.linalg.norm(u_b)
    ctrl_g = np.linalg.norm(u_g)

    Jb_exp = 0.5*(dist_bl**2 - dist_bg**2 + ctrl_b**2)
    Jg_exp = 0.5*(dist_bg**2 - dist_bl**2 + ctrl_g**2)

    Jb = _stage_cost(Qb, qb, Rb, rb, x, u)
    Jg = _stage_cost(Qg, qg, Rg, rg, x, u)

    assert np.isclose(Jb, Jb_exp)
    assert np.isclose(Jg, Jg_exp)

def test_lq_solve_lady_bandit_guard_costtype_1_shapes():
    """check that lq lbg solver returns strategy of appropriate shape"""

    # ~~ ARRANGE ~~
    _ensure_solvers_loaded()

    t_step = 1.0    # [s] length of timestep
    n_steps = 10    # [-] number of timesteps
    orbital_rate = 0.0032   # [rad/s] mean motion of reference (lady) orbit
    # banditX0 = np.array([0.0, -2000.0, 0.0, 0.0, 0.0, 0.0])     # init state of bandit relative to lady in [m] and [m/s]
    # guardX0 = np.array([0.0, -1000.0, 0.0, 0.0, 0.0, 0.0])      # init state of bandit relative to guard in [m] and [m/s]

    # ~~ ACT ~~
    strat = jl.solve_lq_lady_bandit_guard_costtype_1(
        t_step=t_step,
        n_steps=n_steps,
        orbital_rate=orbital_rate,
        w_b_ctrl=1.0,
        w_g_ctrl=1.0,
        # banditX0=banditX0,
        # guardX0=guardX0
    )

    # ~~ ASSERT ~~
    for tidx in range(n_steps):
        assert strat[tidx].P.shape == (6,12)
        assert strat[tidx].α.shape == (6,)

if __name__ == "__main__":

    conv, b_pos, b_vel, b_acc, g_pos, g_vel, g_acc, l_pos, l_vel = test_solve_lady_bandit_guard_costtype_3_3(None)

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
