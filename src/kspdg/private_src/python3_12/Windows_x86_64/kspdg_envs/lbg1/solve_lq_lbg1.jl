# Copyright (c) 2025, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

"""
solve_lq_lbg1.jl 

find equilibrium strategies for bandit and guard in linear-quadratic 
    lady-bandit-guard orbital problem

"""

using iLQGames: 
    LTISystem, 
    LinearSystem, 
    LQGame, 
    QuadraticPlayerCost,
    SVector,
    SMatrix,
    strategytype,
    solve_lq_game!

using FromFile: @from
@from "dynamics.jl" import lbg_cw_discrete_eom__rhntw

"""
    lbg_quad_costs_1(; 
        w_b_bl_dist=1.0, w_b_bg_dist=-1.0,
        w_g_bl_dist=-1.0, w_g_bg_dist=1.0,
        w_b_ctrl=1.0, w_g_ctrl=1.0)

Construct time-invariant quadratic cost matrices for the bandit and
guard players in the Lady–Bandit–Guard (LBG) game under CW dynamics in
a right-handed NTW frame.

Each player i has a stage objective of the form

    J_i(x,u) = 1/2 * x' * Q_i * x + q_i' * x
               + 1/2 * u' * R_i * u + r_i' * u

with x ∈ ℝ¹² defined as the position and velocity
of the bandit and guard wrt to the reference orbit (lady)
expressed in right-handed NTW frame

    x   = [ pos_b_l__rhntw, vel_b_l__rhntw, 
            pos_g_l__rhntw, vel_g_l__rhntw]

        (which expanded with simplified notation, looks like)

        = [ px_b, py_b, pz_b, vx_b, vy_b, vz_b,
            px_g, py_g, pz_g, vx_g, vy_g, vz_g]

And u ∈ ℝ⁶ defined as the accelerations of the 
bandit and guard wrt to the reference orbit (lady)
expressed in right-handed NTW frame. Note that this is a 
non-inertial acceleration that I'm using under the assumption
of short time horizons and close proximity to the origin
(not sure if there are other unexpected consquences of 
naively using non-inertial frame)

    u   = [ acc_b_l__rhntw, acc_g_l__rhntw]
    
        (which expanded with simplified notation, looks like)

        = [ ax_b, ay_b, az_b, ax_g, ay_g, az_g]

The structure encoded here is:

- Bandit:
    - “Proximity to lady” (origin) via px_b, py_b, pz_b.
    - “Proximity / separation to guard” via (px_b - px_g, py_b - py_g, pz_b - pz_g).
    - Quadratic penalty on its own control effort (ax_b, ay_b, az_b).

- Guard:
    - “Proximity to bandit” via (px_b - px_g, py_b - py_g, pz_b - pz_g).
    - Dependence on bandit’s proximity to the lady via px_b, py_b, pz_b.
    - Quadratic penalty on its own control effort (ax_g, ay_g, az_g).

Note: The signs of the weights determine whether each term acts as a
“reward” or a “penalty” depending on whether you maximize or minimize
J_i. For example, with J_i minimized:
- a positive w_b_bl_dist encourages the bandit to stay near the lady
  (origin),
- a positive w_g_bg_dist encourages the guard to stay close to the bandit,
- a negative w_b_bg_dist encourages the bandit to increase separation from
  the guard, etc.

# Arguments
- `w_b_bl_dist::Real=1.0`:
    Weight on bandit’s distance to the lady (origin) in the bandit’s Q.

- `w_b_bg_dist::Real=-1.0`:
    Weight on bandit–guard relative distance in the bandit’s Q.
    The sign of this weight controls whether proximity to the guard is
    favored (positive) or discouraged (negative).

- `w_g_bl_dist::Real=-1.0`:
    Weight on bandit’s distance to the lady in the guard’s Q.
    The sign controls whether the guard “likes” or “dislikes” the bandit
    being near the lady.

- `w_g_bg_dist::Real=1.0`:
    Weight on bandit–guard relative distance in the guard’s Q.

- `w_b_ctrl::Real=1.0`:
    Quadratic weight on bandit control effort (a_x_b, a_y_b, a_z_b).

- `w_g_ctrl::Real=1.0`:
    Quadratic weight on guard control effort (a_x_g, a_y_g, a_z_g).

# Returns
- `Q_b::Matrix{Float64}`:
    12×12 state-weight matrix for the bandit.

- `q_b::Vector{Float64}`:
    12-element linear state-weight vector for the bandit (zero here).

- `R_b::Matrix{Float64}`:
    6×6 control-weight matrix for the bandit; only the first 3 diagonals
    (bandit controls) are nonzero.

- `r_b::Vector{Float64}`:
    6-element linear control-weight vector for the bandit (zero here).

- `Q_g::Matrix{Float64}`:
    12×12 state-weight matrix for the guard.

- `q_g::Vector{Float64}`:
    12-element linear state-weight vector for the guard (zero here).

- `R_g::Matrix{Float64}`:
    6×6 control-weight matrix for the guard; only the last 3 diagonals
    (guard controls) are nonzero.

- `r_g::Vector{Float64}`:
    6-element linear control-weight vector for the guard (zero here).

"""
function lbg_quad_costs_1(; 
    w_b_bl_dist::Real = 1.0,
    w_b_bg_dist::Real = -1.0,
    w_g_bl_dist::Real = -1.0,
    w_g_bg_dist::Real = 1.0,
    w_b_ctrl::Real = 1.0,
    w_g_ctrl::Real = 1.0)

    # --- selection matrices for positions ---
    S_pb = zeros(3, 12)
    S_pb[1,1] = 1.0
    S_pb[2,2] = 1.0
    S_pb[3,3] = 1.0

    S_pg = zeros(3, 12)
    S_pg[1,7] = 1.0
    S_pg[2,8] = 1.0
    S_pg[3,9] = 1.0

    S_rel = S_pb - S_pg

    Q_pos_bandit = S_pb' * S_pb      # ||p_b||^2
    Q_rel        = S_rel' * S_rel    # ||p_b - p_g||^2

    # --- Bandit state weighting ---
    Q_bandit = w_b_bl_dist  * Q_pos_bandit +
               w_b_bg_dist * Q_rel

    # --- Guard state weighting ---
    Q_guard  = w_g_bg_dist * Q_rel +
               w_g_bl_dist   * Q_pos_bandit

    # --- Control weighting ---
    R_bandit = zeros(6, 6)
    R_guard  = zeros(6, 6)

    # Bandit control: first 3 components
    R_bandit[1,1] = w_b_ctrl
    R_bandit[2,2] = w_b_ctrl
    R_bandit[3,3] = w_b_ctrl

    # Guard control: last 3 components
    R_guard[4,4] = w_g_ctrl
    R_guard[5,5] = w_g_ctrl
    R_guard[6,6] = w_g_ctrl

    # --- Zero linear terms ---
    q_bandit = zeros(12)
    q_guard  = zeros(12)
    r_bandit = zeros(6)
    r_guard  = zeros(6)

    # --- convert to StaticArrays at the end ---
    Qb = SMatrix{12,12}(Q_bandit)
    qb = SVector{12}(q_bandit)
    Rb = SMatrix{6,6}(R_bandit)
    rb = SVector{6}(r_bandit)
    Qg = SMatrix{12,12}(Q_guard)
    qg = SVector{12}(q_guard)
    Rg = SMatrix{6,6}(R_guard)
    rg = SVector{6}(r_guard)

    return Qb, qb, Rb, rb,
           Qg, qg, Rg, rg
end


"""
    solve_lady_bandit_guard_costtype_1(TODO)

wrapper function for solve_lq_lady_bandit_guard that defines the costtype-1

# Arguments
w_b_ctrl::Real = 1.0,
w_g_ctrl::Real = 1.0
- `kwargs::Dict`: see solve_lady_bandit_guard for full argument list

# Returns
- see solve_lq_lady_bandit_guard return list
"""
function solve_lq_lady_bandit_guard_costtype_1(;
    w_b_ctrl,
    w_g_ctrl,
    kwargs...)

    # --- define per-player quadratic costs ---
    Qb, qb, Rb, rb, Qg, qg, Rg, rg = lbg_quad_costs_1(
        w_b_ctrl = w_b_ctrl,
        w_g_ctrl = w_g_ctrl
    )
    pcb = QuadraticPlayerCost(qb, Qb, rb, Rb)
    pcg = QuadraticPlayerCost(qg, Qg, rg, Rg)

    # time-invariant costs repeated for nt stages
    costs = [ [pcb, pcg] for _ in 1:kwargs[:n_steps] ]

    # --- call generalized solver with keyword type forwarding
    return solve_lq_lady_bandit_guard(costs; kwargs...)

end

"""
    solve_lq_lady_bandit_guard(TODO)

Use iLQGames to compute feedback nash equilibrium strategy for bandit and guard
in a discrete-time, linear-quadratic lady-bandit-guard (LBG) game based upon 
Clohessy–Wiltshire dynamics defined with the lady satellite as the 
target/reference orbit (assumes lady is in a circular orbit)

# Arguments
- `costs::AbstractArray{}`: tuple of cost functions for each player
- `t_step::AbstractFloat`: time step length [s]
- `n_steps::AbstractArray`: time horizon over which game is solved [s]
- `orbital_rate::Float`: mean motion of reference orbit (i.e. lady satellite) [rad/sec]

# Returns
- `strat::AffineStrategy`:
    feedback Nash equilibrium with terms P (proportional feedback) 
    and α (bias, feedforward) at each time stage t in n_steps.

    P[t] is a matrix of size (nu,nx) and α[t] is a vector of size (nu,)
    at discrete time t such that
    u[t] = -P_t @ x[t] - a_t

    with x ∈ ℝ¹² defined as the position and velocity
    of the bandit and guard wrt to the reference orbit (lady)
    expressed in right-handed NTW frame

    x   = [ pos_b_l__rhntw, vel_b_l__rhntw, 
            pos_g_l__rhntw, vel_g_l__rhntw]

        (which expanded with simplified notation, looks like)

        = [ px_b, py_b, pz_b, vx_b, vy_b, vz_b,
            px_g, py_g, pz_g, vx_g, vy_g, vz_g]

    And u ∈ ℝ⁶ defined as the accelerations of the 
    bandit and guard wrt to the reference orbit (lady)
    expressed in right-handed NTW frame. Note that this is a 
    non-inertial acceleration that I'm using under the assumption
    of short time horizons and close proximity to the origin
    (not sure if there are other unexpected consquences of 
    naively using non-inertial frame)

    u   = [ acc_b_l__rhntw, acc_g_l__rhntw]
    
        (which expanded with simplified notation, looks like)

        = [ ax_b, ay_b, az_b, ax_g, ay_g, az_g]

"""
function solve_lq_lady_bandit_guard(costs;
    t_step::AbstractFloat,
    n_steps::Int,
    orbital_rate::AbstractFloat)

    # --- setup the game dynamics as linear time-invariant system ---
    A, B = lbg_cw_discrete_eom__rhntw(t_step, orbital_rate)
    ls = LinearSystem{t_step}(A, B)
    ltisys = LTISystem(ls, nothing) # seting xyids to nothing because it doesn't seem necessary nor applicable

    # --- define indices of each player's controls ---
    # Player 1: Bandit 
    # Player 2: Guard
    uids = (SVector(1,2,3), SVector(4,5,6))

    # --- package as linear-quadratic game object ---
    lqgame = LQGame(uids, ltisys, costs)

    # --- solve for feedback nash equilibrium affine strategy ---
    strat = zero(strategytype(lqgame))
    solve_lq_game!(strat, lqgame)

    # P = nothing
    # alpha = nothing
    return strat

end