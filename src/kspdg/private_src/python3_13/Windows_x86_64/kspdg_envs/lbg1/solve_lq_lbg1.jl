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
    lbg_quad_costs_1(; w_blady=1.0, w_bguard=1.0,
                                w_gbandit=1.0, w_glady=1.0,
                                w_u_b=1.0, w_u_g=1.0)

Construct time-invariant quadratic cost matrices for the bandit and
guard players in the Lady–Bandit–Guard (LBG) game under CW dynamics in
a right-handed NTW frame.

Each player i has a stage objective of the form

    J_i(x,u) = 1/2 * x' * Q_i * x + q_i' * x
               + 1/2 * u' * R_i * u + r_i' * u

with x ∈ ℝ¹², u ∈ ℝ⁶ defined as

    x = [x_b, y_b, z_b, xdot_b, ydot_b, zdot_b,
         x_g, y_g, z_g, xdot_g, ydot_g, zdot_g]

    u = [a_x_b, a_y_b, a_z_b, a_x_g, a_y_g, a_z_g]

The structure encoded here is:

- Bandit:
    - “Proximity to lady” (origin) via x_b, y_b, z_b.
    - “Proximity / separation to guard” via (x_b - x_g, y_b - y_g, z_b - z_g).
    - Quadratic penalty on its own control effort (a_x_b, a_y_b, a_z_b).

- Guard:
    - “Proximity to bandit” via (x_b - x_g, y_b - y_g, z_b - z_g).
    - Dependence on bandit’s proximity to the lady via x_b, y_b, z_b.
    - Quadratic penalty on its own control effort (a_x_g, a_y_g, a_z_g).

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
function lbg_quad_costs_1(; w_b_bl_dist::Real = 1.0,
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
- `kwargs::Dict`: see solve_lady_bandit_guard for full argument list

# Returns
- see solve_lq_lady_bandit_guard return list
"""
function solve_lq_lady_bandit_guard_costtype_1(;
    kwargs...)

    # --- define per-player quadratic costs ---
    Qb, qb, Rb, rb, Qg, qg, Rg, rg = lbg_quad_costs_1()
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
- `banditX0::AbstractArray{Float64}`: initial state of bandit satellite 
    position [m] and velocity [m/s] in NTW coords relative 
    to lady's reference orbit (px, py, pz, vx, vy, vz)
- `guardX0::AbstractArray{Float64}`: initial state of guard satellite 
    position [m] and velocity [m/s] in NTW coords relative 
    to lady's reference orbit (px, py, pz, vx, vy, vz)

# Returns
- `P::Matrix`: of shape (nt,nu,nx) 
    feedback Nash equilibrium linear term
    P[t] is a size (nu,nx) matrix describing the linear term of the joint affine policy
    at discrete time t such that
    u[t] = -P_t @ x[t] - a_t
- `alpha::Matrix`: of shape (nt,nu) 
    feedback Nash equilibrium bias term
    alpha[t] is a size (nu,) vector describing the bias term of the joint affine policy 
    at discrete time t such that
    u[t] = -P_t @ x[t] - a_t

"""
function solve_lq_lady_bandit_guard(costs;
    t_step::AbstractFloat,
    n_steps::Int,
    orbital_rate::AbstractFloat,
    banditX0::AbstractArray{Float64},
    guardX0::AbstractArray{Float64})

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