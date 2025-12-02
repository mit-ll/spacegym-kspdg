# Copyright (c) 2025, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

"""
solve_lq_lbg1.jl 

find equilibrium strategies for bandit and guard in linear-quadratic 
    lady-bandit-guard orbital problem

"""


using FromFile: @from

@from "dynamics.jl" import cw_discrete__rhntw

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

    # --- define indices of each player's controls ---

    # --- define per-player quadratic costs ---

    # --- package as linear-quadratic game object ---

    # --- solve for feedback nash equilibrium affine strategy ---
    

end