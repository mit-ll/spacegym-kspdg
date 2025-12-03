# Copyright (c) 2025, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

"""
solve_lbg1.jl 

find equilibrium strategies for bandit and guard in lady-bandit-guard orbital problem

"""

using iLQGames: 
    SVector, 
    ControlSystem, 
    ProductSystem,
    FunctionPlayerCost,
    xindex,
    uindex,
    GeneralGame,
    iLQSolver,
    solve,
    norm, 
    SMatrix

using FromFile: @from

@from "dynamics.jl" import lbg_eom1__rhcbci

# functions for which new methods will be defined
import iLQGames: xyindex, dx

# parametes: 
#   NX_SAT: number of states for each satellite. position [km] and velocity [km/s] px, py, pz, vx, vy, vz in 
#   NU_SAT_ACTIVE: number of control inputs for each active satellite. propulsive acceleration [km/s/s] ax_p, ay_p, az_p 
#   NU_SAT_PASSIVE: no control inputs for passive satellites
#   NX_GAME: number of states for game of 3 satellites
#   NU_GAME: number of control inputs of 3 satellites
const NX_SAT = 6
const NU_SAT_ACTIVE = 3
const NU_SAT_PASSIVE = 0
const NX_GAME = 3*NX_SAT
const NU_GAME = 2*NU_SAT_ACTIVE + NU_SAT_PASSIVE

# creat control system of lady-bandit-guard 3-satellite system with non-maneuvering lady
struct LBGSystem{dT} <: ControlSystem{dT, NX_GAME, NU_GAME}
    GM::Float64;    # standard gravitational constant of central body [km^3/s^2]
end

# define dynamics of single satellite based on inertial equations of motion model
function dx(cs::LBGSystem, x::SVector{NX_GAME}, u::SVector{NU_GAME}, t::AbstractFloat)
    # Cast to static vector otherwise LinearSystem does not get static matrix and error is thrown
    return SVector{NX_GAME}(lbg_eom1__rhcbci(t, x, u, cs.GM))
end

"""
    lady_bandit_dist_sqr(x)

Compute distance squared between lady and bandit in lbg1 game state

# Arguments
- `x::AbstractArray{Float}`: full game state vector  
    - `x[1:6]`: lady satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[7:12]`: bandit satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[13:18]`: guard satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)

# Returns
- `d_lb_2::Float` distance squared between lady and bandit satellites [km^2]
"""
function lady_bandit_dist_sqr(x)
    return sum((x[1:3] - x[7:9]) .^2 )
end

"""
    lady_bandit_speed_sqr(x)

Compute relative speed squared between lady and bandit in lbg1 game state

# Arguments
- `x::AbstractArray{Float}`: full game state vector  
    - `x[1:6]`: lady satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[7:12]`: bandit satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[13:18]`: guard satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)

# Returns
- `s_lb_2::Float` relative speed squared between lady and bandit satellites [km^2/s^2]
"""
function lady_bandit_speed_sqr(x)
    return sum((x[4:6] - x[10:12]) .^2 )
end

"""
    bandit_guard_dist_sqr(x)

Compute distance squared between bandit and guard in lbg1 game state

# Arguments
- `x::AbstractArray{Float}`: full game state vector  
    - `x[1:6]`: lady satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[7:12]`: bandit satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[13:18]`: guard satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)

# Returns
- `d_bg_2::Float` distance squared between bandit and guard satellites [km^2]
"""
function bandit_guard_dist_sqr(x)
    return sum((x[7:9]-x[13:15]) .^2 )
end

"""
    bandit_guard_speed_sqr(x)

Compute distance squared between bandit and guard in lbg1 game state

# Arguments
- `x::AbstractArray{Float}`: full game state vector  
    - `x[1:6]`: lady satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[7:12]`: bandit satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[13:18]`: guard satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)

# Returns
- `s_bg_2::Float` relative speed squared between bandit and guard satellites [km^2/s^2]
"""
function bandit_guard_speed_sqr(x)
    return sum((x[10:12]-x[16:18]) .^2 )
end


"""
    accel_quad_penalty(acc_max, acc_max_penalty_weight, acc_max_penalty_sharpness)

Differentiable cost function to penalize acceleration based on quadratic function

# Arguments
- `acc::AbstractArray{Float}`: acceleration vector [km/s/s]
- `acc_max::Float`: thrust-limited maximum propulsive acceleration of bandit [km/s/s]
- `acc_max_penalty_weight::Float`: weight of thrust constraint violation [km^2]

# Returns
- `acc_cost::Float`: penalty cost for exceeding acceleration limit [km^2]
"""
function accel_quad_penalty(
    acc,
    acc_max, 
    acc_max_penalty_weight)

    # See ref: https://github.com/lassepe/iLQGames.jl/blob/ad850d174d0e46925be191ba75837b34b98936e3/src/cost_design_utils.jl#L4
    # note that I think this may be less code efficient to implement Q-matrix like this; does it get called too often?
    # See how Jake used it instead: https://llcad-github.llan.ll.mit.edu/MAST/SpaceGames.jl/blob/3155a8fc5a09930bbea613e0092d857ee4e5af4c/src/CW_games.jl#L16
    Q = SMatrix{3,3}([1. 0. 0.; 0. 1. 0.; 0. 0. 1.] * (1/acc_max^2))

    acc_cost = acc_max_penalty_weight * 1/2*acc'*Q*acc

    return acc_cost

end


"""
    bandit_cost_3(g, x, u, t)

Compute the total cost for bandit including speed differences and quadratic control penalty

# Arguments
- `g`: iLQGames.jl GeneralGame object
- `x::AbstractArray{Float}`: full game state vector  
    - `x[1:6]`: lady satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[7:12]`: bandit satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[13:18]`: guard satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
- `u::AbstractArray{Float}`: full game control vector   
    - `u[1:3]`: bandit propulsive acceleration [km/s/s] in cbci coords (ax_p, ay_p, az_p)
    - `u[4:6]`: guard propulsive acceleration [km/s/s] in cbci coords (ax_p, ay_p, az_p)
- `t::AbstractFloat`: time step [s]
- `acc_max::Float`: thrust-limited maximum propulsive acceleration of bandit [km/s/s]
- `acc_max_penalty_weight::Float`: weight of thrust constraint violation [km^2]
- `bg_dist_weight:: Float`: scaling factor bandit-guard distance cost [-]
- `lb_dist_weight:: Float`: scaling factor lady-bandit distance cost [-]
- `lb_speed_weight:: Float`: scaling factor lady-bandit relative speed cost [s^2]

# Returns
- `cost::Float`: cost value for bandit [km^2]

# Notes
- this is meant to be an alternative formulation to the game rather than bandit_guard_total_cost_1 and bandit_cost_2
"""
function bandit_cost_3(
    g, x, u, t,
    acc_max, 
    acc_max_penalty_weight, 
    bg_dist_weight, 
    lb_dist_weight,
    lb_speed_weight)

    # minimize distance and relative speed between lady and bandit
    cost = lb_dist_weight * lady_bandit_dist_sqr(x)
    cost += lb_speed_weight * lady_bandit_speed_sqr(x)

    # maximize distance between bandit and guard
    cost += -bg_dist_weight * bandit_guard_dist_sqr(x)

    # penalize control input based on quadratic of acceleration
    cost += accel_quad_penalty(u[1:3], acc_max, acc_max_penalty_weight)

    return cost
end

"""
    guard_cost_3(g, x, u, t)

Compute the total cost for guard including speed differences and quadratic control penalty

# Arguments
- `g`: iLQGames.jl GeneralGame object
- `x::AbstractArray{Float}`: full game state vector  
    - `x[1:6]`: lady satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[7:12]`: bandit satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x[13:18]`: guard satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
- `u::AbstractArray{Float}`: full game control vector   
    - `u[1:3]`: bandit propulsive acceleration [km/s/s] in cbci coords (ax_p, ay_p, az_p)
    - `u[4:6]`: guard propulsive acceleration [km/s/s] in cbci coords (ax_p, ay_p, az_p)
- `t::AbstractFloat`: time step [s]
- `acc_max::Float`: thrust-limited maximum propulsive acceleration of bandit [km/s/s]
- `acc_max_penalty_weight::Float`: weight of thrust constraint violation [km^2]
- `bg_dist_weight:: Float`: scaling factor bandit-guard distance cost [-]
- `lb_dist_weight:: Float`: scaling factor lady-bandit distance cost [-]
- `bg_speed_weight:: Float`: scaling factor bandit-guard relative speed cost [s^2]

# Returns
- `cost::Float`: cost value for guard [km^2]

# Notes
- this is meant to be an alternative formulation to the game rather than bandit_guard_total_cost_1 and guard_cost_2
"""
function guard_cost_3(
    g, x, u, t,
    acc_max, 
    acc_max_penalty_weight, 
    bg_dist_weight, 
    lb_dist_weight,
    bg_speed_weight)

    # minimize distance and relative speed between bandit and guard
    cost = bg_dist_weight * bandit_guard_dist_sqr(x)
    cost += bg_speed_weight * bandit_guard_speed_sqr(x)

    # maximize bandits distance from the lady
    cost -= lb_dist_weight * lady_bandit_dist_sqr(x)

    # penalize control input based on quadratic of acceleration
    cost += accel_quad_penalty(u[4:6], acc_max, acc_max_penalty_weight)

    return cost
end

# must define function for x and y position coordinates of LBGSystem
# otherwise some error is thrown by ilqgames (just using lady xy position for now)
xyindex(cs::LBGSystem) = SVector(1, 2)


"""
    solve_lady_bandit_guard_costtype_3(TODO)

Use iLQGames to compute equilibrium strategy for bandit and guard in LBG1 differential game using type-3 cost functions

# Arguments
- `bandit_acc_max::AbstractFloat`: bandit's thrust-limited maximum propulsive acceleration of bandit [km/s/s]
- `bandit_acc_max_penalty_weight::AbstractFloat`: cost weight of bandit's thrust constraint violation [km^2]
- `guard_acc_max::AbstractFloat`: guard's thrust-limited maximum propulsive acceleration of bandit [km/s/s]
- `guard_acc_max_penalty_weight::AbstractFloat`: cost weight of guard's thrust constraint violation [km^2]
- `bg_dist_weight:: Float`: scaling factor bandit-guard distance cost [-]
- `lb_dist_weight:: Float`: scaling factor lady-bandit distance cost [-]
- `bg_speed_weight:: Float`: scaling factor bandit-guard relative speed cost [s^2]
- `lb_speed_weight:: Float`: scaling factor lady-bandit relative speed cost [s^2]
- `kwargs::Dict`: see solve_lady_bandit_guard for full argument list

# Returns
- see solve_lady_bandit_guard return list
"""

function solve_lady_bandit_guard_costtype_3(; 
    bandit_acc_max::AbstractFloat,
    bandit_acc_max_penalty_weight::AbstractFloat, 
    guard_acc_max::AbstractFloat, 
    guard_acc_max_penalty_weight::AbstractFloat, 
    bg_dist_weight::AbstractFloat,
    lb_dist_weight::AbstractFloat,
    bg_speed_weight::AbstractFloat,
    lb_speed_weight::AbstractFloat,
    kwargs...)

    # --- setup type-1 costs
    costs = (

    # bandit cost function
    FunctionPlayerCost((g, x, u, t) -> 
        bandit_cost_3(g, x, u, t, 
            bandit_acc_max, 
            bandit_acc_max_penalty_weight, 
            bg_dist_weight, 
            lb_dist_weight,
            lb_speed_weight)),

    # guard cost function
    FunctionPlayerCost((g, x, u, t) -> 
        guard_cost_3(g, x, u, t, 
            guard_acc_max, 
            guard_acc_max_penalty_weight, 
            bg_dist_weight, 
            lb_dist_weight,
            bg_speed_weight)),
    )

    # --- call generalized solver with keyword type forwarding
    return solve_lady_bandit_guard(costs; kwargs...)

end

"""
    solve_lady_bandit_guard(TODO)

Use iLQGames to compute equilibrium strategy for bandit and guard in LBG1 differential game

# Arguments
- `costs::AbstractArray{}`: tuple of cost functions for each player
- `t_step::AbstractFloat`: time step length [s]
- `n_steps::AbstractArray`: time horizon over which game is solved [s]
- `mu::Float`: standard gravitational constant of central body [km^3/s^2]
- `ladyX0::AbstractArray{Float64}`: initial state of lady satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
- `banditX0::AbstractArray{Float64}`: initial state of bandit satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
- `guardX0::AbstractArray{Float64}`: initial state of guard satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
- `solver_max_n_iter::Int`: Iteration is aborted if this number is exceeded
- `solver_state_regularization::AbstractFloat`: The regularization term for the state cost quadraticization
- `solver_control_regularization::AbstractFloat`: The regularization term for the control cost quadraticization

# Returns
- `conv::Bool`: true if converged to Nash equilibrium
- `bandit_pos::AbstractArray{Float64}`: solved trajectory of bandit's position [km] in cbci coords. Entry i is (px, py, pz) at ith timestep
- `bandit_vel::AbstractArray{Float64}`: solved trajectory of bandit's velocity [km/s] in cbci coords. Entry i is (vx, vy, vz) at ith timestep
- `bandit_acc::AbstractArray{Float64}`: solved trajectory of bandit's thrust-based acceleration [km/s/s] in cbci coords. Entry i is (ax, ay, az) at ith timestep
- `guard_pos::AbstractArray{Float64}`: solved trajectory of bandit's position [km] in cbci coords. Entry i is (px, py, pz) at ith timestep
- `guard_vel::AbstractArray{Float64}`: solved trajectory of bandit's velocity [km/s] in cbci coords. Entry i is (vx, vy, vz) at ith timestep
- `guard_acc::AbstractArray{Float64}`: solved trajectory of bandit's thrust-based acceleration [km/s/s] in cbci coords. Entry i is (ax, ay, az) at ith timestep
- `lady_pos::AbstractArray{Float64}`: solved trajectory of bandit's position [km] in cbci coords. Entry i is (px, py, pz) at ith timestep
- `lady_vel::AbstractArray{Float64}`: solved trajectory of bandit's velocity [km/s] in cbci coords. Entry i is (vx, vy, vz) at ith timestep
"""
function solve_lady_bandit_guard(costs;
    t_step::AbstractFloat,
    n_steps::Int,
    mu::AbstractFloat,
    ladyX0::AbstractArray{Float64},
    banditX0::AbstractArray{Float64},
    guardX0::AbstractArray{Float64},
    solver_max_n_iter::Int,
    solver_state_regularization::AbstractFloat,
    solver_control_regularization::AbstractFloat
)

    # --- setup the game dynamics
    # Note: cannot use ProductSystem because lady satellite has different dynamics (no controls)
    # than those of the bandit and guard
    lbg_dyn = LBGSystem{t_step}(mu)

    # --- indices of each player's controls
    # Player 1: Bandit 
    # Player 2: Guard
    uids = (SVector(1,2,3), SVector(4,5,6))

    # --- encode initial conditions
    ladyX0 = SVector(ladyX0...)
    banditX0 = SVector(banditX0...)
    guardX0 = SVector(guardX0...)
    gameX0 = SVector([ladyX0 ; banditX0 ; guardX0]...) 

    # --- setup game definition
    game = GeneralGame(n_steps, uids, lbg_dyn, costs)

    # --- create a solver
    solver = iLQSolver(game, 
        max_n_iter = solver_max_n_iter,
        state_regularization = solver_state_regularization, 
        control_regularization = solver_control_regularization
    )

    # --- solve
    conv, traj, _ = solve(game, solver, gameX0)

    # --- parse out each satellites states and controls
    lady_pos = hcat([traj.x[i][1:3] for i in 1:length(traj.x)]...)      
    lady_vel = hcat([traj.x[i][4:6] for i in 1:length(traj.x)]...)           

    bandit_pos = hcat([traj.x[i][7:9] for i in 1:length(traj.x)]...)        
    bandit_vel = hcat([traj.x[i][10:12] for i in 1:length(traj.x)]...)        
    bandit_acc = hcat([traj.u[i][1:3] for i in 1:length(traj.u)]...) 
    
    guard_pos = hcat([traj.x[i][13:15] for i in 1:length(traj.x)]...)        
    guard_vel = hcat([traj.x[i][16:18] for i in 1:length(traj.x)]...)        
    guard_acc = hcat([traj.u[i][4:6] for i in 1:length(traj.u)]...) 

    return (conv, 
        bandit_pos, bandit_vel, bandit_acc,
        guard_pos, guard_vel, guard_acc,
        lady_pos, lady_vel
    )
    
end

function debug_solve_lady_bandit_guard()
    printstyled("Running solver debug function\n", color=:cyan)
    sleep(1.0)
    return true, vcat(ones(1,10), zeros(1,10), zeros(1,10))
end