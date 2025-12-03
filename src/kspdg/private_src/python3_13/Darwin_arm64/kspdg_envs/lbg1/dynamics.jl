# Copyright (c) 2025, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

"""
dynamics.jl 

functions for definition equations of motion of vehicles in orbit

"""

using iLQGames: SMatrix

"""
    satellite_eom__rhcbci(t, x_state, u_ctrl, mu)

Contiuous-time state derivative (equation of motion) of satellite under 
gravitational and propulsive acceleration expressed in 
right-handed celestial-body-centered inertial coordinates

# Arguments
- `t::Float64`: The current time (not used in this example but required for ODE solver compatibility).
- `x_state::Vector{Float64}`: A vector representing the current state of the satellite, `[x, y, z, vx, vy, vz]`, where 
    - `(px, py, pz)` are the position coordinates in inertial frame [km]
    - `(vx, vy, vz)` are the velocity components in inertial frame [km/s]
- `u_ctrl::Vector{Float64}`: A vector representing the acceleration due to propulsion, 
    - `[ax_p, ay_p, az_p]` acceleration components due to propulstion in inertial frame [km/s^2]
- `mu::Float64`: The standard gravitational parameter of the central body [km^3/s^2]

# Returns
- `dx_state::Vector{Float64}`: A vector representing the time derivatives of the state, `[vx, vy, vz, ax, ay, az]`, where 
    - `(vx, vy, vz)` are the velocity components in inertial frame [km/s]
    - `(ax, ay, az)` are the total acceleration components in inertial frame [km/s^2]

"""
function satellite_eom__rhcbci(t, x_state, u_ctrl, mu)

    # Unpack the state vector
    px, py, pz, vx, vy, vz = x_state

    # Compute the distance from the central body
    r = sqrt(px^2 + py^2 + pz^2)

    # Compute gravitational acceleration components
    ax_g = -mu * px / r^3
    ay_g = -mu * py / r^3
    az_g = -mu * pz / r^3

    # Total acceleration is the sum of gravitational and thrust accelerations
    ax = ax_g + u_ctrl[1]
    ay = ay_g + u_ctrl[2]
    az = az_g + u_ctrl[3]

    # Return the time derivatives of the state
    return [vx, vy, vz, ax, ay, az]
end

"""
    lbg_eom1__rhcbci(t, x_state, u_ctrl, mu)

Calculate state derivative (equation of motion) of lady-bandit-guard 3-satellite system in inertial coords
Assumes passive (non-maneuvering) lady satellite and 

# Arguments
- `t::Float64`: The current time (not used in this example but required for ODE solver compatibility).
- `x_state::AbstractArray{Float}`: A vector representing the current state of the satellites, where
    - `x_state[1:6]`: lady satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x_state[7:12]`: bandit satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
    - `x_state[13:18]`: guard satellite position [km] and velocity [km/s] in cbci coords (px, py, pz, vx, vy, vz)
- `u_ctrl::AbstractArray{Float}`: A vector representing the acceleration due to propulsion for bandit and guard
    - `u_ctrl[1:3]`: x,y,z acceleration components of bandit due to propulstion in inertial frame [km/s^2]
    - `u_ctrl[4:6]`: x,y,z acceleration components of guard due to propulstion in inertial frame [km/s^2]
- `mu::Float64`: The standard gravitational parameter of the central body [km^3/s^2]

# Returns
- `dx_state::AbstractArray{Float64}`: A vector representing the time derivatives of the state, `[vx, vy, vz, ax, ay, az]`, where 
    - `dx_state[1:6]`: lady satellite velocity [km/s] and acceleration [km/s/s] in cbci coords (vx, vy, vz, ax, ay, az)
    - `dx_state[7:12]`: bandit satellite velocity [km/s] and acceleration [km/s/s] in cbci coords (vx, vy, vz, ax, ay, az)
    - `dx_state[13:18]`: guard satellite velocity [km/s] and acceleration [km/s/s] in cbci coords (vx, vy, vz, ax, ay, az)

"""
function lbg_eom1__rhcbci(t, x_state, u_ctrl, mu)

    # lady equations of motion (non-maneuvering)
    dx_lady = satellite_eom__rhcbci(t, x_state[1:6], zeros(3), mu)

    # bandit and guard equations of motion
    dx_bandit = satellite_eom__rhcbci(t, x_state[7:12], u_ctrl[1:3], mu)
    dx_guard = satellite_eom__rhcbci(t, x_state[13:18], u_ctrl[4:6], mu)

    # concat and return
    return vcat(dx_lady, dx_bandit, dx_guard)

end

"""
    cw_discrete__rhntw(dt, n)

Return discrete-time (A, B) matrices for the 3D Clohessy–Wiltshire (Hill) model
under zero-order hold (ZOH) on the input over timestep `dt`. Expressed in 
right-hand NTW (normal-out, velocity-tangent, orthogonal) frame.

Note that these matrices are based upon the analytic solution to the 
continuous-time CW differential equations of form xdot = Ax + Bu assuming
constant input acceleration vector u over the timestep dt. Therefore, 
they are not Euler approximations of A and B, but rather exact solutions

# Arguments
- `dt::Float64`: time-step to next state [s]
- `n::Float64`: orbital rate of reference orbit [rad/s]

# Returns
- `A::Matrix{T}`: Feedforward terms of linear system
- `B::Matrix{T}`: feedback control terms of linear system

# Notes

State (NTW / Hill frame, chief on circular orbit):
    x = [x, y, z, xdot, ydot, zdot]

Input (accelerations in same NTW frame):
    u = [a_x, a_y, a_z]

Continuous-time dynamics:
    xdot = A_c * x + B_c * u

Discretized (ZOH):
    x_{k+1} = A * x_k + B * u_k

You provide the mean motion n [rad/s] directly.
"""
function cw_discrete__rhntw(dt::Real, n::Real)
    @assert dt > 0 "dt must be positive"
    @assert n > 0 "n must be positive"

    T  = typeof(n * dt)     # promote type
    s  = n * dt
    c  = cos(s)
    s_sin = sin(s)
    n2 = n^2

    # -----------------------
    # Discrete A = Phi(dt)
    # -----------------------
    # Blocks: [ r ; v ] = [Phi_rr  Phi_rv; Phi_vr  Phi_vv] * [ r0 ; v0 ]
    phi_rr = Matrix{T}(undef, 3, 3)
    phi_rr[1,1] = 4 - 3c
    phi_rr[1,2] = 0
    phi_rr[1,3] = 0

    phi_rr[2,1] = 6 * (s_sin - s)
    phi_rr[2,2] = 1
    phi_rr[2,3] = 0

    phi_rr[3,1] = 0
    phi_rr[3,2] = 0
    phi_rr[3,3] = c

    phi_rv = Matrix{T}(undef, 3, 3)
    phi_rv[1,1] =  s_sin / n
    phi_rv[1,2] =  2 * (1 - c) / n
    phi_rv[1,3] =  0

    phi_rv[2,1] =  2 * (c - 1) / n
    phi_rv[2,2] =  (4 * s_sin - 3s) / n
    phi_rv[2,3] =  0

    phi_rv[3,1] =  0
    phi_rv[3,2] =  0
    phi_rv[3,3] =  s_sin / n

    phi_vr = Matrix{T}(undef, 3, 3)
    phi_vr[1,1] =  3n * s_sin
    phi_vr[1,2] =  0
    phi_vr[1,3] =  0

    phi_vr[2,1] =  6n * (c - 1)
    phi_vr[2,2] =  0
    phi_vr[2,3] =  0

    phi_vr[3,1] =  0
    phi_vr[3,2] =  0
    phi_vr[3,3] = -n * s_sin

    phi_vv = Matrix{T}(undef, 3, 3)
    phi_vv[1,1] =  c
    phi_vv[1,2] =  2 * s_sin
    phi_vv[1,3] =  0

    phi_vv[2,1] = -2 * s_sin
    phi_vv[2,2] =  4c - 3
    phi_vv[2,3] =  0

    phi_vv[3,1] =  0
    phi_vv[3,2] =  0
    phi_vv[3,3] =  c

    A = Matrix{T}(undef, 6, 6)
    A[1:3,1:3] = phi_rr
    A[1:3,4:6] = phi_rv
    A[4:6,1:3] = phi_vr
    A[4:6,4:6] = phi_vv

    # -----------------------
    # Discrete B = Gamma(dt)
    # Gamma_r = ∫_0^dt phi_rv(tau) dtau
    # Gamma_v = ∫_0^dt phi_vv(tau) dtau
    # -----------------------
    k1 = (1 - c) / n2
    k2 = 2 * dt / n - 2 * s_sin / n2
    k3 = -2 * dt / n + 2 * s_sin / n2
    k4 = -1.5 * dt^2 + 4 * (1 - c) / n2
    kz = k1

    gamma_r = Matrix{T}(undef, 3, 3)
    gamma_r[1,1] = k1
    gamma_r[1,2] = k2
    gamma_r[1,3] = 0

    gamma_r[2,1] = k3
    gamma_r[2,2] = k4
    gamma_r[2,3] = 0

    gamma_r[3,1] = 0
    gamma_r[3,2] = 0
    gamma_r[3,3] = kz

    gamma_v = Matrix{T}(undef, 3, 3)
    gamma_v[1,1] =  s_sin / n
    gamma_v[1,2] =  2 * (1 - c) / n
    gamma_v[1,3] =  0

    gamma_v[2,1] =  2 * (c - 1) / n
    gamma_v[2,2] = -3 * dt + 4 * s_sin / n
    gamma_v[2,3] =  0

    gamma_v[3,1] =  0
    gamma_v[3,2] =  0
    gamma_v[3,3] =  s_sin / n

    B = Matrix{T}(undef, 6, 3)
    B[1:3, :] = gamma_r
    B[4:6, :] = gamma_v

    return A, B
end

"""
    lbg_cw_discrete_eom__rhntw(dt::Real, n::Real)

Discrete-time Clohessy–Wiltshire (CW) equations of motion for a
Lady–Bandit–Guard (LBG) game in a right-handed NTW frame (radial-out,
velocity-tangential, orthogonal).

The lady satellite is assumed to follow a circular reference orbit with
mean motion `n` and is **uncontrolled**; CW dynamics are defined for the
bandit and guard satellites relative to this lady orbit.

A single CW chaser has state:
    x_single = [x, y, z, xdot, ydot, zdot]

and input:
    u_single = [a_x, a_y, a_z]

This function stacks **two** such chasers (bandit, guard) into one
discrete-time LTI system:

State:
    x = [
        x_b, y_b, z_b, xdot_b, ydot_b, zdot_b,
        x_g, y_g, z_g, xdot_g, ydot_g, zdot_g
    ]  ∈ ℝ¹²

Input:
    u = [
        a_x_b, a_y_b, a_z_b,
        a_x_g, a_y_g, a_z_g
    ] ∈ ℝ⁶

Dynamics (discrete-time, ZOH):
    x_{k+1} = A * x_k + B * u_k

where the bandit and guard each follow CW dynamics with the same
reference mean motion `n`.

Internally this uses `cw_discrete__rhntw(dt, n)` for a single chaser and
forms a block-diagonal system:

    A = [ A_cw    0
          0       A_cw ]

    B = [ B_cw    0
          0       B_cw ]

# Arguments
- `dt::Real`:
    Discrete-time step [s]. Must be positive.

- `n::Real`:
    Mean motion [rad/s] of the circular reference orbit (lady satellite)
    about which the CW dynamics are defined. Both bandit and guard are
    modeled relative to this same NTW frame.

# Returns
- `A::SMatrix{T}`:
    12×12 discrete-time state transition matrix for the stacked
    bandit + guard system, where `T` is a floating-point type promoted
    from `dt` and `n`.

- `B::SMatrix{T}`:
    12×6 discrete-time input matrix. Columns 1:3 map bandit NTW
    accelerations `[a_x_b, a_y_b, a_z_b]` into the bandit state; columns
    4:6 map guard NTW accelerations `[a_x_g, a_y_g, a_z_g]` into the
    guard state.

"""
function lbg_cw_discrete_eom__rhntw(dt::Real, n::Real)
    # Single-satellite CW discrete dynamics (assumed exact ZOH)
    A_cw, B_cw = cw_discrete__rhntw(dt, n)

    T = eltype(A_cw)

    # 12x12 block-diagonal A: bandit (top-left), guard (bottom-right)
    A = zeros(T, 12, 12)
    A[1:6,   1:6]   = A_cw         # bandit
    A[7:12,  7:12]  = A_cw         # guard

    # 12x6 B: first 3 inputs -> bandit, last 3 -> guard
    B = zeros(T, 12, 6)
    B[1:6,   1:3]   = B_cw         # bandit: u_b = [a_x_b, a_y_b, a_z_b]
    B[7:12,  4:6]   = B_cw         # guard:  u_g = [a_x_g, a_y_g, a_z_g]

    # --- convert to StaticArrays at the end ---
    As = SMatrix{12,12,T}(A)
    Bs = SMatrix{12,6,T}(B)

    return As, Bs
end
