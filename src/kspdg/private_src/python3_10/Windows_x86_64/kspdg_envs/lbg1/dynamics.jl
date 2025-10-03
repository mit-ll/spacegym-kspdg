# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

"""
dynamics.jl 

functions for definition equations of motion of vehicles in orbit

"""

"""
    satellite_eom__rhcbci(t, x_state, u_ctrl, mu)

Calculate state derivative (equation of motion) of satellite under gravitational and propulsive acceleration
expressed in right-handed celestial-body-centered inertial coordinates

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