# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import numpy as np

import astropy.units as astro_units

from copy import deepcopy
from typing import List

from poliastro.iod import izzo
from astropy.units import Quantity
from poliastro.core.propagation.vallado import vallado as propagate_vallado

from kspdg.utils import constants as CONST

def convert_lhcbci_to_rhcbci(v__lhcbci: List[float]) -> List[float]:
    '''convert vector in left-handed celestial-body-centered-inertial coords 
        to right-handed cbci coords

    Note: CBCI coords are roughly equivalent to "ECI" (earth-centered inertial)
    and "IJK" coords in Vallado, 3rd ed, 3.3.2 and BMW 2.2.2; but we 
    don't use the "ECI" terminology because we aren't necessarily working
    in Earth's orbit (i.e. Kerbin with no axial tilt and no defined vernal equinox)
    
    Args:
        v__lhcbci : List[float]
            3-vector represented in left-handed CBCI coords

    Returns:
        v__rhcbci : List[float]
            3-vector represented in right-handed CBCI coords
    '''
    v__rhcbci = deepcopy(v__lhcbci)
    v__rhcbci[1] = deepcopy(v__lhcbci[2])
    v__rhcbci[2] = deepcopy(v__lhcbci[1])
    return v__rhcbci

def convert_rhcbci_to_lhcbci(v__rhcbci: List[float]) -> List[float]:
    '''convert vector in right-handed celestial-body-centered-inertial coords 
        to left-handed cbci coords

    Note: CBCI coords are roughly equivalent to "ECI" (earth-centered inertial)
    and "IJK" coords in Vallado, 3rd ed, 3.3.2 and BMW 2.2.2; but we 
    don't use the "ECI" terminology because we aren't necessarily working
    in Earth's orbit (i.e. Kerbin with no axial tilt and no defined vernal equinox)
    
    Args:
        v__rhcbci : List[float]
            3-vector represented in right-handed CBCI coords

    Returns:
        v__lhcbci : List[float]
            3-vector represented in left-handed CBCI coords
    '''
    v__lhcbci = deepcopy(v__rhcbci)
    v__lhcbci[1] = deepcopy(v__rhcbci[2])
    v__lhcbci[2] = deepcopy(v__rhcbci[1])
    return v__lhcbci

def convert_lhntw_to_rhntw(v__lhntw: List[float]) -> List[float]:
    '''convert vector in left-handed NTW coordinates 
    to standard right-handed NTW coordinates

    Args:
        v__lhntw : List[float]
            3-vector represented in left-handed NTW coords
    
    Returns:
        v__rhntw : List[float]
            3-vector represented in right-handed NTW coords

    Ref:
        Left-handed system: https://krpc.github.io/krpc/tutorials/reference-frames.html#introduction
        Right-handed system: Vallado, 3rd Edition, Sec 3.3.3
    '''
    v__rhntw = deepcopy(v__lhntw)
    v__rhntw[0] = -v__rhntw[0]
    return v__rhntw

def convert_rhntw_to_lhntw(v__rhntw: List[float]) -> List[float]:
    '''convert vector in right-handed NTW coordinates 
    to standard left-handed NTW coordinates

    Args:
        v__rhntw : List[float]
            3-vector represented in right-handed NTW coords
    
    Returns:
        v__lhntw : List[float]
            3-vector represented in left-handed NTW coords

    Ref:
        Left-handed system: https://krpc.github.io/krpc/tutorials/reference-frames.html#introduction
        Right-handed system: Vallado, 3rd Edition, Sec 3.3.3
    '''
    v__lhntw = deepcopy(v__rhntw)
    v__lhntw[0] = -v__lhntw[0]
    return v__lhntw

def convert_lhbody_to_rhbody(v__lhbody: List[float]) -> List[float]:
    '''convert vector from left-hand to right-hand vessel body coords

    Args:
        v__lhbody : List[float]
            3-vector represented in left-handed vessel body coords (right, forward, down)

    Returns:
        v__rhbody : List[float]
            3-vectory represented in right-handed vessel body coords (forward, right, down)

    Ref:
        Left-handed vessel body coords: 
            https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
    '''
    v__rhbody = deepcopy(v__lhbody)
    v__rhbody[0] = v__lhbody[1]
    v__rhbody[1] = v__lhbody[0]
    return v__rhbody

def convert_rhbody_to_lhbody(v__rhbody: List[float]) -> List[float]:
    '''convert vector from left-hand to right-hand vessel body coords

    Args:
        v__rhbody : List[float]
            3-vectory represented in right-handed vessel body coords (forward, right, down)

    Returns:
        v__lhbody : List[float]
            3-vector represented in left-handed vessel body coords (right, forward, down)

    Ref:
        Left-handed vessel body coords: 
            https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
    '''
    v__lhbody = deepcopy(v__rhbody)
    v__lhbody[0] = v__rhbody[1]
    v__lhbody[1] = v__rhbody[0]
    return v__lhbody

def solve_lambert(p0__rhcbci, pf__rhcbci, time_of_flight):
    """ Solve lambert targeting problem to determin velocities at two points

    Args:
        p0__rhcbci : ArrayLike (len=3)
            position vector of initial orbital state in right-handed 
            celestial-body-centered-inertial coords [m]
        pf__rhcbci : ArrayLike
            position vector of final orbital state in right-handed 
            celestial-body-centered-inertial coords [m]
        time_of_flight : float
            time of flight to propagate [sec]
    
    Returns:
        v0__rhcbci : ArrayLike (len=3)
            velocity vector of initial orbital state in right-handed 
            celestial-body-centered-inertial coords [m/s]
        vf__rhcbci : ArrayLike
            velocity vector of final orbital state in right-handed 
            celestial-body-centered-inertial coords [m/s]

    Ref: 
        https://docs.poliastro.space/en/stable/examples/Revisiting%20Lamberts%20problem%20in%20Python.html
        https://docs.poliastro.space/en/stable/autoapi/poliastro/iod/izzo/index.html
    """

    # "Quantify" mu, init position, final position and time of flight
    q_mu = Quantity(CONST.KERBIN.MU, astro_units.m**3 / astro_units.s**2)
    q_p0 = Quantity(p0__rhcbci, astro_units.m)
    q_pf = Quantity(pf__rhcbci, astro_units.m)
    q_tf = Quantity(time_of_flight, astro_units.s)

    # call lambert solver (assumes defaults, i.e. "short way" around)
    q_v0, q_vf = izzo.lambert(k=q_mu, r0=q_p0, r=q_pf, tof=q_tf)

    # de-"Quantify" velocity vectors
    v0__rhcbci = q_v0.to_value(astro_units.m/astro_units.s)
    vf__rhcbci = q_vf.to_value(astro_units.m/astro_units.s)

    return v0__rhcbci, vf__rhcbci

def propagate_orbit_tof(p0__rhcbci, v0__rhcbci, time_of_flight):
    """propagate an orbit based on a time of flight

    Args:
        p0__rhcbci : ArrayLike (len=3)
            position vector of initial orbital state in right-handed 
            celestial-body-centered-inertial coords [m]
        v0__rhcbci : ArrayLike (len=3)
            velocity vector of initial orbital state in right-handed 
            celestial-body-centered-inertial coords [m/s]
        time_of_flight : float
            time of flight to propagate [sec]
    
    Returns:
        pf__rhcbci : ArrayLike
            position vector of final orbital state in right-handed 
            celestial-body-centered-inertial coords [m]
        vf__rhcbci : ArrayLike
            velocity vector of final orbital state in right-handed 
            celestial-body-centered-inertial coords [m/s]

    Refs:
        https://docs.poliastro.space/en/latest/autoapi/poliastro/core/propagation/vallado/index.html
    """

    f, g, fdot, gdot = propagate_vallado(
        k = CONST.KERBIN.MU,
        r0 = p0__rhcbci,
        v0 = v0__rhcbci,
        tof = time_of_flight,
        numiter=32,
    )

    pf__rhcbci = f * p0__rhcbci + g * v0__rhcbci
    vf__rhcbci = fdot * p0__rhcbci + gdot * v0__rhcbci

    return pf__rhcbci, vf__rhcbci

def estimate_capture_dv(p0_prs, v0_prs, p0_evd, v0_evd, tof):
    """ Propagate orbits and use one-off lambert targeting to estimate delta-v to capture
        evader by pursuer

    Args:
        p0_prs : ArrayLike (len=3)
            position vector of initial orbital state of pursuer in right-handed 
            celestial-body-centered-inertial coords [m]
        v0_prs : ArrayLike (len=3)
            velocity vector of initial orbital state of pursuer in right-handed 
            celestial-body-centered-inertial coords [m/s]
        p0_evd : ArrayLike
            position vector of initial orbital state of evader in right-handed 
            celestial-body-centered-inertial coords [m]
        v0_evd : ArrayLike
            velocity vector of initial orbital state of evader in right-handed 
            celestial-body-centered-inertial coords [m/s]
        tof : float
            time of flight to propagate [sec]
    
    Returns:
        dv0 : float
            delta-v at initial state to put pursuer on capturing transfer orbit
        dvf : float
            delta-v at final state to put pursuer on capturing transfer orbit
    """
    # propagate evader's current orbit (using episode timeout length)
    # https://poliastro-py.readthedocs.io/en/latest/api/safe/twobody/propagation.html#poliastro.twobody.propagation.kepler
    p0_e_cb__rhcbci = p0_evd
    v0_e_cb__rhcbci = v0_evd
    pf_e_cb__rhcbci, vf_e_cb__rhcbci = propagate_orbit_tof(
        p0__rhcbci=p0_e_cb__rhcbci, 
        v0__rhcbci=v0_e_cb__rhcbci, 
        time_of_flight=tof
    )

    # solve lambert's problem for pursuer from current state to evader's
    # propagated position (short way around)
    p0_p_cb__rhcbci = p0_prs
    v0_p_cb__rhcbci = v0_prs
    v0trans_p_cb__rhcbci, vftrans_p_cb__rhcbci = solve_lambert(
        p0__rhcbci=p0_p_cb__rhcbci,
        pf__rhcbci=pf_e_cb__rhcbci,
        time_of_flight=tof
    )

    # compute delta-v for pursuer from current velocity to lambert's 
    # transfer orbit velocity
    dv0 = np.linalg.norm(v0trans_p_cb__rhcbci - v0_p_cb__rhcbci)
    dvf = np.linalg.norm(vf_e_cb__rhcbci - vftrans_p_cb__rhcbci)

    return dv0, dvf

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

###########################################################################

## Broken/Non-functional code. Implementation details kept for posterity

###########################################################################

def BROKEN_get_rcs_net_directional_properties(vessel, burn_vec__rhbody):
    '''compute the max thrust, specific impulse, and fuel consumption of rcs along a burn vector

    BROKEN: DO NOT USE
        This function is left in place because, in principle, it should work; however
        There seems to be fundamental errors with krpc's handling of thruster objects
        for RCS and engines
    
    Args:
        vessel : krpc.SpaceCenter.Vessel
            vessel object defined by krpc
            https://krpc.github.io/krpc/python/api/space-center/vessel.html#SpaceCenter.Vessel
        burn_vec__rhbody : List[float]
            burn vector for rcs in right-hand vessel body coords (rhvbody)
        
    Returns:
        net_max_thrust : float
            [N] total thrust possible (max) from all rcs thrusters
        net_specific_impulse : float
            [s] combined specific impulse from all rcs thrusters
            https://krpc.github.io/krpc/tutorials/parts.html#combined-specific-impulse
        net_max_fuel_consumption : float
            [kg/s] total mass flow possible from all rcs thrusters 
    '''

    # collect all thrusters on craft
    thrusters = [t for r in vessel.parts.rcs for t in r.thrusters]

    # collect directional properties of all thrusters
    dir_thrust_props = [BROKEN_get_thruster_directional_properties(t, burn_vec__rhbody) for t in thrusters]

    # compute total max thrust and fuel consumption from all thrusters
    net_max_effective_thrust = sum(dtp[0] for dtp in dir_thrust_props)
    net_max_effective_fuel_consumption = sum(dtp[1] for dtp in dir_thrust_props)

    # compute total effective specific impulse
    net_specific_impulse = net_max_effective_thrust / (net_max_effective_fuel_consumption * CONST.G0)
    return net_max_effective_thrust, net_max_effective_fuel_consumption, net_specific_impulse

def BROKEN_get_thruster_directional_properties(thruster, burn_vec__rhbody):
    '''compute max thrust and fuel consumption of a thruster along a specific burn vector

    BROKEN: DO NOT USE
        This function is left in place because, in principle, it should work; however
        There seems to be fundamental errors with krpc's handling of thruster objects
        for RCS and engines

    Args:
        thruster : krpc.SpaceCenter.Thruster
            thruster object to be analyzed
        burn_vec__rhbody : List[float]
            desired burn vector for thruster in right-hand vessel body coords (rhvbody)

    Returns: 
        max_effective_thrust : float
            [N] total thrust that is effectively possible along body-fixed burn vector
        max_effective_fuel_consumption : float
            [kg/s] maximum fuel consumption effectively possible when burning along body-fixed vector
    
    Refs:
        https://krpc.github.io/krpc/python/api/space-center/parts.html#thruster
    '''
    # convert desired burn vector into krpc's left handed body coords
    bv__lhbody = convert_rhbody_to_lhbody(burn_vec__rhbody)

    # get thrusters thrust orientation in lh body coords
    # https://krpc.github.io/krpc/python/api/space-center/vessel.html#SpaceCenter.Vessel.reference_frame
    tv__lhbody = thruster.thrust_direction(thruster.part.vessel.reference_frame)

    # determine maximum effective thrust thruster can produce along burn vector
    # thruster cannot produce negative thrust, thus take max with 0
    max_effective_thrust = thruster.part.rcs.max_thrust * max(0.0, np.dot(bv__lhbody, tv__lhbody))

    # determine maximum effective fuel consumption
    max_effective_fuel_consumption = max_effective_thrust / (CONST.G0 * thruster.part.rcs.specific_impulse)

    return max_effective_thrust, max_effective_fuel_consumption
