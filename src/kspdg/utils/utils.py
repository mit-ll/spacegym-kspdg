# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import numpy as np

from copy import deepcopy
from typing import List

_G0 = 9.80665 # standard gravity [m/s/s]

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
    net_specific_impulse = net_max_effective_thrust / (net_max_effective_fuel_consumption * _G0)
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
    max_effective_fuel_consumption = max_effective_thrust / (_G0 * thruster.part.rcs.specific_impulse)

    return max_effective_thrust, max_effective_fuel_consumption

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
