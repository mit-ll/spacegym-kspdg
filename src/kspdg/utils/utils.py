# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import numpy as np

from copy import deepcopy
from typing import List

from kspdg.utils import constants as CONST

def compute_R_rhntw_rhcbci(p_ref_cb__rhcbci, v_ref_cb__rhcbci, eps):
    '''compute rotation matrix R from right-handed 
        celestial-body-centered-ineertial (rhcbci) coords to 
        right-handed radial-out, velocity-tangent, orthogonal (NTW frame 
        defined in Vallado, 3rd Edition, Sec 3.3.3)

    Note: CBCI coords are roughly equivalent to "ECI" (earth-centered inertial)
    and "IJK" coords in Vallado, 3rd ed, 3.3.2 and BMW 2.2.2; but we 
    don't use the "ECI" terminology because we aren't necessarily working
    in Earth's orbit (i.e. Kerbin with no axial tilt and no defined vernal equinox)

    Args:
        p_ref_cb__rhcbci : List[float]
            position (wrt central body) within reference orbit expressed in rhcbci (rhntw origin) 
        v_ref_cb__rhcbci : List[float]
            velocity (wrt central body) within reference orbit expressed in rhcbci (rhntw tangent direction) 
        eps : float
            tolerance of vector magnitude and alignment to identify degenerate cases

    Returns:
        R_rhntw_rhcbci : ndarray[float]
            3x3 matrix that rotates a column vector expressed in rhcbci into rhntw frame, when 
            right-multiplied; i.e. a__rhntw = R_rhntw_rhcbci @ a__rhcbci 
    '''

    # abbreviate and convert to numpy for brevity and simplicity
    p_ref = np.asarray(p_ref_cb__rhcbci, dtype=float)
    v_ref = np.asarray(v_ref_cb__rhcbci, dtype=float)

    # Unit Tangential (T): parallel to reference orbit velocity
    vnorm = np.linalg.norm(v_ref)
    if vnorm < eps:
        raise ValueError("Reference velocity magnitude too small to define tangential axis.")
    t_hat = v_ref / vnorm

    # Unit Orbit-Normal (W): orthogonal to reference position and velocity
    w_vec = np.cross(p_ref, v_ref)
    w_norm = np.linalg.norm(w_vec)

    if w_norm < eps:
        # Degenerate case: r_hat nearly parallel to t_hat (e.g., strongly radial motion).
        raise ValueError("Colinear reference position and velocity, unable to define NTW frame.")

    w_hat = w_vec / w_norm

    # Unit Normal (N): completes right-handed system
    # generally pointing in radial-out direction, but not parallel to radial-out vector in non-circular orbits
    n_hat = np.cross(t_hat, w_hat)

    # Rotation matrix rows are the basis vectors (N,T,W) in inertial coords
    R_rhntw_rhcbci = np.vstack((n_hat, t_hat, w_hat))  # 3x3

    return R_rhntw_rhcbci

def convert_rhcbci_to_rhntw(r_tar__rhcbci, p_ref_cb__rhcbci, v_ref_cb__rhcbci, eps=1e-9):
    '''convert vector in right-handed celestial-body-centered-inertial coords 
        to right-handed NTW coords (Vallado, 3rd Edition, Sec 3.3.3)

    Note: CBCI coords are roughly equivalent to "ECI" (earth-centered inertial)
    and "IJK" coords in Vallado, 3rd ed, 3.3.2 and BMW 2.2.2; but we 
    don't use the "ECI" terminology because we aren't necessarily working
    in Earth's orbit (i.e. Kerbin with no axial tilt and no defined vernal equinox)
    
    Args:
        r_tar__rhcbci : List[float]
            3-vector represented in right-handed CBCI coords to be converted to rhntw
            Note that this is treated as an arbitrary vector for which only the frame is converted,
            we do not assume it to be some absolute position that must be made relative to the 
            rhntw origin
        p_ref_cb__rhcbci : List[float]
            position (wrt central body) within reference orbit expressed in rhcbci (rhntw origin) 
        v_ref_cb__rhcbci : List[float]
            velocity (wrt central body) within reference orbit expressed in rhcbci (rhntw tangent direction) 
        eps : float
            tolerance of vector magnitude and alignment to identify degenerate cases

    Returns:
        r_tar__rhntw : List[float]
            3-vector represented in right-handed NTW frame
    '''

    # Compute rotation matrix
    R_rhntw_rhcbci = compute_R_rhntw_rhcbci(
        p_ref_cb__rhcbci=p_ref_cb__rhcbci, 
        v_ref_cb__rhcbci=v_ref_cb__rhcbci,
        eps=eps)

    # Components in NTW: project target vector onto (n̂, t̂, ŵ)
    r_tar__rhntw = R_rhntw_rhcbci @ r_tar__rhcbci
    return r_tar__rhntw

def convert_rhntw_to_rhcbci(r_tar__rhntw, p_ref_cb__rhcbci, v_ref_cb__rhcbci, eps=1e-9):
    '''convert vector in right-handed NTW coords to right-handed celestial-body-centered-inertial 
        coords (inverse of convert_rhcbci_to_rhntw)

    Args:
        r_tar__rhntw : List[float]
            3-vector represented in right-handed NTW coords to be converted to rhcbci.
            This is treated as an arbitrary vector in the NTW frame; we do not assume it
            is a relative position that must be shifted by the frame origin.
        p_ref_cb__rhcbci : List[float]
            position (wrt central body) within reference orbit expressed in rhcbci 
            (defines the NTW frame together with v_ref_cb__rhcbci)
        v_ref_cb__rhcbci : List[float]
            velocity (wrt central body) within reference orbit expressed in rhcbci 
            (defines the tangential direction of the NTW frame)

    Returns:
        r_tar__rhcbci : numpy.ndarray
            3-vector represented in right-handed CBCI frame
    '''

    # Compute rotation matrix
    R_rhntw_rhcbci = compute_R_rhntw_rhcbci(
        p_ref_cb__rhcbci=p_ref_cb__rhcbci, 
        v_ref_cb__rhcbci=v_ref_cb__rhcbci,
        eps=eps)

    # Inverse mapping: r_rhcbci = R^T * r_ntw
    r_tar__rhcbci = R_rhntw_rhcbci.T @ r_tar__rhntw
    return r_tar__rhcbci


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
