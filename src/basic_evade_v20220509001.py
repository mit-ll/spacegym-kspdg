# This script is for a two-vessl, in-orbit pursuit-evasion scenario.
#
# It is expected that a human operator controls the pursuer
# while this script controls the evader
#
# The evasion algorithm is very simplistic:
# if the pursuer comes within a specified radius, the
# evader simply burns orbit-normal direction to evade

import time

import krpc
import numpy as np

# set parameters for evasion
DIST_THRESHOLD = 50.0

# establish krpc connect to send remote commands
conn = krpc.connect(name="basic_evader")
print("Connected to kRPC server")

# get vessel objects
vesE, vesP = conn.space_center.vessels[:2]

# Set the pursuer as the the active (i.e. human-controlled) vessel
# and target evader
conn.space_center.active_vessel = vesP
conn.space_center.target_vessel = vesE

# set the evader to stability assist and orient in orbit-normal direction
vesE.control.sas = True
vesE.control.sas_mode = vesE.control.sas_mode.anti_normal
print("Re-orienting Evader...")
time.sleep(10)  # give time to re-orient

# actuate RCS thrusters
vesE.control.rcs = True

# run basic evasion loop
print("Executing evasion loop")
was_evading = False
while True:
    # get distance to pursuer
    p_vesE_vesP__vesP = vesE.position(vesP.reference_frame)
    d_vesE_vesP = np.linalg.norm(p_vesE_vesP__vesP)

    # if pursuer is too close, evade in orbit-normal direction
    if d_vesE_vesP < DIST_THRESHOLD:
        vesE.control.forward = 1.0
        if not was_evading:
            print("Pursuer detected! Executing evasive maneuvers")
        was_evading = True
    else:
        vesE.control.forward = 0.0
        if was_evading:
            print("No pursuer in range. Zeroing thrust")
        was_evading = False
