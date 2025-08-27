# Lady-Bandit-Guard Group 1 Environments (Target Guarding)

A collection of 1-v-2 target guarding environments with shared parameters/characteristics.
The terminology "Lady-Bandit-Guard (lbg)" comes from Rusnak's 2005 paper which defined a similar differential game but not in the orbital domain:
> Rusnak, Ilan. "The lady, the bandits and the body guards–a two team dynamic game." IFAC Proceedings Volumes 38, no. 1 (2005): 441-446.

Distinct environments are compositions of Lady-Guard policies and initial orbital configurations. Therefore, for example, an environment titled `lbg1_lg0_i1` is a distinct environment from `lbg1_lg0_i2`, although they will have the same behavior policies for the Lady-Guard team but different initial orbital configurations. See [Environment Naming Scheme](#environment-naming-scheme)

##  Environment Naming Scheme

This README documents/decodes the somewhat cryptic naming scheme used to uniquely identify environments. 
+ The naming scheme follows a patter of: `{GroupID}_lg{LadyGuardPolicyID}_i{InitCondID}_v{VersionNum}`
+ The primary reason for these cryptic naming schemes is brevity; it becomes ungainly and impractical to have files named things like: `LadyBanditGuardEnvGroup1_LadyNashPolicy3_GuardHeuristicPolicy4_EllipticalCircularInitOrbits12_v5`
+ *GroupID* identifies a collection of environments with shared properties. 
    + Example: `lbg1` is this group's name which represents "Lady-Bandit-Guard Environments Group 1"
    + The group name can be cryptic, verbose, or completely non-descriptive of the group of environments' properties; but it does need to be _unique_ from all other group ID's
    + The sub-group naming scheme can vary between environment groups. We use the `_lb{LadyGuardPolicyID}_i{InitCondID}` naming scheme for this particular environment group but it may be meaningless in another group
+ *LadyGuardPoliyID* identifies the policy of the Lady-Guard team within the environment
    + Example: `lgnash1` could represent an lady-guard team that uses Nash-equilibrium evasion policies, or we could use `lb5` to represent the same thing. 
    + Similar to GroupID, the exact name is somewhat arbitrary, it is really only important that LadyGuardPolicyID be _unique_ from all other Evasion ID's within the group
+ *InitCondID* identifies the initial condition (i.e. orbital parameters) of the pursuer in the environment
    + Example: `i2` could represent an initial condition where the pursuer is inclined relative to the evader but a conjunction is staged at some point in the future
    + Similar to other IDs it is most important that it is unique within the group
+ *VersionNum* is a monotonically increasing number starting from 0 used bookkeeping purposes. 
    + Any change to the group's shared parameters, evasion behiavor, and/or initial conditions that occur between version updates of the `kspdg` library should be reflected in an increment of the environment's VersionNum. 
    + This increment is necessary because any agent trained under different group params, evasion behavior, and/or initial conditions was technically trained under a different environment

## Constants Across Environments in Group

+ There is 1 "Bandit", 1 "Guard", and 1 "Lady" spacecraft
+ Agent controls the Bandit spacecraft, a scripted bot(s) controls the Lady and the Guard (but the bot's policy varies between environments within the group)
+ Bandit and Guard have identical vehicle capabilities in each scenario, Lady vehicle may have the same or different vehicle capabilities
+ Lady initial orbit constant across all sencarios/envs; Bandit and Guard initial orbits are varied across environments in the group
+ Observation and Action spaces are constant across all scenarios/envs

## Observation and Action Space:

See [`get_observation()`](https://github.com/mit-ll/spacegym-kspdg/blob/main/src/kspdg/lbg1/lbg1_base.py) for detailed description of observation vector.

See `Group1BaseEnv.vessel_step()` in [`base_envs.py`](https://github.com/mit-ll/spacegym-kspdg/blob/main/src/kspdg/base_envs.py) for detailed description of action dictionary

## Lady-Guard Policy Environment Identifiers

+ ID `lg0`: Lady and Guard are passive, no maneuvering
+ ID `lg1`: Lady is passive, Guard applies heuristic pursuit of Bandit using target-zero_vel-target relative maneuver sequence
+ ID `lg2`: Lady applies hueristic evasive maneuvers of Bandit using out-of-plane burns in order to maintian constant period of orbit, Guard applies heuristic pursuit of Bandit using target-zero_vel-target relative maneuver sequence
+ ID `lg3`: Code-obfuscated environment with passive Lady and advanced algorithms for Guard maneuvering
+ ID `lg4`: Code-obfuscated environment with passive Lady and advanced algorithms for Guard maneuvering
+ ID `lg5`: Code-obfuscated environment with passive Lady and advanced algorithms for Guard maneuvering

## Initial Orbit Environment Identifiers

+ ID `i1`: Lady and Guard are in the same cicrular orbit with Guard ~600m prograde of the Lady. The Bandit is in an elliptical orbit with an upcoming tangential conjunction at apoapsis with the Lady spacecraft
+ ID `i2`: Lady, Bandit, and Guard are all in the same circular orbit. Guard is in between Lady and Bandit with Guard ~600m retrograde of lady and Bandit ~2000m retrograde of Guard

## Evaluation Criteria 

The performance of each solution technique in each environment/scenario can measured based upon

+ Closest approach between Bandit and Lady
+ Elapsed time to closest approach between Bandit and Lady
+ Closest approach between Bandit and Guard
+ Closest approach between Lady and Guard
+ Minimum relative position-velocity product between Bandit and Lady
+ Lady fuel usage
+ Bandint fuel usage
+ Guard fuel usage
+ Approximate delta-V needed for Bandit to rendezvous with Lady based on a single Lambert Targeting solution at final time step assuming no further maneuvering by Lady.

## Single-Value Scoring Function (v1) 

> :warning: **Attention**
> __Lower-valued__ scores indicate higher performance

In order to rank the performance of a different control algorithms, we provide a single-value scoring function that is a sum of the closest lady-bandit approach distance squared and the inverse of the closest bandit-guard approach (plus a positive offset to avoid divide by zero). Therfore high-values of dm_lb are dispro

dm_lb: closest approach distance between lady and bandit [m]
dm_bg: closest approach distance between bandit and guard [m]
a = 1e6, scaling factor such that apprx 100m of dm_lb is as good as 100m of dm_bg is bad
b = 0.1, positive offset that makes the highest possible dm_bg penalty roughly equal to the highest possible dm_lb penalty

score = dm_lb^2 + a/(dm_bg+b)

## Environment Termination Conditions 

Each scenario/environment may have it's own unique termination conditions which will be some combination of:

+ Bandit zero fuel
+ Approach distance threshold between Bandit and Lady
+ Approach distance threshold between Bandit and Guard
+ Elapsed time (max 10 minutes for each scenario)


