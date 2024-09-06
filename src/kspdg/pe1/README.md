# Pursuit-Evasion Group 1 Environments

A collection of 1-v-1 pursuit-evasion environments with shared parameters/characteristics

## Environment Naming Scheme

This README documents/decodes the somewhat cryptic naming scheme used to uniquely identify environments. 

+ The naming scheme follows a patter of: `{GroupID}_e{EvasionID}_i{InitCondID}_v{VersionNum}`
+ The primary reason for these cryptic naming schemes is brevity; it becomes ungainly and impractical to have files named things like: `PursuitEvadeEnvGroup1_EvaderPolicy3_EllipticalCircularInitOrbits12_v5`
+ *GroupID* identifies a collection of environments with shared properties. 
    + Example: `pe1` is this group's name which represents "Pursuit-Evasion Environments Group 1"
    + The group name can be cryptic, verbose, or completely non-descriptive of the group of environments' properties; but it does need to be _unique_ from all other group ID's
    + The sub-group naming scheme can vary between environment groups. We use the `_e{EvasionID}_i{InitCondID}` naming scheme for this particular environment group but it may be meaningless in another group
+ *EvasionID* identifies the policy of the evader agent within the environment
    + Example: `eNash` could represent an evader that uses a Nash-equilibrium evasion policy, or we could use `e4` to represent the same thing
    + Similar to GroupID, it is really only important that EvasionID be _unique_ from all other Evasion ID's within the group
+ *InitCondID* identifies the initial condition (i.e. orbital parameters) of the pursuer in the environment
    + Example: `i2` could represent an initial condition where the pursuer is inclined relative to the evader but a conjunction is staged at some point in the future
    + Similar to other IDs it is most important that it is unique within the group
+ *VersionNum* is a monotonically increasing number starting from 0 used bookkeeping purposes. 
    + Any change to the group's shared parameters, evasion behiavor, and/or initial conditions that occur between version updates of the `kspdg` library should be reflected in an increment of the environment's VersionNum. 
    + This increment is necessary because any agent trained under different group params, evasion behavior, and/or initial conditions was technically trained under a different environment

## Constants Across Environments in Group

+ Agent controls pursuer, a scripted bot controls evader (but the bot's policy varies between envs)
+ Pursuer and evader have identical vehicle params in each scenario and across all scenarios
+ Evader initial orbit constant across all sencarios/envs; Pursuer initial orbit is varied
+ Observation and Action spaces are constant across all scenarios/envs

## Observation and Action Space:

See [`get_observation()`](https://github.com/mit-ll/spacegym-kspdg/blob/main/src/kspdg/pe1/pe1_base.py) for detailed description of observation vector.

See `Group1BaseEnv.vessel_step()` in [`base_envs.py`](https://github.com/mit-ll/spacegym-kspdg/blob/main/src/kspdg/base_envs.py) for detailed description of action dictionary


## Evader Policy Identifiers

+ ID `e1`: Passive, no maneuvering (pure rendezous)
+ ID `e2`: Randomized evasion maneuvers
+ ID `e3`: Heuristic/mission-centric evasion maneuvers
+ ID `e4`: Low-thrust orbit raise. Evader applies a constant, low-thrust, velocity-tangent burn regardless of pursuer

## Initial Orbit Environment Identifiers

+ ID `i1`: Elliptical and Circular orbits with tangential conjunction
+ ID `i2`: Relatively inclined initial orbits with unactuated conjunction
+ ID `i3`: ~3km separation (< 1 deg mean anomaly difference) in same circular orbit
+ ID `i4`: Inclined, out-of-phase, and unsyncronized periods 

## Evaluation Criteria

The performance of each solution technique in each environment/scenario can measured based upon

+ Closest approach
+ Elapsed time to closest approach
+ Minimum relative position-velocity product
+ Pursuer fuel usage
+ Evader fuel usage
+ Approximate delta-V needed for Pursuer to rendezvous with Evader based on a single Lambert Targeting solution at final time step assuming no further maneuvering by Evader.

## Single-Value Scoring Function (v1) 

> :warning: **Attention**
> __Lower-valued__ scores indicate higher performance

In order to rank the performance of a different control algorithms, we provide a single-value scoring function that is an exponential polynomial of the above evaluation metrics.
Each component is scaled (s) and then given an importance exponent (w)

+ `d`: Closest approach distance [m]:                s = 0.1     w = 2.0
+ `v`: Relative speed at closes approach [m/s]:      s = 0.5     w = 1.5
+ `f`: Pursuer fuel usage at closes approach [kg]:   s = 0.1     w = 1.25
+ `t`: Elapsed time at closest approach [s]:         s = 0.01    w = 1.0

Therefore the scoring function is: `(0.1*d)^2.0 + (0.5*v)^1.5 + (0.1*f)^1.25 + (0.01*t)^1.0`

__Note on exponential polynomial:__ An exponential polynomial---instead of a simple weighted sum---was chosen to disproportionally weight different metrics in different regimes. For example, the closest approach distances is the primary driver of the score (thus the exponent of 2.0). We don't want a direct trade-off between closest-approach and relative speed at closest approach; we want particpants to focus on minimizing closest approach _AND THEN_ prioritizes minizing relative speed at closest approach _AND THEN_ prioritizes minimizing fuel usage. 

## Environment Termination Conditions

Each scenario may have it's own unique termination conditions which will be some combination of:

+ Pursuer zero fuel
+ Approach distance threshold
+ Elapsed time (max 10 minutes for each scenario)

