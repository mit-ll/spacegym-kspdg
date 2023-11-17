# Sun-Blocking Group 1 Environments

A collection of 1-v-1 pursuit-evasion environments where the objective is to arive a co-linear point between the evader and the sun.

For reference, see: O. Jansson, M. Harris, and D. Geller, “A parallelizable reachable set method for pursuit-evasion games using interior-point methods,” in 2021 IEEE Aerospace Con- ference (50100). IEEE, 2021, pp. 1–9.

Note: These environments are actually direct extensions of the pursuit-evasion group 1 envrionemnts (pe1) with a modified objective and observations. See `pe1/README.md` for additional information on nomenclature

## Environment Naming Scheme

This README documents/decodes the somewhat cryptic naming scheme used to uniquely identify environments. 

+ The naming scheme follows a patter of: `{GroupID}_e{EvasionID}_i{InitCondID}_v{VersionNum}`
+ The primary reason for these cryptic naming schemes is brevity; it becomes ungainly and impractical to have files named things like: `PursuitEvadeEnvGroup1_EvaderPolicy3_EllipticalCircularInitOrbits12_v5`
+ *GroupID* identifies a collection of environments with shared properties. 
    + Example: `sb1` is this group's name which represents "Sun Blocking Environments Group 1"
    + The group name can be cryptic, verbose, or completely non-descriptive of the group of environments' properties; but it does need to be _unique_ from all other group ID's
    + The sub-group naming scheme can vary between environment groups. We use the `_e{EvasionID}_i{InitCondID}` naming scheme for this particular environment group but it may be meaningless in another group
+ *EvasionID* identifies the policy of the evader agent (agent being blocked from sun) within the environment
    + Example: `ePassive` could represent an evader that uses a passive policy, or we could use `e1` to represent the same thing
    + Similar to GroupID, it is really only important that EvasionID be _unique_ from all other Evasion ID's within the group
+ *InitCondID* identifies the initial condition (i.e. orbital parameters) of the pursuer in the environment
    + Example: `i5` could represent an initial condition where the pursuer (sun blocker) starts in the optimal blocking position relative to the evader who is in a circular orbit
    + Similar to other IDs it is most important that it is unique within the group
+ *VersionNum* is a monotonically increasing number starting from 0 used bookkeeping purposes. 
    + Any change to the group's shared parameters, evasion behiavor, and/or initial conditions that occur between version updates of the `kspdg` library should be reflected in an increment of the environment's VersionNum. 
    + This increment is necessary because any agent trained under different group params, evasion behavior, and/or initial conditions was technically trained under a different environment

## Constants Across Environments in Group

+ Agent controls Pursuer (Sun Blocker), a scripted bot controls Evader---but the bot's policy may vary between envs
+ Pursuer (Blocker) and evader have identical vehicle params in each scenario and across all scenarios
+ Evader initial orbit constant across all sencarios/envs; Pursuer (Blocker) initial orbit is varied
+ Observation and Action spaces are constant across all scenarios/envs

## Observation and Action Space:

See [`get_observation()`](https://github.com/mit-ll/spacegym-kspdg/blob/main/src/kspdg/sb1/sb1_base.py) for detailed description of observation vector.

See `Group1BaseEnv.vessel_step()` in [`base_envs.py`](https://github.com/mit-ll/spacegym-kspdg/blob/main/src/kspdg/base_envs.py) for detailed description of the action dictionary 

## Evader Policy Identifiers

+ ID `e1`: Passive, no maneuvering (pure rendezous)

## Initial Orbit Environment Identifiers

+ ID `i1`: Elliptical and Circular orbits with tangential conjunction
+ ID `i2`: Relatively inclined initial orbits with unactuated conjunction
+ ID `i3`: ~3km separation (< 1 deg mean anomaly difference) in same circular orbit
+ ID `i4`: Inclined, out-of-phase, and unsyncronized periods 
+ ID `i5`: Elliptical and circular orbits with puruser intially co-linear between sun and evader

## Evaluation Criteria

The performance of each solution technique in each environment/scenario can measured based upon

+ Cumulative sun-blocking reward
    + reward function varies with pursuer-evader distance and evader-pursuer-sun angle. This function puts a maximum at theta=pi (pursuer directly between sun and evader) and r=r_d, where r_d is the desired range between evader and pursuer. It puts a minimum on the opposite side where theta=pi (evader directly between sun and pursuer) and r=r_d. Way from these points, reward drops to zero, which I think makes sense for the scenario
    + See image for general shape, scale is incorrect, though.
    ![20230224_sun_blocking_reward_viz.gif](../../../docs/20230224_sun_blocking_reward_viz.gif)
+ Maximum sun-blocking reward
+ Minimum sun-blocking reward
+ Pursuer (Blocker) fuel usage
+ Evader fuel usage

## Single-Value Scoring Function (v1) 

> :warning: **Attention**
> __Higher-valued__ scores indicate higher performance

In order to rank the performance of a different control algorithms, we provide a single-value scoring function that is the time-integrated cumulative reward function, integrated using a trapezoid approximation

## Environment Termination Conditions

Each scenario may have it's own unique termination conditions which will be some combination of:

+ Pursuer zero fuel
+ Approach distance threshold
+ Elapsed time (max 10 minutes for each scenario)