# Pursuit-Evasion Group 1 Environments

A collection of 1-v-1 pursuit-evasion environments with shared parameters/characteristics

+ Constants across all environments/scenarios:
    + Agent controls pursuer, a scripted bot controls evader (but the bot's policy varies between envs)
    + Pursuer and evader have identical vehicle params in each scenario and across all scenarios
    + Evader initial orbit constant across all sencarios/envs; Pursuer initial orbit is varied
    + Observation and Action spaces are constant across all scenarios/envs
+ Variable Evader Policies:
    + ID e1: Passive, no maneuvering (pure rendezous)
    + ID e2: Randomized evasion maneuvers
    + ID e3: Heuristic/mission-centric evasion maneuvers
    + ID e4: Low-thrust orbit raise. Evader applies a constant, low-thrust, velocity-tangent burn regardless of pursuer
    + ID e5: Nash/optimal evasion maneuvers
        + _NotImplemented_
+ Variable Initial Conditions:
    + ID i1: Elliptical and Circular orbits with tangential conjunction
    + ID i2: Relatively inclined initial orbits with unactuated conjunction
    + ID i3: ~3km separation (< 1 deg mean anomaly difference) in same circular orbit
    + ID i4: Inclined, out-of-phase, and unsyncronized periods 
+ Evaluation Criteria: The performance of each solution technique in each environment/scenario can measured based upon
    + Closest approach
    + Elapsed time to closest approach
    + Minimum relative position-velocity product
    + Pursuer fuel usage
    + Evader fuel usage
    + Final delta-V needed for evader to return to original orbit based on a single Lambert Targeting solution
+ Scenario Termination Conditions: Each scenario may have it's own unique termination conditions which will be some combination of:
    + Pursuer zero fuel
    + Approach distance threshold
    + Elapsed time (max 10 minutes for each scenario)
+ Environment naming scheme:
    + `{GroupID}_e{EvasionID}_i{InitCondID}_v{VersionNum}`
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
