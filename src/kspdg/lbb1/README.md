# Lady-Bandit-Bodyguard Group 1 Environments (Target Guarding)

A collection of 1-v-2 target guarding environments with shared parameters/characteristics.
The terminology "Lady-Bandit-Bodyguard (lbb)" comes from Rusnak's 2005 paper which defined a similar differential game but not in the orbital domain:
> Rusnak, Ilan. "The lady, the bandits and the body guards–a two team dynamic game." IFAC Proceedings Volumes 38, no. 1 (2005): 441-446.

Distinct environments are compositions of Lady-Bodyguard policies and initial orbital configurations. Therefore, for example, an environment titled `lbb1_lb0_i1` is a distinct environment from `lbb1_lb0_i2`, although they will have the same behavior policies for the Lady-Bodyguard team but different initial orbital configurations. See [Environment Naming Scheme](#environment-naming-scheme)

##  Environment Naming Scheme

This README documents/decodes the somewhat cryptic naming scheme used to uniquely identify environments. 
+ The naming scheme follows a patter of: `{GroupID}_lb{LadyBodyguardPolicyID}_i{InitCondID}_v{VersionNum}`
+ The primary reason for these cryptic naming schemes is brevity; it becomes ungainly and impractical to have files named things like: `LadyBanditBodyguardEnvGroup1_LadyNashPolicy3_BodyguardHeuristicPolicy4_EllipticalCircularInitOrbits12_v5`
+ *GroupID* identifies a collection of environments with shared properties. 
    + Example: `lbb1` is this group's name which represents "Lady-Bandit-Bodyguard Environments Group 1"
    + The group name can be cryptic, verbose, or completely non-descriptive of the group of environments' properties; but it does need to be _unique_ from all other group ID's
    + The sub-group naming scheme can vary between environment groups. We use the `_lb{LadyBodyguardPolicyID}_i{InitCondID}` naming scheme for this particular environment group but it may be meaningless in another group
+ *LadyBodyguardPoliyID* identifies the policy of the Lady-Bodyguard team within the environment
    + Example: `lbnash1` could represent an lady-bodyguard team that uses Nash-equilibrium evasion policies, or we could use `lb5` to represent the same thing. 
    + Similar to GroupID, the exact name is somewhat arbitrary, it is really only important that LadyBodyguardPolicyID be _unique_ from all other Evasion ID's within the group
+ *InitCondID* identifies the initial condition (i.e. orbital parameters) of the pursuer in the environment
    + Example: `i2` could represent an initial condition where the pursuer is inclined relative to the evader but a conjunction is staged at some point in the future
    + Similar to other IDs it is most important that it is unique within the group
+ *VersionNum* is a monotonically increasing number starting from 0 used bookkeeping purposes. 
    + Any change to the group's shared parameters, evasion behiavor, and/or initial conditions that occur between version updates of the `kspdg` library should be reflected in an increment of the environment's VersionNum. 
    + This increment is necessary because any agent trained under different group params, evasion behavior, and/or initial conditions was technically trained under a different environment


## Constants Across Environments in Group

+ There is 1 "Bandit", 1 "Bodyguard", and 1 "Lady" spacecraft
+ Agent controls the Bandit spacecraft, a scripted bot(s) controls the Lady and the Bodyguard (but the bot's policy varies between environments within the group)
+ Bandit and Bodyguard have identical vehicle capabilities in each scenario, Lady vehicle may have the same or different vehicle capabilities
+ Lady initial orbit constant across all sencarios/envs; Bandit and Bodyguard initial orbits are varied across environments in the group
+ Observation and Action spaces are constant across all scenarios/envs

## Lady-Bodyguard Policy Environment Identifiers

+ ID `lb0`: Lady and Bodyguard are passive, no maneuvering
+ ID `lb1`: Lady is passive, bodyguard applies heuristic pursuit of Bandit using target-zero_vel-target relative maneuver sequence
+ ID `lb2`: Lady applies hueristic evasive maneuvers of Bandit using out-of-plane burns in order to maintian constant period of orbit, bodyguard applies heuristic pursuit of Bandit using target-zero_vel-target relative maneuver sequence
+ ID `lbnash1`: Lady is passive, Nash-equliibrium/optimal pursuit maneuvers for Bodyguard
    + _NotImplemented_
+ ID `lbnash2`: Nash-equliibrium/optimal pursuit maneuvers for Lady and Bodyguard
    + _NotImplemented_

## Initial Orbit Environment Identifiers

+ ID `i1`: Lady and Bodyguard are in the same cicrular orbit with Bodyguard ~500m prograde of the Lady. The Bandit is in an elliptical orbit with an upcoming tangential conjunction at apoapsis with the Lady spacecraft

## Evaluation Criteria: 

The performance of each solution technique in each environment/scenario can measured based upon

+ Closest approach between Bandit and Lady
+ Elapsed time to closest approach between Bandit and Lady
+ Closest approach between Bandit and Bodyguard
+ Closest approach between Lady and Bodyguard
+ Minimum relative position-velocity product between Bandit and Lady
+ Lady fuel usage
+ Bandint fuel usage
+ Bodyguard fuel usage
+ Approximate delta-V needed for Bandit to rendezvous with Lady based on a single Lambert Targeting solution at final time step assuming no further maneuvering by Lady.

## Environment Termination Conditions: 

Each scenario/environment may have it's own unique termination conditions which will be some combination of:

+ Bandit zero fuel
+ Approach distance threshold between Bandit and Lady
+ Approach distance threshold between Bandit and Bodyguard
+ Elapsed time (max 10 minutes for each scenario)


