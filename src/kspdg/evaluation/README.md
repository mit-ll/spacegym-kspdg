# Agent-Environment Evaluation

Here we provide tools for evaluating user-defined agents in various KSPDG environments, and logging the results in a way that they can be authenticated.

Here is an example of how to perform an evaluation:
1. Start KSP game application.
2. Select `Start Game` > `Play Missions` > `Community Created` > `pe1_i3` > `Continue`
In kRPC dialog box click `Add server`. Select `Show advanced settings` and select `Auto-accept new clients`. Then select `Start Server`
```bash
conda activate kspdg # while it is not strictly necessary to use conda environments, it is encouraged for development and debugging purpose
cd spacegym-kspdg/evaluation # navigate to this evaluation/ directory for relative paths to work correctly
python evaluate.pyc configs/example_eval_cfg.yaml
```
