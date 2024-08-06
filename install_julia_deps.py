# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# install necessary julia dependencies via juliacall and juliapkg

import juliapkg     # this should have been installed during the juliacall install within conda env

# iLQGames.jl: Need URL since ilqgames does not appear to be registered
juliapkg.add("iLQGames", "ae812560-bd7e-11e9-3bd3-b502f7cff3d3", url="https://github.com/lassepe/iLQGames.jl.git")

# you should be able to see this change reflected in ~/miniconda3/envs/kspdg/julia_env/pyjuliapkg/juliapkg.json

juliapkg.resolve()  # resolves julia packages, ie ilqgames