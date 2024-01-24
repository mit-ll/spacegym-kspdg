# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# library-wide constant parameters

from types import SimpleNamespace

G0 = 9.80665 # standard gravity [m/s/s]

# parameters of celestial body Kerbin
KERBIN = SimpleNamespace()
KERBIN.RADIUS = 6.0e5 # [m] https://wiki.kerbalspaceprogram.com/wiki/Kerbin
KERBIN.MU = 3.5316e12 # [m^3/s^2] https://wiki.kerbalspaceprogram.com/wiki/Kerbin

