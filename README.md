# KSPDG: Kerbal Space Program Differential Games

~~> TODO: *Add emblem/logo upon completion*

## Overview

This library provides a suite of differential game (DG) environments built within the [Kerbal Space Program (KSP)](https://www.kerbalspaceprogram.com/) game engine.

The KSP differential game environments are implemented using the [OpenAI Gym](https://www.gymlibrary.ml/) and [PettingZoo](https://www.pettingzoo.ml/) standards. Non-GUI control of the KSP game engine is enabled by [kRPC](https://krpc.github.io/krpc/)

### Design Principles

*TODO*

------------

## Citation

~~> TODO: *Zenodo DOI to be added upon making repo public*

~~> TODO: *SpaceGym paper citation to be added upon publication at IEEE AeroConf 2023*

------------

## Disclaimer

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

© 2022 Massachusetts Institute of Technology.

Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

SPDX-License-Identifier: MIT

The software/firmware is provided to you on an As-Is basis

Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.

------------

## Installation

The installation process includes several components:
+ [Kerbal Space Program](https://www.kerbalspaceprogram.com/)
+ [Making History Expansion](https://store.privatedivision.com/game/kerbal-space-program-making-history-expansion-official-pc)
+ [Mission Save Files](https://github.com/mit-ll/spacegym-kspdg/tree/master/ksp_saves/missions)
+ [kRPC Server](https://krpc.github.io/krpc/getting-started.html#the-server-plugin)
+ [PhysicsRangeExtender](https://github.com/jrodrigv/PhysicsRangeExtender)
+ [`kspdg` python package](https://github.com/mit-ll/spacegym-kspdg/tree/master/src/kspdg)
+ [Luna Multiplayer](http://lunamultiplayer.com/) (optional/future work)

> :warning: **Note**
> These instructions have been written and verified on a macOS. Other operating systems should be similar with deviations on file and directory names


### Install KSP & Making History Expansion

1. Purchase and Download Kerbal Space Program and Making History expansion: https://store.privatedivision.com/game/buy-kerbal-space-program-ksp
    + Make sure to purchase _DRM Free Private Division_ as the platform
    + Download KSP v1.12.3 "On Final Approach" Portable (.zip)
    + Download Making History v1.12.1 
2. Unzip `ksp-osx-1.12.3.zip` to desired location; for simplicity, all instructions assume the unzipped KSP folder is placed on the Desktop
3. Attempt to open the KSP game executable/app (e.g. `KSP.app` on Mac)

> :warning: **Troubleshooting**
> + On a Mac, you might run into an error where you can't open the KSP app because the developer can't be verified.
> + To change these preferences on your Mac, choose Apple menu > System Preferences, click Security & Privacy
> + Under the General tab there should be a notification saying something like "KSP.app was blocked ..." if you've attempted to open the KSP app. Click the "Open Anyway" button next to the notification

> :warning: **Troubleshooting**
> + On a Mac, after enabling KSP to be opened in Security and Privacy, you may encounter a bug where [the game loading screen stalls indefinitely](ttps://forum.kerbalspaceprogram.com/index.php?/topic/151986-just-purchased-and-stuck-on-loading-screen-mac-os/)
> + The workaround is to move the `KSP.app` icon onto the desktop and then back into the `KSP_osx` directory. For some reason bash commands didn't seem to work to fix this bug. Had to manually open Finder, drag the KSP.app icon onto the Desktop (even tried to open the app on the desktop, but that shouldn't work because it's not in the correct file structure), and then drag it back into the KSP_osx/ directory

4. Unzip `KSP-Making_History_Expansion-en-us-mac-1.12.1.zip`
5. Follow the instructions in the `Instructions-xxx-xx-xx.txt` file located in the unzipped Making History Expansion directory. 

> Instructions:
> 1. Copy the two other files located in this folder (.command and .zip) to the folder where the KSP app is located
> 2. Once you have copied the files, double click the .command file
> Thats it! Enjoy the Making History Expansion of Kerbal Space Program!

6. Test installation by opening KSP (e.g. KSP.app on Mac). When main screen has loaded, select `Start Game` and you should see options for `Play Missions` and `Mission Builder` to confirm that the Making History Expansion was successfully installed

### Install KSPDG Mission Save Files

For each differential game environment there are associated mission files created using the "Making History" expansion pack that serves to populate the KSP game engine with the necessary spacecraft in the appropriate orbits. There is also a number of mission save files for in-game software development testing purposes.

The save files are located in this repo under `ksp_files/saves/missions` and `ksp_files/Missions`; both sets are necessary to populate the differential game environments. These mission save files must be downloaded and manaully installed into the correct directory of the KSP game installation.

Copy the contents of `ksp_files/saves/missions/` and `ksp_files/Missions` directory into your local installation of KSP. For example, on a Mac with this repo and KSP's install directories on the desktop this would look like:
```bash
cd kspdg
cp -r ksp_files/saves/missions/. ~/Desktop/KSP_osx/saves/missions
cp -r ksp_files/Missions/. ~/Desktop/KSP_osx/Missions
```

### Install kRPC Server

kRPC is what allows external scripts and processes (such as python programs) to send commands to and control the KSP game engine
1. Download `krpc-0.4.8.zip` from [GitHub link on the kRPC Getting Started Page](https://krpc.github.io/krpc/getting-started.html#installation)
2. Unzip `krpc-0.4.8/` folder to `~/Desktop/krpc-0.4.8/`
3. Create a new directory in KSP's `GameData` directory and move all of the krpc contents there
```bash
mkdir ~/Desktop/KSP_osx/GameData/kRPC
mv ~/Desktop/krpc-0.4.8/* ~/Desktop/KSP_osx/GameData/kRPC/
```

### Install PhysicsRangeExtender

By default in KSP, high-fidelity physical simulation of spacecraft is only performed for spacecraft very near to the active spacecraft (e.g. only around [2km](https://steamcommunity.com/app/220200/discussions/0/3044985412465032716/)). [PhysicsRangeExtender](https://github.com/jrodrigv/PhysicsRangeExtender) allows for better simulation (e.g. thusting maneuvers) of more distant spacecraft.
1. Clone PhysicsRangeExtender (assumed to be cloned to Desktop in these instructions, but you can put it wherever you like since you will be copying things from the clone to `GameData`)
2. Copy necessary subfolder from PhysicsRangeExtender to your KSP install's `GameData` folder
```bash
# clone PhysicsRange Extender locally
cd ~/Desktop
git clone git@github.com:jrodrigv/PhysicsRangeExtender.git

# copy the necessary game data for the mod into your KSP install
mkdir ~/Desktop/KSP_osx/GameData/PhysicsRangeExtender
cp -r ~/Desktop/PhysicsRangeExtender/PhysicsRangeExtender/Distribution/GameData/PhysicsRangeExtender/ ~/Desktop/KSP_osx/GameData/PhysicsRangeExtender/
```

### Install `kspdg`

Clone this repository locally on your machine

```bash
git clone git@github.com:mit-ll/spacegym-kspdg.git
```

To install this package, run:

```bash
cd kspdg
pip install -e .
pip install krpc    # must be installed after kspdg installattion due to setuptools compatability
```

For development of this package, we recommend using the conda environment defined in `environment.yml`. To create and activate this environment, run:

```bash
cd kspdg
conda env create -f environment.yml
conda activate kspdg
pip install krpc    # must be installed after conda env creation for setuptools compatability
``` 

> :warning: **Troubleshooting**
> + Note that the `kspdg` library depends upon [poliastor](https://docs.poliastro.space/en/stable/), which in turn depends upon [astropy](https://www.astropy.org/), which in turn depends upon [pyerfa](https://github.com/liberfa/pyerfa)
> + __FOR MAC USERS with M1 chipsets:__ as of this writing, [pyerfa has not fully supported M1's arm64 architecture](https://github.com/liberfa/pyerfa/issues/83)
> + This can lead to errors running `kspdg` such as
> ```
> (mach-o file, but is an incompatible architecture (have 'x86_64', need 'arm64e'))
> ```
> + The workaround for Mac users with M1 chipsets is described [here](https://github.com/liberfa/pyerfa/issues/83#issuecomment-1255333177). For Python 3.9, the workaround entails cloning pyerfa locally, checking out a specific version, and installing in the conda environment
> ```bash
> # get pyerfa source code and switch to specific release of pyerfa
> git clone --recursive https://github.com/liberfa/pyerfa/
> cd pyerfa
> git fetch origin
> git checkout v2.0.0.1
> 
> # install specific version of pyerfa in conda environment
> conda activate kspdg
> pip install .
> ```

### Install Luna Multiplayer (LMP)

_Future Work_

### Verify Installation

__NOTE:__ Because the KSPDG environments are inexorably linked to the KSP game engine, many of the library's unit/integration test can only be run when a particular game mission file has been loaded and running. This means that verifying installation and testing during code development is a bit more involved than just a single `pytest` call

__Serverless Tests:__ Quick test to run without KSP game engine running nor kRPC server connection

```bash
cd kspdg
conda activate kspdg
pytest tests/serverless_tests/
```

__KSP In-Game Tests:__ These tests require the KSP game engine to be running, the test-specific mission to be loaded, and a connection to the kRPC server

1. Start KSP game application. 
2. Select `Start Game` > `Play Missions` > `Community Created` > `pe1_i3` > `Continue`
3. In kRPC dialog box click `Add server`. Select `Show advanced settings` and select `Auto-accept new clients`. Then select `Start Server`
4. In a bash terminal:
```bash
cd kspdg
conda activate kspdg
pytest tests/ksp_ingame_tests/test_pe1_e1_i3.py

# for additional tests, load a different mission in KSP: 
# ESC > Quit to Main Menu > Exit to Main Menu > Play Missions > 20220516_PursuitEvade > Continue
pytest tests/ksp_ingame_tests/test_pursuit_v20220516.py
```
5. You should see the KSP game reset and focus on a vehicle that then performs several oreintation and propulsive maneuvers. The pytest command should then indicate the number of passed tests.


> :warning: **Troubleshooting**
> If you are using a Mac with an arm64 architecture (e.g. M1 chipset) and recieve an error like `(mach-o file, but is an incompatible architecture (have 'x86_64', need 'arm64e'))`, please refer to instructions in the [kspdg library installation section](#install-kspdg) about installing `pyerfa` from source.


------------

## References

Throughout the documentation and code comments we refer to aerospace literature such as "Vallado Chp 3" for brevity. However this assumes anyone reading this code knows that "Vallado" is short hand for David Vallado's "Fundamentals of Astrodynamics and Applications", which is an unfair assumption to make. Here we list some of our short-hand references

+ Vallado, David A. Fundamentals of astrodynamics and applications. Vol. 12. Springer Science & Business Media, 2001.
    + short hands: "Vallado"
    + There are multiple editions with slightly different section layouts. We try to specify which edition when referencing specific figures/sections but mostly pulling from 3rd or 4th edition
+ Bate, R. R., Mueller, D. D., & White, J. E. (1971). Fundamentals of astrodynamics. Fundamentals of astrodynamics.
    + short hands: "BMW", "Bate, Mueller, White"

------------

## Code Notation

Coordinate transforms are used throughout this code base. Without a rigorous notation it can become very difficult to determine the meaning of any particular variable. for example and ambiguous variable name like `pursuer_pos` seems to imply the position of a "pursuer" spacecraft, but position relative to what? Relative to a planet or relative to another spacecraft? Furthermore it says nothing of what coordinate frame that variable is expressed within. The pursuer position relative to the planet can be expressed in earth-centered inertial (ECI), perifocal, or even within another satellite's RSW frame (see Vallado, 3rd Edition, Sec 3.3 for frame descriptions). Each of these expressions lead to different numerical values for vector representations of the same physical vector

To make matters worse, kRPC---the crucial library that provides a python interface for controlling Kerbal Space Program---uses [left-handed coordinate systems](https://krpc.github.io/krpc/tutorials/reference-frames.html#introduction)!! This will lead to much confusion if not pedantically handled.

To reduce the confusion we will strive to use the following notation for vector variables: `w_x_y__z`
+ `w` is a description of the vector. For example `pos`, `vel` may be using in the `w` position to represent position and velocity vectors, respectively
+ `x_y` is read as "x with respect to y". For example, if we want a vector for the position of satellite A with respect to satellite B we would have the partial variable name: `pos_satA_satB`
+ `__z` is read as "expressed in z". We need not only describe what the physical represents (i.e. `w_x_y`), we need to also described what coordinate frame the physcial vector is being expressed within; this is the purpose of `__z`. Therefore if we want a variable for the position of satellite A with respect to satellite B expressing in the right-handed earth-centered inertial coordinates we would write: `pos_satA_satB__rheci`

Here are some abbreviations and acronyms used throughout the code
+ `lhntw` = left-handed NTW coordinate frame (y-axis parallel to velocity vector, x-axis in orbital plane along---but not necessarily parallel to---radial in direction, z-axis perpendicular to complete left-handed system) based on [kRPC's Vessel `orbital_reference_frame`](https://krpc.github.io/krpc/python/api/space-center/vessel.html#SpaceCenter.Vessel.orbital_reference_frame)
+ `rhntw` = right-handed NTW coordinate frame (y-axis parallel to velocity vector, x-axis in orbital plane along---but not necessarily parallel to---radial out direction, z-axis perpendicular to complete right-handed system) as described in Vallado 3rd ed. sec. 3.3.3
+ `rhrsw` = right-handed RSW (radial, along-track, normal) coordinate frame described in Vallado 3rd Edition, Sec 3.3.3
+ `lhrsw` = left-handed variant RSW coordinate frame (anti-radial, along-track, normal). Note that kRPC does not have a default lhrsw frame; the `orbital_reference_frame` is in fact an NTW frame
+ `rhcbci` = right-handed celestial-body-centered inertial coordinate frame. This is a right-handed version of kRPC's [Celestial Body `non_rotating_reference_frame`](https://krpc.github.io/krpc/python/api/space-center/celestial-body.html#SpaceCenter.CelestialBody.non_rotating_reference_frame). In the real-world this would be approximately equivalent to earth centered inertial (ECI)) coords (see Vallado 3rd ed, sec 3.3.2 on Geocentric Equitorial Coordinate System IJK) except in kerbal you aren't necessarily orbiting the earth and their is often no axial tilt to the celestial body
+ `lhcbci` = left-handed celestial-body-centered inertial coordinate frame based on kRPC's [Celestial Body `non_rotating_reference_frame`](https://krpc.github.io/krpc/python/api/space-center/celestial-body.html#SpaceCenter.CelestialBody.non_rotating_reference_frame)
+ `lhvbody` = left-handel vessel-centered, vessel-fixed body coordinates that align with [kRPC's `Vessel.reference_frame`](https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame). x-axis points out right side of vessel, y-axis points forward on vessel, z-axis points down. Vessel-fixed implies tha the reference frame rotates with the vessel. Note that `v` is often omitted or replaced with some identifier abbreviation of the particular vessel; e.g. `lhpbody` could be left-handed *pursuer* vessel body coords
+ `rhvbody` = right-handed vessel-centered, vessel-fixed body coordinates.  x-axis points out forward of vessel, y-axis points out right side on vessel, z-axis points down
