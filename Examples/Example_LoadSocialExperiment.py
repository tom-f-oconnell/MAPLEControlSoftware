##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##
#
#  File: Example_LoadSocialExperiment.py
#  Description: Example routine of loading and unloading 96 flies (1 full FlyPlate) for one-day social experiments.
#  Workspace dimenions are defined in ExampleSocialWorkspace file.
#  Example routine includes moving flies from FlyPlate wells into social arenas.
#  MAPLE waits for the social experiment to complete before unloading flies back into FlyPlate wells. Manually press any key when the arena tray is placed back in the workspace.

import sys
import os

import cv2

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import robotutil
import commonFlyTasks as cft
import ExampleSocialWorkspace


#### Initialize MAPLE ####
robot = robotutil.MAPLE("MAPLE.cfg")

## Starts main example transportation routines
# Only transport flies into the left compartment first.
for well, arena in zip(range(48), range(48)):
    # withdraws fly from individual well n
    cft.homeWithdraw(robot,
        ExampleSocialWorkspace.SocialWorkspace['plate1'], well,
        refptX='N', refptY='N', carefulZ=7, dislodgeZ=25, vacBurst=1,
        homeZ=45)

    cft.arenaDeposit(robot,
        ExampleSocialWorkspace.SocialWorkspace['socialArena'], arena,
        arenaRad=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Radii,
        turnZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].POIz,
        airPos=50,
        airZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Vacz,
        closePos=0)

# Repeat for the right compartment.
for well, arena in zip(range(48,96), range(48)):
    # withdraws fly from individual well n
    cft.homeWithdraw(robot,
        ExampleSocialWorkspace.SocialWorkspace['plate1'], well,
        refptX='N', refptY='N', carefulZ=7, dislodgeZ=25, vacBurst=1,
        homeZ=45)

    cft.arenaDeposit(robot,
        ExampleSocialWorkspace.SocialWorkspace['socialArena'], arena,
        arenaRad=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Radii,
        turnZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].POIz,
        # THIS SEEMS TO BE THE ONLY DIFFERENT LINE. could simplify...
        airPos=300,
        airZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Vacz,
        closePos=0)

# MAPLE waits for any keypress before unloading flies back into FlyPlate wells.
robot.home()
cv2.waitKey(0)

# Only transport flies into the left compartment first.
for well, arena in zip(range(48), range(48)):
    cft.arenaWithdraw(robot,
        ExampleSocialWorkspace.SocialWorkspace['socialArena'], arena,
        arenaRad=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Radii,
        turnZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].POIz,
        vacPos=50,
        vacZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Vacz,
        closePos=0, vacstrategy=2, vacBurst=1, imgshow=0)

    cft.homeDeposit(robot, ExampleSocialWorkspace.SocialWorkspace['plate1'],
        well, refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=44)

# Repeat for the right compartment.
for well, arena in zip(range(48,96), range(48)):
    cft.arenaWithdraw(robot,
        ExampleSocialWorkspace.SocialWorkspace['socialArena'], arena,
        arenaRad=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Radii,
        turnZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].POIz,
        # LIKEWISE, THIS SEEMS TO BE THE ONLY DIFFERENT LINE HERE
        vacPos=300,
        vacZ=ExampleSocialWorkspace.SocialWorkspace['socialArena'].Vacz,
        closePos=0, vacstrategy=2, vacBurst=1, imgshow=0)

    cft.homeDeposit(robot, ExampleSocialWorkspace.SocialWorkspace['plate1'],
        well, refptX='N', refptY='N', carefulZ=9, vacBurst=1, homeZ=44)

# Home MAPLE after all flies are transported.
robot.home()
