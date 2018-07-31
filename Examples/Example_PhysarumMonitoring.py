##
#
#  File: Example_PhysarumMonitoring.py
#  Description: Repeatedly captures images of slime mold plates arranged in a 3 by 3 configuration for high-throughput time lapse movies and offline analysis.
#  Inter-image interval and total duration are variable.

import sys
import os
import time

import cv2

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import robotutil
import commonYeastTasks as cyt
import ExampleYeastWorkspace


#### BEGIN PGM ####
robot = robotutil.MAPLE("MAPLE.cfg")

imgInterval = 60		# in seconds
tTotal = 0
picnum = 0
while tTotal < 60*60*10:		# in seconds
	tStart = time.time()
	for numArena in range(0,9):
                # TODO should whether light is required to take frames just be
                # set in confiq, and change how robot methods work? or are there
                # cases where some actions will require light?
		curImg = cyt.imgArena(robot, ExampleYeastWorkspace.YeastWorkspace['yeastArena3x3'], numArena, light=0)
		time.sleep(0.1)
		cv2.imwrite(str(numArena)+ '_' + str(picnum) + '.png', curImg)
	tEnd = time.time()
	tDelta = tEnd - tStart
	tWait = imgInterval - tDelta
	tTotal = tTotal + tDelta + tWait
	picnum = picnum + 1
	print 'Waiting', tWait, 'seconds.'
	time.sleep(tWait)
