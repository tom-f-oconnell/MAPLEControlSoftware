##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##
#
#  File: robotutil.py
#  Description: Contains classes and functions used to control
#  the FlySorter automated experiment platform (project name MAPLE).
#  High-level commands can be called in primary experimental scripts with relevant coordinates.

import time
import math
import ConfigParser
import urllib2
import importlib
import pyclbr
import atexit

import numpy as np
import cv2

import flysorterSerial
import cameras


class CamDisabledError(IOError):
    pass

class MAPLE:
    """Class for fly manipulation robot."""

    smoothiePort = ""
    dispenserPort = ""

    # These variables should be kept up-to-date by functions that change them
    currentPosition = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    currentRotation = np.array([0.0, 0.0])

    # Initialize these variables to zero -- they should be read in by readConfig
    Z0Offset = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    Z2Offset = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    maxExtents = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    OutputDir = ""

    travelSpeed = 5000
    acceleration = 200

    # Configuration defaults
    configDefaults = {'WorkspaceXSize': '1000',
                      'WorkspaceYSize': '300',
                      'Z1FloorHeight': '40.0',
                      'MaxZ0Depth': '68',
                      'MaxZ1Depth': '50',
                      'MaxZ2Depth': '55',
                      'OutputDir': 'Photos',
                      'Z0OffsetX': '-40',
                      'Z0OffsetY': '0',
                      'Z0OffsetZ': '23',
                      'Z2OffsetX': '40',
                      'Z2OffsetY': '0',
                      'Z2OffsetZ': '8',
                      'HFOV': '14.5',
                      'VFOV': '11.25',
                      'StatusURL': '',
                      'camera_class': 'cameras.PyICIC_Camera',
                      'camera_enabled': True
                      }

    def __init__(self, robotConfigFile, cam_class=None):
        print "Reading config file...",
        self.config = ConfigParser.RawConfigParser(self.configDefaults)
        self.readConfig(robotConfigFile)
        print "done."

        print "Initializing serial connections:"
        self.smoothie = None

        # Search serial ports, look for motor control board and smoothie
        portList = flysorterSerial.availablePorts()
        print "Port list:", portList
        for portDesc in portList:
            tempPort = flysorterSerial.serialDevice(portDesc, 115200)
            if tempPort.sendCmdGetReply("version\n").startswith("Build version"):
                print "Port:", portDesc, "is smoothie."
                self.smoothiePort = portDesc
                self.smoothie = tempPort
                portList.remove(portDesc)
                continue
            tempPort.close()

        if self.smoothie is None:
            raise IOError(
                'Serial initialization failed. Smoothie board not found.')

        if self.cam_enabled:
            def print_builtin_cameras():
                cams = pyclbr.readmodule('cameras')
                cams.pop('Camera')
                cams.pop('CameraNotFoundError')
                cams.pop('NoFrameError')
                print '\n\nAvailable cameras:'
                for c in cams:
                    print 'cameras.{}'.format(c)

            def class2str(cls):
                return '{}.{}'.format(cls.__module__, cls.__name__)

            print "Initializing camera...",
            if cam_class is None:
                parts = self.full_cam_class_name.rsplit('.', 1)
                try:
                    cam_module = importlib.import_module(parts[0])
                    cam_class = getattr(cam_module, parts[1])

                except (ImportError, AttributeError) as e:
                    print_builtin_cameras()
                    print ('\nSet camera_class to one of them in your ' +
                        'MAPLE.cfg file.\n')
                    self.smoothie.close()
                    raise

            if not issubclass(cam_class, cameras.Camera):
                print_builtin_cameras()
                self.smoothie.close()
                raise ValueError('camera_class must subclass cameras.Camera' +
                    ', but class {} does not.'.format(class2str(cam_class)))

            try:
                self.cam = cam_class()
            except ImportError as e:
                print_builtin_cameras()
                print ('\nSet camera_class to one of them in your ' +
                    'MAPLE.cfg file.\n')
                print 'Missing dependencies for camera_class={}'.format(
                    class2str(cam_class))
                self.smoothie.close()
                raise

        print "done."

        atexit.register(self.release)

        self.currentPosition = self.getCurrentPosition()
        print "Robot initialized."

        # TODO need newline???
        self.smoothie.sendCmd("M999")
        self.flyManipAir(False)
        self.smallPartManipVac(False)
        # TODO define other two valve states as well?
        self.home()


    def readConfig(self, configFile):
        """Read in the config, and assign values to the appropriate vars
        """
        self.config.read(configFile)
        section = 'DEFAULT'
        self.OutputDir = self.config.get(section, 'OutputDir')
        self.Z1FloorHeight = float(self.config.get(section, 'Z1FloorHeight'))
        self.maxExtents = np.array( [ float(self.config.get(section, 'WorkspaceXSize')), float(self.config.get(section, 'WorkspaceYSize')),
                                      float(self.config.get(section, 'MaxZ0Depth')),
                                      float(self.config.get(section, 'MaxZ1Depth')),
                                      float(self.config.get(section, 'MaxZ2Depth')) ] )
        self.Z0Offset = np.array( [ float(self.config.get(section, 'Z0OffsetX')), float(self.config.get(section, 'Z0OffsetY')),
                                      float(self.config.get(section, 'Z0OffsetZ')), 0.0, 0.0 ] )
        self.Z2Offset = np.array( [ float(self.config.get(section, 'Z2OffsetX')), float(self.config.get(section, 'Z2OffsetY')),
                                      0.0, 0.0, float(self.config.get(section, 'Z2OffsetZ')) ] )
        self.FOV = np.array([ float(self.config.get(section, 'HFOV')), float(self.config.get(section, 'VFOV')) ])
        self.StatusURL = self.config.get(section, 'StatusURL')
        if ( self.StatusURL != ''):
            urllib2.urlopen(StatusURL + "&" + "st=1")

        self.full_cam_class_name = self.config.get(section, 'camera_class')
        self.cam_enabled = self.config.getboolean(section, 'camera_enabled')


    def release(self):
        self.light(False)
        self.flyManipVac(False)
        self.smallPartManipVac(False)
        self.flyManipAir(False)
        self.smallPartManipAir(False)

        # Turns off the stepper motors.
        self.smoothie.sendSyncCmd('M84\n')
        self.smoothie.close()

        if self.cam_enabled:
            self.cam.close()


    def home(self):
        self.smoothie.sendSyncCmd("G28\n")
        self.smoothie.sendSyncCmd("G01 F{0}\n".format(self.travelSpeed))
        self.currentPosition = np.array([0., 0., 0., 0., 0.])


    def homeZ2(self):
        """Only homes end effectors; Faster than regular home()
        """
        self.smoothie.sendSyncCmd("G28 B0\n")


    def homeZ0(self):
        self.smoothie.sendSyncCmd("G28 Z0\n")


    def captureImage(self):
        """Captures current camera image; returns as numpy array
        """
        if self.cam_enabled:
            return self.cam.get_frame()
        else:
            raise CamDisabledError('Camera needed for this function.')


    def getCurrentPosition(self):
        """Returns current position as array
        """
        # M114.2 returns string like:
        # "ok MCS: X:0.0000 Y:0.0000 Z:0.0000 A:0.0000 B:0.0000"
        while True:
            try:
                positions = self.smoothie.sendCmdGetReply("M114.2\n").split(' ')
                xPos  = float(positions[2].split(':')[1])
                break
            except:
                time.sleep(0.01)
        xPos  = float(positions[2].split(':')[1])
        yPos  = float(positions[3].split(':')[1])
        z0Pos = float(positions[4].split(':')[1])
        z1Pos = float(positions[5].split(':')[1])
        z2Pos = float(positions[6].split(':')[1])
        return np.array( [ xPos, yPos, z0Pos, z1Pos, z2Pos ] )


    def moveXY(self, pt):
        """Simple coordinate-move command
        """
        if len(pt) != 2:
            raise ValueError('incorrect coordinate string.')

        if not self.isPtInBounds((pt[0], pt[1], 0., 0., 0.)):
            raise ValueError('point out of bounds (less than zero, ' +
                'greater than maxExtents)')

        cmd = "G01 X{0[0]} Y{0[1]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.currentPosition[0] = pt[0]
        self.currentPosition[1] = pt[1]


    def moveXYSpd(self, pt, spd):
        """
        Coordinate-move command with added mandatory speed (Also updates default
        speed)
        """
        if not self.isPtInBounds((pt[0], pt[1], 0., 0., 0.)):
            raise ValueError('point out of bounds (less than zero, ' +
                'greater than maxExtents)')

        # Save current speed
        self.smoothie.sendSyncCmd("M120\n")

        # Move at commanded speed
        cmd = "G01 X{0[0]} Y{0[1]} F{1}\n".format(pt, spd)
        self.smoothie.sendSyncCmd(cmd)

        # Reset speed to old value
        self.smoothie.sendSyncCmd("M121\n")

        self.currentPosition[0] = pt[0]
        self.currentPosition[1] = pt[1]


    def moveZ(self, pt):
        """Simple Z-axis move command.
        
        First 2 inputs (X and Y axis) should be 0.
        """
        if len(pt) != 5:
            raise ValueError('incorrect coordinate string.')

        if not self.isPtInBounds(pt):
            raise ValueError('point out of bounds (less than zero, ' +
                'greater than maxExtents)')

        cmd = "G01 Z{0[2]} A{0[3]} B{0[4]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.dwell_ms(1)
        # TODO use slice notation if currentPosition supports that
        self.currentPosition[2] = pt[2]
        self.currentPosition[3] = pt[3]
        self.currentPosition[4] = pt[4]


    def moveZ0(self, position):
        """Moves the part manipulator effector.
        """
        # TODO assert position is floating point? numeric?
        cmd = 'G01 Z{}\n'.format(position)
        self.smoothie.sendSyncCmd(cmd)
        self.currentPosition[2] = position


    def moveZ1(self, position):
        """Moves the camera effector.
        """
        cmd = 'G01 A{}\n'.format(position)
        self.smoothie.sendSyncCmd(cmd)
        self.currentPosition[3] = position


    def moveZ2(self, position):
        """Moves the fly manipulator effector.
        """
        cmd = 'G01 B{}\n'.format(position)
        self.smoothie.sendSyncCmd(cmd)
        self.currentPosition[4] = position


    def moveTo(self, pt):
        """All axes and end effectors move in unison.
        """
        if len(pt) != 5:
            raise ValueError('incorrect coordinate string: {}'.format(pt))

        if not self.isPtInBounds(pt):
            raise ValueError('point out of bounds (less than zero, ' +
                'greater than maxExtents)')

        cmd = "G01 X{0[0]} Y{0[1]} Z{0[2]} A{0[3]} B{0[4]}\n".format(pt)
        self.smoothie.sendSyncCmd(cmd)
        self.dwell_ms(1)
        self.currentPosition = pt


    def moveToSpd(self, pt, spd):
        """All axes and end effectors move in unison, sets new default speed.
        """
        if len(pt) != 5:
            raise ValueError('incorrect coordinate string.')

        if not self.isPtInBounds(pt):
            raise ValueError('point out of bounds (less than zero, ' +
                'greater than maxExtents)')

        # Save current speed
        self.smoothie.sendSyncCmd("M120\n")

        cmd = "G01 X{0[0]} Y{0[1]} Z{0[2]} A{0[3]} B{0[4]} F{1}\n".format(pt, spd)
        self.smoothie.sendSyncCmd(cmd)

        self.dwell_ms(1)

        # Reset speed to old value
        self.smoothie.sendSyncCmd("M121\n")

        self.currentPosition = pt


    def moveRel(self, pt):
        """Moves robot relative to current position, not absolute coordinates.
        """
        self.moveTo( map(sum,zip(self.currentPosition, pt)) )


    def moveXYList(self, ptList):
        """Moves robot to all coordinates in the list.

        (Needs 5 scalars per list entry)
        """
        for pt in ptList:
            self.moveXY(pt)


    def moveCirc2(self, mid, r, n=360, startpos=0, endpos=360, spd=1500, rest=100, z=53, full=True, retreatZ=10, descendZ=9):       #also lowers onto z height
        """
        Circular movement followed by lowering of fly manipulating end effector
        (Hit-detection causes Z-axis retreat)
        """
        careful = 0     # failsafe when already starts lowered
        if spd >= 2001:
            print 'Adjusting speed to safe level 2000 ...'  # Prevents calibration errors due to post-motion-induced positional changes.
            spd = 2000
        while startpos > 360:
            startpos = startpos - 360
            print 'Startpos corrected to:', startpos
        while endpos > 360:
            endpos = endpos - 360
            print 'Endpos corrected to:', endpos
        deg = np.ones((n+1,2))
        for x in range(0,len(deg)):
            deg[x,:] = (math.sin(2*math.pi/n*x)*r)+mid[0], (math.cos(2*math.pi/n*x)*r)+mid[1]
        if endpos - startpos < 0:
            step = -1
            offset = -2
        else:
            step = 1
            offset = 2
        for i in range(startpos, endpos+1, step):       # loop that performs the circular movement and lowering only on iteration 1
            if careful != 1:
                self.moveXYSpd(deg[i], spd)
                self.dwell_ms(rest)
                if i == startpos:
                    self.dwell_ms(10)  # just a tiny buffer before lowering
                XYpos = self.getCurrentPosition()
                if i == startpos and (XYpos[0] - deg[startpos,0] <= 1) and (XYpos[1] - deg[startpos,1] <= 1):   # only lower on first iteration and if degrees match
                    self.dwell_ms(10)
                    self.moveZ(pt=[XYpos[0],XYpos[1],0,0,z-descendZ]) # lowers to 9 units above detected opening
                    self.dwell_ms(1) # give time before the loop
                    for j in range(1, descendZ+2):     #carefully lower into opening and start at 0 just to also check current limit
                        self.dwell_ms(1)
                        self.moveZ(pt=[XYpos[0],XYpos[1],0,0,(z-descendZ)+j])
                        careful = self.getLimit()
                        if careful == 1:
                            self.moveZ(pt=[XYpos[0],XYpos[1],0,0,retreatZ])
                            break
            elif careful != 0:
                self.moveZ(pt=[XYpos[0],XYpos[1],0,0,retreatZ])
                break      #super overkill but more secure
        if careful != 1 and full == True:
            self.moveXYSpd(deg[endpos], spd)
            XYpos = deg[endpos]
            self.dwell_ms(1)
        elif careful != 0:
            self.moveZ(pt=[XYpos[0],XYpos[1],0,0,retreatZ])
        return {'endXY':XYpos, 'endDeg':endpos, 'oldMid': mid, 'limit': careful, 'startDeg': startpos}


    def tryOpening(self, mid, r, n=360, startpos=0, endpos=360, spd=1000, rest=5, z=53, full=True, retreatZ=10, descendZ=9):
        tryspd = spd
        trymid = mid
        trystart = startpos
        tryend = endpos
        tryz = z
        unsure=0
        radi = r
        careonce = 0
        trylower = self.moveCirc2(mid=trymid, r=radi, n=360, startpos=trystart, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
        startposFirst = startpos
        while trylower['limit'] == 1 and unsure != 1:
            for cw in xrange(2,10,2):
                if trylower['limit'] == 1:
                    careonce = 1
                    startpos = startpos + cw
                    trylower = self.moveCirc2(mid=trymid, r=radi, n=360, startpos=startpos, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
                else:
                    break
            startpos = startposFirst
            for ccw in xrange(2,10,2):       # can skip 0 degree offset bc clockwise covered it
                if trylower['limit'] == 1:
                    careonce = 1
                    startpos = startpos - ccw
                    trylower = self.moveCirc2(mid, r=radi, n=360, startpos=startpos, endpos=tryend, spd=tryspd, rest=5, z=tryz, full=True, retreatZ=42, descendZ=descendZ)
                else:
                    break
            if trylower['limit'] == 1:
                print 'Could not find opening - detecting anew...'
                unsure = 1
                self.homeZ2()
        trylower.update({'limitonce':careonce})
        return trylower


    def lowerCare(self, z, descendZ=9, retreatZ=18):        # z: depth of descend; descendZ: number of careful steps to reach depth z; retreatZ: RELATIVE height retreat upon hit
        """
        Step-by-step lowering of fly-manipulating end effector with
        hit-detection.
        """
        if z > 55 or z < 0:
            print 'Z not in range 0,55 - skipping...'
            return
        if descendZ > z:
            print 'descendZ larger than Z - correcting...'
            descendZ = z-1
        posbegin = self.getCurrentPosition()
        self.moveZ(pt=[posbegin[0],posbegin[1],10,0,z-descendZ])
        for i in range(1, descendZ+2):
            self.dwell_ms(1)
            self.moveRel(pt=[0,0,0,0,1])
            careful = self.getLimit()
            if careful == 1:
                self.moveRel(pt=[0,0,0,0,-retreatZ])
                break
        posend = self.getCurrentPosition
        return {'pos.begin': posbegin, 'pos.end': posend, 'limit': careful}


    def detectMotionAt(self, camcoordX, camcoordY, camcoordZ):
        """Moves to coordinates and returns whether movement was detected
        """
        self.moveToSpd(pt=[float(camcoordX), float(camcoordY), 0, camcoordZ, 10], spd=5000)
        self.dwell_ms(10)
        flyremaining = self.detectMotion( minpx=40, maxpx=2000)
        return flyremaining


    def getLimit(self):     # if this breaks look at position of limit max B in the string!
        limitgot = 0
        while limitgot < 10:
            try:
                templimit = str(self.smoothie.sendCmdGetReply("M119\n").split(' '))
                limit = int(templimit[150])
                limitgot = 10
            except:
                limitgot = limitgot + 1
        if limit == 1:
            print 'Fly manipulator limit switch hit.'
        return limit


    def isPtInBounds(self, pt):
        """
        Check whether desired coordinate (vector of exactly len 5) is larger
        than maximum workspace size or smaller than 0.
        """
        if len(pt) != 5:
            raise ValueError('incorrect coordinate length.')

        if (  ( pt[0] > self.maxExtents[0] ) or
              ( pt[1] > self.maxExtents[1] ) or
              ( pt[2] > self.maxExtents[2] ) or
              ( pt[3] > self.maxExtents[3] ) or
              ( pt[4] > self.maxExtents[4] ) or
              ( pt[0] < 0.0 ) or ( pt[1] < 0.0 ) or ( pt[2] < 0.0 ) or ( pt[3] < 0.0 ) or ( pt[4] < 0.0 ) ):
            return False
        else:
            return True


    def dwell_ms(self, milliseconds):
        """Dwell (do nothing) for the specified number of milliseconds.
        """
        # TODO why have this retry logic at all? why outside of flysorterserial?
        # why only for this command?
        cmd = "G04 P{0}\n".format(milliseconds)
        self.smoothie.sendSyncCmd(cmd)
        '''
        while True:
            try:
                # P is milliseconds *unless* we are in "grbl mode" in which
                # case, it is interpreted as float seconds.
                # We are assuming the Smoothieware is not in this mode.
                cmd = "G04 P{0}\n".format(milliseconds)
                self.smoothie.sendSyncCmd(cmd)
                break

            # TODO at the very least, catch a specific exception
            except:
                print 'Error: retrying to send dwell_ms command'
        '''


    def light(self, state):
        """Controls light-circle LED around camera
        """
        if state:
            cmd = "M48\n"
        else:
            cmd = "M49\n"
        self.smoothie.sendCmd(cmd)
        return


    def flyManipAir(self, state):
        if state:
            cmd = "M46\n"
        else:
            cmd = "M47\n"
        self.smoothie.sendCmd(cmd)


    def flyManipVac(self, state):
        """Controls negative air pressure out of fly manipulating end effector
        """
        if state:
            cmd = "M44\n"
        else:
            cmd = "M45\n"
        self.smoothie.sendCmd(cmd)


    def smallPartManipAir(self, state):
        if state:
            cmd = "M42\n"
        else:
            cmd = "M43\n"
        self.smoothie.sendCmd(cmd)


    def smallPartManipVac(self, state):
        """Controls negative air pressure out of part manipulating end effector

        (Holds part)
        """
        if state:
            cmd = "M40\n"
        else:
            cmd = "M41\n"
        self.smoothie.sendCmd(cmd)


    def unused_fet(self, state):
        if state:
            cmd = "M50\n"
        else:
            cmd = "M51\n"
        self.smoothie.sendCmd(cmd)


    def SavePicAt(self, Xcoords, Ycoords, IndVect, qualPic=25, Zcam=40, ImgName='errImage.jpg'):
        """Captures arena picture at location
        
        Images named consecutively if multiple coordinates specified
        """
        if not self.cam_enabled:
            raise CamDisabledError('Camera needed to save images.')

        self.light(True)
        for ImgNum in range(len(Xcoords)):
            self.moveToSpd(pt=[float(Xcoords[ImgNum]), float(Ycoords[ImgNum]), 0, Zcam, 10, 5000])
            self.dwell_ms(50)      # Put higher to reduce effect of motion-caused rig trembling on picture
            curInd = str(IndVect[ImgNum])
            self.cam.write_jpg(curInd + 'errImage.jpg', quality=qualPic)
            self.dwell_ms(10)
        self.light(False)


    def findFly(self, image):
        """Finds immobile fly on white surface (CO2 board)
        """
        # Convert BGR to HSV
        h, s, v = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))
        # Now threshold in the value channel
        r, mask = cv2.threshold(255-v, 100, 255, cv2.THRESH_BINARY)
        # Find contours
        contours, h = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            mmnts = cv2.moments(c)
            if ( 20000 < mmnts['m00'] < 200000 ):
                # Center of contour is m10/m00, m01/m00
                # We want transform from pixels to mm, using self.FOV
                # Find the shape of the image
                (pxHeight, pxWidth) = mask.shape
                imgCoords = np.array([ int(mmnts['m10'] / mmnts['m00'] ), int( mmnts['m01'] / mmnts['m00']) ], dtype=np.int16)
                cv2.line(image, tuple(imgCoords - np.array([ 20, 0])), tuple(imgCoords + np.array([ 20, 0])), (0,0,0), 5)
                cv2.line(image, tuple(imgCoords - np.array([ 0, 20])), tuple(imgCoords + np.array([ 0, 20])), (0,0,0), 5)
                cv2.line(image, tuple(imgCoords - np.array([ 20, 0])), tuple(imgCoords + np.array([ 20, 0])), (255,255,255), 3)
                cv2.line(image, tuple(imgCoords - np.array([ 0, 20])), tuple(imgCoords + np.array([ 0, 20])), (255,255,255), 3)
                coords = np.array([ (mmnts['m10'] / mmnts['m00'] - pxWidth/2.)*self.FOV[0]/pxWidth,
                                    (mmnts['m01'] / mmnts['m00'] - pxHeight/2.)*self.FOV[1]/pxHeight ])
                return coords
        return None


    def detectMotion(self, minpx=40, maxpx=800,showimg=False):
        """
        Compares 2 images ~800ms apart and returns 1 if minpix < detected
        difference pixels < maxpix
        """
        self.light(True)
        time.sleep(0.2)
        self.light(True)
        time.sleep(0.2)
        image1 = self.captureImage()
        self.dwell_ms(800)
        image2 = self.captureImage()
        self.light(False)
        image1 = cv2.resize(image1, (1280, 960))
        h1, s1, v1 = cv2.split(cv2.cvtColor(image1, cv2.COLOR_BGR2HSV))
        image2 = cv2.resize(image2, (1280, 960))
        h2, s2, v2 = cv2.split(cv2.cvtColor(image2, cv2.COLOR_BGR2HSV))
        image = cv2.subtract(v1,v2)
        ret,gray = cv2.threshold(image,25,255,0)
        gray2 = gray.copy()
        gray2 = cv2.morphologyEx(gray2, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
        gray2 = cv2.Canny(gray2,30,100)
        gray2 = np.nonzero(gray2)
        gray2 = len(np.nonzero(gray2)[0])
        if showimg == True:
            cv2.imshow('image', gray2)
            cv2.waitKey(0)
        if minpx<gray2<maxpx:
            return True
        else:
            return False


    def sweep(self, ptsx, ptsy, camz=45, spd=5000):
        """
        Input coordinate vector is returned as logic depending on iterating
        detectMotion() on each index
        """
        detectvect = range(len(ptsx))
        indvect = np.array(range(len(ptsx)))
        for i in range(len(ptsx)):
            self.moveToSpd(pt=[float(ptsx[i]), float(ptsy[i]), 0, camz, 0], spd=spd)
            detectvect[i] = self.detectMotion()
        detectvect = np.array(detectvect, dtype = bool)
        indvect = np.array(indvect[detectvect])
        return indvect


    def findOpening(self, image, slowmode=False, MAX_SIZE=74, MIN_SIZE=63, startp1=119, startp2=142, startp3=2.7, imgshow=0):
        """
        Finds circle in input image. Used to find opening in arena lid to allow
        access.
        """
        result = []
        detect = 0
        if slowmode == False:
            image = cv2.resize(image, (1280, 960))
            output = image.copy()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
            while detect == 0:      # decrease sensitivity until at least one circle is found
                circles = cv2.HoughCircles(thresh,cv2.cv.CV_HOUGH_GRADIENT,startp3,50, param1=startp1,param2=startp2,minRadius=MIN_SIZE,maxRadius=MAX_SIZE)
                 ## change param 1 and 2 for more-less circles
                if  circles is not None:
                    # convert the (x, y) coordinates and radius of the circles to integers
                    circles = np.round(circles[0, :]).astype("int")
                    # loop over the (x, y) coordinates and radius of the circles
                    for i in range(0,len(circles)):
                        # draw the circle in the output image, then draw a rectangle
                        # corresponding to the center of the circle
                        cv2.circle(output, (circles[i,0], circles[i,1]), circles[i,2], (0, 255, 0), 4)
                        cv2.rectangle(output, (circles[i,0] - 5, circles[i,1] - 5), (circles[i,0] + 5, circles[i,1] + 5), (0, 128, 255), -1)
                    if len(circles) == 1:
                        detect = 1
                    elif startp2 > 100 and len(circles) > 1:     #probably not starting to find only one circle
                        startp2 = startp2 - 3
                    elif startp2 <= 100 or len(circles) > 1:
                        detect = 1      # to get out if too many circles get found repeatedly -- unlikely to result in wrong angle as it needs validation
                else:
                    startp2 = startp2 - 3       # get less sensitive if no circles were found
            if imgshow == 1:
                cv2.imshow("thresh", thresh)
                cv2.imshow("output", output)
                cv2.waitKey(0)
        elif slowmode == True:      # Improves findOpening on the off-chance that something wrong in the image processing
            oldcircles = np.zeros((1,3), dtype=np.int)
            detect = 0
            while detect == 0:      # decrease sensitivity until at least one circle is found
                image3 = cv2.resize(image, (1280, 960))
                output = image3.copy()
                gray = cv2.cvtColor(image3, cv2.COLOR_BGR2GRAY)
                thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
                circles = cv2.HoughCircles(thresh,cv2.cv.CV_HOUGH_GRADIENT,startp3,50, param1=startp1,param2=startp2,minRadius=MIN_SIZE,maxRadius=MAX_SIZE)
                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    detect = 1
                    # show the output image
                    for i in range(0,len(circles)):
                     # draw the circle in the output image, then draw a rectangle
                     # corresponding to the center of the circle
                        if imgshow == 1:
                            cv2.circle(output, (circles[i,0], circles[i,1]), circles[i,2], (0, 255, 0), 4)
                            cv2.rectangle(output, (circles[i,0] - 5, circles[i,1] - 5), (circles[i,0] + 5, circles[i,1] + 5), (0, 128, 255), -1)
                            cv2.imshow("thresh", thresh)
                            cv2.imshow("output", output)
                            cv2.waitKey(0)
                else:
                    startp2 = startp2 - 3       # get less sensitive if no circles were found
        return circles


    def getDegs(self, circleList, img_width=1280, img_height=960):
        """
        Returns degrees of a point's coordinates relative to the image midpoint
        (the opening)
        """
        imgmid = [img_width/2, img_height/2]
        if len(circleList[:,1]) >= 2 and not (circleList[0,0] - circleList[1,0] >= 150) and not (circleList[0,1] - circleList[1,1] >= 150):    # allows 2 close circles and takes mean coords instead (Accuracy - Iteration tradeoff. Works well if less than 10px apart).
            circleList[0,0] = (circleList[0,0] + circleList[1,0])/2
            circleList[0,1] = (circleList[0,1] + circleList[1,1])/2
            print 'Multiple adjoining openings detected - correcting target coordinates...'
        dx = circleList[0,0] - imgmid[0]
        dy = (img_height - circleList[0,1]) - imgmid[1]
        rads = math.atan2(-dy,dx)
        rads %= 2*math.pi
        degs = math.degrees(rads)
        degs = (degs-90)* (-1)
        if degs <= 0:
            degs = 360 + degs
        return degs


    def findDegs(self, slowmode=True, precision=4, MAX_SIZE=74, MIN_SIZE=63, startp1=139, startp2=150, startp3=2.6, imgshow=0):
        """Combines findOpening and getDegs
        """
        trymax=MAX_SIZE
        trymin=MIN_SIZE
        try1 = startp1
        try2 = startp2
        try3 = startp3
        imgshow = imgshow
        slwmd = slowmode
        if slowmode == True:
            certain = 0
            while certain != 1:
                tempdeg = np.arange(2)
                for i in range(0,2):
                    self.light(True)
                    time.sleep(0.2)
                    img = self.captureImage()
                    self.light(False)
                    circles = self.findOpening(img, slowmode=slwmd, MAX_SIZE=trymax, MIN_SIZE=trymin, startp1=try1, startp2=try2, startp3=try3, imgshow=imgshow)
                    tempdeg[i] = self.getDegs(circles)
                if abs(tempdeg[0] - tempdeg[1]) <= precision:
                    certain = 1
                    return np.mean(tempdeg)
        elif slowmode == False:
            self.light(True)
            time.sleep(0.2)
            time.sleep(0.2)
            img = self.captureImage()
            self.light(False)
            circles = self.findOpening(img, slowmode=slwmd, MAX_SIZE=trymax, MIN_SIZE=trymin, startp1=try1, startp2=try2, startp3=try3, imgshow=imgshow)
            tempdeg = self.getDegs(circles)
            return tempdeg

