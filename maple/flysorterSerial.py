##
## This copyrighted software is distributed under the GPL v2.0 license.
## See the LICENSE file for more details.
##

## Serial port handling class

import os
import sys
import time
import glob

import serial

import errs

# Serial communications class that is used for multiple devices.
#
# In MAPLE, the smoothie board is a serial devices. The smoothie is connected directly
# via USB.
#
# Documentation is available online for G-codes (smoothie):
#
# http://smoothieware.org/supported-g-codes and
# http://reprap.org/wiki/G-code
#
# This class is also used to communicate with the Fly Dispenser (if connected & used).
# The Dispenser, similar to the Smoothie, connects by USB and appears as a COM port.

class serialDevice:
    """Serial class for FlySorter serial device (FlyDispenser, MAPLE robot, etc)."""

    WaitTimeout = 3
    portName = ""

    def __init__(self, port, baud = 9600, timeout = float(0.1)):
        self.isOpened = False
        try:
            self.ser = serial.Serial(port, baudrate = baud, timeout = timeout)
        except:
            raise IOError('Failed to open port {}'.format(port))

        self.isOpened = True

    def close(self):
        self.ser.close()

    # Retrieve any waiting data on the port
    def getSerOutput(self):
        #print "GSO:"
        output = ''
        while True:
            # read() blocks for the timeout set above *if* there is nothing to read
            #   otherwise it returns immediately
            byte = self.ser.read(1)
            if byte is None or byte == '':
                break
            output += byte
            if byte == '\n':
                break
        #print "GSO Output:", output
        # TODO TODO handle limit siwtch here? (would it just be missed now?)
        return output

    # Block and wait for the device to reply with "ok" or "OK"
    # Times out after self.WaitTimeout (set above)
    def waitForOK(self):
        # TODO TODO disregard all warnings? or maybe only propagate them up as
        # warnings, rather than runtimeerrors?
        # TODO or should i change the code to avoid causing the warning about
        # homing after a half (see errors.txt in choice/maple for details on
        # this error)?
        #print "WFO:"
        output = ''
        timeoutMax = self.WaitTimeout / self.ser.timeout
        timeoutCount = 0
        while True:
            byte = self.ser.read(1)
            if byte is None or byte == '':
                timeoutCount += 1
                time.sleep(1)
            else:
                output += byte
            if timeoutCount > timeoutMax:
                print 'Serial timeout.'
                break
            if byte == '\n':
                break

        if not (output.startswith("ok") or output.startswith("OK")):
            # maybe refactor so this isn't all done with exceptions?

            # TODO this won't be discovered until a few commands later though...
            # TODO is this ever triggered? (searching for right string?)
            #print('waitForOK output: {}'.format(output))
            # TODO TODO TODO print last height in this case, for faster setting
            # of appropriate working height
            if output.startswith('Limit switch B was hit'):
                raise errs.FlyManipulatorCrashError

            elif output.startswith('WARNING: After HALT you should HOME ' +
                'as position is currently unknown'):

                #raise errs.NeedToHomeError
                return

            # TODO what is hex encoding useful for?
            # TODO where are the "!!" lines coming from? error in
            # above parsing or something meaningful from smoothie?
            # does !! mean a limit switch was hit or something?
            # (!! response to everything from switch hit to M999/reset?)
            raise RuntimeError('Unexpected serial output: {} ({})'.format(
                output.rstrip('\r\n'),
                ':'.join(x.encode('hex') for x in output)))


    # Send a command to the device via serial port
    # Asynchronous by default - doesn't wait for reply
    def sendCmd(self, cmd):
        #print "SC:", cmd
        self.ser.write(cmd)
        self.ser.flush()

    # Send a command to the device via serial port
    # Waits to receive reply of "ok" or "OK" via waitForOK()
    # TODO TODO what is the point of checking for OK? it does not mean the
    # command finished successfully (just that it was sent? sent and will be
    # excecuted if nothing goes wrong in between? anything more?)
    def sendSyncCmd(self, cmd):
        #print "SSC:", cmd
        self.ser.flushInput()
        self.ser.write(cmd)
        self.ser.flush()
        try:
            self.waitForOK()

        except RuntimeError:
            # TODO log
            print('Unexpected output on command {}'.format(cmd))
            raise

        except errs.NeedToHomeError:
            # Otherwise, we ARE homing, so it doesn't matter.
            if not cmd.startswith('G28'):
                raise


    # Send a command and retrieve the reply
    def sendCmdGetReply(self, cmd):
        self.ser.flushInput()
        self.ser.write(cmd)
        self.ser.flush()
        out = self.getSerOutput()
        # TODO this a read flush too? if not, just read until no bytes left
        self.ser.flush()
        # TODO maybe test w/ another getSerOutput / other read call
        return out


def availablePorts():
    """Lists serial ports"""

    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result
