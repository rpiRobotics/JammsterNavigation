#!/usr/bin/env python
import rospy
import math
import serial
import struct
import time
import sys
from sensor_msgs.msg import Joy

#roboclaw
checksum = 0
m1CurrentDuty = 0
m2CurrentDuty = 0
m1MaxDuty = 160
m2MaxDuty = 160
port = serial.Serial("/dev/ttyACM0", baudrate=38400, timeout=.5)

xJoy = 0
yJoy = 0
newData = False

#********************************#
#             FUNCTIONS                #
#********************************#

def writebyte(val):
        global checksum
        checksum += val
        return port.write(struct.pack('>B',val));
def writesbyte(val):
        global checksum
        checksum += val
        return port.write(struct.pack('>b',val));
def writeword(val):
        global checksum
        checksum += val
        checksum += (val>>8)&0xFF
        return port.write(struct.pack('>H',val));
def writesword(val):
        global checksum
        checksum += val
        checksum += (val>>8)&0xFF
        return port.write(struct.pack('>h',val));
def writelong(val):
        global checksum
        checksum += val
        checksum += (val>>8)&0xFF
        checksum += (val>>16)&0xFF
        checksum += (val>>24)&0xFF
        return port.write(struct.pack('>L',val));
def writeslong(val):
        global checksum
        checksum += val
        checksum += (val>>8)&0xFF
        checksum += (val>>16)&0xFF
        checksum += (val>>24)&0xFF
        return port.write(struct.pack('>l',val));

def sendcommand(address,command):
        global checksum
        checksum = address
        port.write(chr(address));
        checksum += command
        port.write(chr(command));
        return;

def SetM1DutyAccel(accel,duty):
	sendcommand(128,52)
	writesword(duty)
	writeword(accel)
	writebyte(checksum&0x7F);
	return;

def SetM2DutyAccel(accel,duty):
	sendcommand(128,53)
	writesword(duty)
	writeword(accel)
	writebyte(checksum&0x7F);
	return;

def controlCall(data):
    print "callback"
    global xJoy, yJoy, newData
    xJoy = data.axes[0]
    yJoy = data.axes[1]
    print xJoy
    newData = True


def base_controller():
    global newData, xJoy, yJoy
    rospy.init_node('xboxControl', anonymous=True)
    rospy.Subscriber("/xbox/joy", Joy,  controlCall)

    while True:
        if not newData:
            continue
            
        m1Duty = (yJoy * -250) + (xJoy * 250)
        m2Duty = (yJoy * -250) - (xJoy * 250)
        #maintain bounds
        if abs(m1Duty) > m1MaxDuty:
            m1Duty = cmp(m1Duty, 0) * m1MaxDuty
        if abs(m2Duty) > m2MaxDuty:
            m2Duty = cmp(m2Duty, 0) * m2MaxDuty

        SetM1DutyAccel(400, int(m1Duty))
        SetM2DutyAccel(400, int(m2Duty))
        newData = False

if __name__ == '__main__':
    base_controller()

