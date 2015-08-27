#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import serial
import struct
import time
import sys
from pid import PID
from kalman import KalmanFilter

#********************************#
#          GLOBAL VARS           #
#********************************#

#roboclaw
checksum = 0
m1CurrentDuty = 0
m2CurrentDuty = 0
m1MaxDuty = 180
m2MaxDuty = 180
port = serial.Serial("/dev/ttyACM3", baudrate=38400, timeout=.5)


#keep a filter for both linear velocity and angular velocity


newData = False


#safety
somethingWentWrong = False

#********************************#
#          FUNCTIONS             #
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
    
def commandCallback(data):
    global m1CurrentDuty, m2CurrentDuty, m1MaxDuty, m2MaxDuty, somethingWentWrong

    print "****NEW COMMAND RECEIVED***"
    
    linearV=data.linear.x
    angularV=data.angular.z

    #start each pulsewidth with the offset required to get the wheelchair moving
    m1CurrentDuty = cmp(linearV, 0)*-50 + -1*linearV*m1MaxDuty 
    m2CurrentDuty = cmp(linearV, 0)*-50 + -1*linearV*m2MaxDuty


    #maintain bounds for linear  
    if abs(m1CurrentDuty) > m1MaxDuty:
       m1CurrentDuty = cmp(m1CurrentDuty, 0) * m1MaxDuty
    if abs(m2CurrentDuty) > m2MaxDuty:
      m2CurrentDuty = cmp(m2CurrentDuty, 0) * m2MaxDuty

    m1CurrentDuty = m1CurrentDuty + 1.3*angularV*m1MaxDuty
    m2CurrentDuty = m2CurrentDuty - 1.3*angularV*m2MaxDuty

    #maintain bounds for angular
    if abs(m1CurrentDuty) > m1MaxDuty:
       m1CurrentDuty = cmp(m1CurrentDuty, 0) * m1MaxDuty
    if abs(m2CurrentDuty) > m2MaxDuty:
      m2CurrentDuty = cmp(m2CurrentDuty, 0) * m2MaxDuty
     
    #stop the wheelchair if the command is to stop
    if linearV==0 and angularV==0:
      m1CurrentDuty = 0
      m2CurrentDuty = 0

    print "M1 duty cycle set to ",  m1CurrentDuty
    print "M2 duty cycle set to ",  m2CurrentDuty
    SetM1DutyAccel(500,  int(m1CurrentDuty))
    SetM2DutyAccel(500,  int(m2CurrentDuty))
    
    

def base_controller():
    rospy.init_node('base_controller', anonymous=True)
    rospy.Subscriber("/cmd_vel",  Twist,  commandCallback)

    odom = Odometry()
    global newData, somethingWentWrong
    
    while True:
        try:
            
	    odom.twist.twist.angular.y = 0

        except KeyboardInterrupt:
            print "Stopping, as commanded"
            SetM1DutyAccel(500,  0)
            SetM2DutyAccel(500,  0)
            sys.exit()
    

if __name__ == '__main__':
    base_controller()
