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
#              OBJECTS                     #
#********************************#

class MovingAverage:
    def __init__(self):
        # array of readings
        self.readings = [0, 0, 0, 0, 0,  0,  0,  0,  0,  0]
        # index of last reading added
        self.index = 0
        
    def get(self):
        return sum(self.readings) / float(len(self.readings))
        
    def add(self, new):
        self.readings[self.index] = new
        self.index = self.index + 1
        if self.index >= len(self.readings):
            self.index = 0


#********************************#
#             GLOBAL VARS              #
#********************************#

#roboclaw
checksum = 0
m1CurrentDuty = 0
m2CurrentDuty = 0
m1MaxDuty = 160
m2MaxDuty = 160
port = serial.Serial("/dev/ttyACM0", baudrate=38400, timeout=.5)


#keep a filter for both linear velocity and angular velocity
currentLeftV = MovingAverage()
currentRightV = MovingAverage()
#currentLeftV = KalmanFilter(.05, .05)
#currentRightV = KalmanFilter(.05, .05)


#indicate whether new data has been received
newData = False

#PID objects
		#p    i    d
rightPID = PID( 5.0, 0, 500, 0, 0, 100, -100)
rightPID.setPoint(0)
leftPID = PID(  5.0, 0, 500, 0, 0, 100, -100)
leftPID.setPoint(0)

#safety
somethingWentWrong = False

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


def odometryCallback(data):
    global m1CurrentDuty, m2CurrentDuty, somethingWentWrong, newData
    currentLeftV.add(data.twist.twist.angular.x)
    currentRightV.add(data.twist.twist.angular.y)
    newData = True
    if not somethingWentWrong:
       print "Current  left V %.2f " % currentLeftV.get(), "Desired  left V %.2f " % leftPID.getPoint(), "Duty %.1f" % m1CurrentDuty
       print "Current right V %.2f " % currentRightV.get(), "Desired right V %.2f " % rightPID.getPoint(), "Duty %.1f" % m2CurrentDuty

    else:
       print "Something went wrong"
    
    
def commandCallback(data):
    print "****NEW COMMAND RECEIVED***"
    linearV=data.linear.x
    angularV=data.angular.z
    # calculate desired velocities for wheels see Georgia Tech mobile robot 2.2 video for equation
    L=.57
    R=.9424
    leftPID.setPoint( (2*linearV - angularV*L)/(2*R) )
    rightPID.setPoint( (2*linearV + angularV*L)/(2*R) )

# put platform specific code here
def executeCommand(commandedLeftV,  commandedRightV):
    global m1CurrentDuty, m2CurrentDuty, somethingWentWrong

    m1CurrentDuty = m1CurrentDuty + (commandedLeftV * -1)
    m2CurrentDuty = m2CurrentDuty + (commandedRightV * -1)
    
    #maintain bounds
    if abs(m1CurrentDuty) > m1MaxDuty:
       m1CurrentDuty = cmp(m1CurrentDuty, 0) * m1MaxDuty
    if abs(m2CurrentDuty) > m2MaxDuty:
      m2CurrentDuty = cmp(m2CurrentDuty, 0) * m2MaxDuty
     
    #catch special case of not moving
    if leftPID.getPoint() == 0 and rightPID.getPoint() == 0:
        m1CurrentDuty = 0
        m2CurrentDuty = 0


    #safety feature if the duty cycle is higher than a threshold and there is no movement
    #turn off movement
    if abs(m1CurrentDuty) > 250 and abs(currentLeftV.get()) < .05:
        leftPID.setPoint(0)
        m1CurrentDuty = 0
	somethingWentWrong = True

    if abs(m2CurrentDuty) > 250 and abs(currentRightV.get()) < .05:
        rightPID.setPoint(0)
        m2CurrentDuty = 0
	somethingWentWrong = True

    #print "M1 duty cycle set to ",  m1CurrentDuty
    #print "M2 duty cycle set to ",  m2CurrentDuty
    SetM1DutyAccel(200,  int(m1CurrentDuty))
    SetM2DutyAccel(200,  int(m2CurrentDuty))
    
    

def base_controller():
    rospy.init_node('base_controller', anonymous=True)
    rospy.Subscriber("/odom", Odometry,  odometryCallback)
    rospy.Subscriber("/cmd_vel",  Twist,  commandCallback)

    wheelPub = rospy.Publisher("/DEBUG/desired_wheel_velocities", Odometry)
    odom = Odometry()
    global newData, somethingWentWrong
    
    while True:
        try:
            if not newData:
                continue
            
            newLeft = leftPID.update(currentLeftV.get())
            newRight = rightPID.update(currentRightV.get())
            executeCommand(newLeft,  newRight)
            newData = False

            odom.header.stamp = rospy.Time.now()
            odom.twist.twist.angular.x = leftPID.getPoint()
	    odom.twist.twist.angular.y = rightPID.getPoint()
	    wheelPub.publish(odom)

        except KeyboardInterrupt:
            print "Stopping, as commanded"
            SetM1DutyAccel(500,  0)
            SetM2DutyAccel(500,  0)
            sys.exit()
    

if __name__ == '__main__':
    base_controller()
