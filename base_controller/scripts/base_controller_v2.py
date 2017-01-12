#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 21 10:46:10 2015

@author: Andrew Cunningham
"""
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import time
from discrete_pid import PID
import roboclaw
#********************************#
#              OBJECTS           #
#********************************#

class MovingAverage:
    """
    used to filter imu data
    
    """
    def __init__(self):
        # array of readings
        self.readings = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # index of last reading added
        self.index = 0
        
    def get(self):
        """
        gets an averaged reading from the pool of collected data
        """
        return sum(self.readings) / float(len(self.readings))
        
    def add(self, new):
        """
        incorporate a new reading into the moving average
        """
        self.readings[self.index] = new
        self.index = self.index + 1
        if self.index >= len(self.readings):
            self.index = 0
            
class RoboClaw:
    """ 
    used to control the roboclaw
    """
    def __init__(self):
        # Roboclaw parameters
        self.m1Duty = 0
        self.m2Duty = 0
        #largest absolute value that the motors can be set to
        self.dutyMax = 5000
        
        self.address = 0x80
        roboclawPort = '/dev/ttyACM1'
        roboclaw.Open(roboclawPort,115200)

        
    def writeM1M2(self,m1Duty,m2Duty):
        if abs(m1Duty) > self.dutyMax:
            self.m1Duty = self.dutyMax * cmp(m1Duty, 0)
        else:
            self.m1Duty = m1Duty
            
        if abs(m2Duty) > self.dutyMax:
            self.m2Duty = self.dutyMax * cmp(m2Duty, 0)
        else:
            self.m2Duty = m2Duty
         
        #print self.m1Duty,self.m2Duty
        roboclaw.DutyAccelM1(self.address,5000,self.m1Duty)
        roboclaw.DutyAccelM2(self.address,5000,self.m2Duty)
            
class BaseController:
    """
    coordinates motor commands with imu odometry to follow velocity commands
    """
    def __init__(self):
        # Set PID structures to intial values
        self.rightPID = PID()
        self.rightPID.setPoint(0)
        self.leftPID = PID()
        self.leftPID.setPoint(0)
        
        self.somethingWentWrong = False
        
        # Ensure that data is fresh from both wheels
        self.newDataM1 = False
        self.newDataM2 = False
        self.newData = False
        
        # IMU structures
        self.currentLeftV = MovingAverage()
        self.currentRightV = MovingAverage()
        
        # Roboclaw
        self.myRoboclaw = RoboClaw()
        
        # ROS subscribers
        rospy.Subscriber("/cmd_vel",  Twist,  self.commandCallback)
        rospy.Subscriber("/imu1", Imu, self.imu1Callback)
        rospy.Subscriber("/imu2", Imu, self.imu2Callback)
        time.sleep(1)
        
        self.lastDataTime = time.time()
        
        self.repeatCount = 0
        self.currentM1Measurement = 0
        self.currentM2Measurement = 0
        self.lastM1Measurement = 0
        self.lastM2Measurement = 0
                
    
    def imu1Callback(self,data):
        self.newDataM1 = True
        self.currentM1Measurement = data.angular_velocity.z
        self.currentLeftV.add(data.angular_velocity.z)
        
    def imu2Callback(self,data):
        self.newDataM2 = True
        self.currentM2Measurement = data.angular_velocity.z
        self.currentRightV.add(data.angular_velocity.z)
    
    def commandCallback(self, data):
        """
        Will set the internal PID controller's setpoints according to the command
        """
        print "****NEW COMMAND RECEIVED***"
        linearV=data.linear.x
        angularV=data.angular.z
        # calculate desired velocities for wheels see Georgia Tech mobile robot 2.2 video for equation
        L=.57
        R=.9424
        self.leftPID.setPoint( (2*linearV - angularV*L)/(2*R))
        self.rightPID.setPoint( (2*linearV + angularV*L)/(2*R))
        
    def shutdown(self):
        """
        To be called if something went wrong, stop the roboclaw and exit out
        """
        print "shutting down"
        self.myRoboclaw.writeM1M2(0,0)
        exit()
        
    def commandLoop(self):
        """
        Main loop that writes the commands to the roboclaw.  If new data is received
        at a rate lower than 4Hz, then the wheelchair will shutdown
        """
        
        # Thigns to check:
        #  1) We got new data at the correct frequency
        #  2) We are not getting repeat data from our imus (imus are still working)
        
        # run loop 20 times a second
        newLeft = 0
        newRight = 0
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            # Wait until we get new data from both motors
            if self.newDataM1 and self.newDataM2:
                self.newData = True
                
            if not self.newData:
                if time.time() - self.lastDataTime > .5:
                    print "Too long between messages!"
                    self.shutdown()
                else:
                    continue
            
            # We have data, ensure that it isnt repeated data
            else:
                self.lastDataTime = time.time()
                self.newDataM1 = False
                self.newDataM2 = False
                self.newData = False
                
                # Check the number of repeat readings
                if self.currentM1Measurement == self.lastM1Measurement or self.currentM2Measurement == self.lastM2Measurement:
                    self.repeatCount +=1
                
                else:
                    self.repeatCount = 0
                
                # If we have too many repeats, shut the system down
                if self.repeatCount > 4:
                    print "Too many repeats!"
                    self.shutdown()
                    
                self.lastM1Measurement= self.currentM1Measurement
                self.lastM2Measurement= self.currentM2Measurement           
                
                newLeft = newLeft + int(self.leftPID.update(self.currentLeftV.get()))
                newRight = newRight + int(self.rightPID.update(self.currentRightV.get()))
                
                #catch special case of not moving
                if self.leftPID.getPoint() == 0 and self.rightPID.getPoint() == 0:
                    newLeft = 0
                    newRight = 0
                if abs(newLeft) > 0:                
                    print "LEFT setpoint,  measurement, update",'%1.2f' % self.leftPID.getPoint(), '%1.2f' % self.currentLeftV.get(), '%1.2f' % newLeft, '%1.2f' % int(self.leftPID.update(self.currentLeftV.get()))
                    print "RIGHT setpoint, measurement, update",'%1.2f' % self.rightPID.getPoint(), '%1.2f' % self.currentRightV.get(),'%1.2f' % newRight, '%1.2f' % int(self.rightPID.update(self.currentRightV.get()))
                self.myRoboclaw.writeM1M2(newRight,newLeft) # MAKE SURE THESE ARE IN THE CORRECT ORDER!!!!
                
            r.sleep()
            

def main():
    rospy.init_node('base_controller', anonymous=True)
    
    myBaseController = BaseController()
    myBaseController.commandLoop()
    rospy.spin()
            
if __name__ == '__main__':
    main()
