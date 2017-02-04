#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 17 18:55:13 2017

@author: cunnia3
"""

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import tf
from pid import PID

def deltaAngle(x, y):
    return math.atan2(math.sin(x-y), math.cos(x-y))

class GoToGoal:
    """ Simple go to point controller.  Does nothing for obstacle avoidance"""
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)        
        rospy.Subscriber("/odometry", Odometry , self._odomCallback)
        rospy.Subscriber("/goal", Pose, self._goalCallback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.w_pid = PID(P=2.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=2, Integrator_min=-2)
        self.v = 0
        self.w = 0

        self.v_max = .2
        self.w_max = .2
        
        self.current_x = 0
        self.current_y = 0
        self.current_bearing = 0
        
        self.desired_x = 0
        self.desired_y = 0
        self.desired_bearing = 0        
        
        self.close_enough = True

    def _goalCallback(self, data):
        print "Got new goal"
        self.desired_x = data.position.x
        self.desired_y = data.position.y
        quat = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.desired_bearing = euler[2]

    def _odomCallback(self, data):
        quat = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.current_bearing = euler[2]
        
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y

        if math.sqrt((self.current_x - self.desired_x)**2 + (self.current_y - self.desired_y)**2) > .4:
            self.close_enough = False
        else:
            self.close_enough = True            
        return

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # STOP CASE
            if self.desired_x == 0 and self.desired_y ==0:
                print "STOP MODE"
                self.v = 0
                self.w = 0 

            # CLOSE ENOUGH CASE
            elif self.close_enough:
                print "CLOSE ENOUGH MODE"
                self.v = 0
                self.w_pid.setPoint(self.desired_bearing)
                self.w = self.w_pid.update(self.current_bearing)
        
            # MOVE TO POINT CASE
            else:
                d_angle = math.atan2((self.desired_y - self.current_y), (self.desired_x - self.current_x))
                # TURN TOWARDS GOAL
                if abs(deltaAngle(d_angle,self.current_bearing)) > .25:
                    print "TURN TOWARDS GOAL MODE"
                    print abs(d_angle - self.current_bearing)
                    self.v = 0
                    self.w_pid.setPoint(d_angle)
                    self.w = self.w_pid.update(self.current_bearing)

                # GO TOWARDS GOAL
                else:
                    print "GO TOWARDS GOAL MODE"
                    self.w = 0
                    self.v = .2


            if self.v > self.v_max:
                self.v = self.v_max
            if self.v < -self.v_max:
                self.v = -self.v_max

            if self.w > self.w_max:
                self.w = self.w_max
            if self.w < -self.w_max:
                self.w = -self.w_max                

            print "%.2f, %.2f, %.2f" %(self.w_pid.getError(), self.desired_bearing, self.current_bearing)
            msg = Twist()
            msg.linear.x = self.v
            msg.angular.z = self.w
            self.pub.publish(msg)            
            r.sleep()

go_goal_node = GoToGoal()
go_goal_node.spin()
