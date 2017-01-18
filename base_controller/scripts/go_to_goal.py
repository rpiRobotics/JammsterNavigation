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

class GoToGoal:
    """ Simple go to point controller.  Does nothing for obstacle avoidance"""
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)        
        rospy.Subscriber("/odometry", Odometry , self._odomCallback)
        rospy.Subscriber("/goal", Pose, self._goalCallback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.my_pid = PID(P=4.0, I=0.0, D=1.0, Derivator=0, Integrator=1, Integrator_max=10, Integrator_min=-10)
        self.v = 0
        self.w = 0
        
        self.current_x = 0
        self.current_y = 0
        self.current_bearing = 0
        
        self.desired_x = 0
        self.desired_y = 0
        self.desired_bearing = 0        
        
        self.close_enough = False

    def _goalCallback(self, data):
        self.desired_x = data.position.x
        self.desired_y = data.position.y

    def _odomCallback(self, data):
        quat = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(quat)
        self.current_bearing = euler[2]
        
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        self.desired_bearing = math.atan2(self.desired_y-self.current_y, self.desired_x-self.current_x)
        if math.sqrt((self.current_x - self.desired_x)**2 + (self.current_y - self.desired_y)**2) > .2:
            self.close_enough = False
        else:
            self.close_enough = True
            
        return

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.my_pid.set_point(self.desired_bearing)
            self.w=self.my_pid.update(self.current_bearing)
            
            if self.close_enough:
                self.v = 0
                self.w = 0
            else:
                self.v = .2
                
            msg = Twist()
            msg.linear.x = self.v
            msg.angular.z = self.w
            self.pub.publish(msg)            
            r.sleep()