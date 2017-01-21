# -*- coding: utf-8 -*-
"""
Created on Sat Dec 31 14:28:48 2016

@author: cunnia3
@description: simulation of a robot with imu's on each of its wheels
"""
from WheelchairObjects import *
import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from std_msgs.msg import Int16
from discrete_pid import PID
import math


class SimulatedRobotNode:
    def __init__(self, dt):
        self.dt = dt
        rospy.init_node('sim_robot', anonymous=True)
        self.true_state = np.zeros([3,1])        
        
        self.left_motor = SimulatedMotor(0)
        self.right_motor = SimulatedMotor(0)
        self.left_motor.state = 0
        self.right_motor.state = 0
        self.r = .15
        self.l = .55
        
        self.imu1_pub = rospy.Publisher('/imu1', Imu, queue_size = 1)
        self.imu2_pub = rospy.Publisher('/imu2', Imu, queue_size = 1)
        self.v1_pub = rospy.Publisher('left_voltage_pwm', Int16, queue_size = 1)     
        self.v2_pub = rospy.Publisher('right_voltage_pwm', Int16, queue_size = 1)
        
        rospy.Subscriber("/cmd_vel",  Twist,  self._command_callback)
        
    def _command_callback(self, data):
        """ Change motor commands depending on Twist command """
        print "Got command!"
        linearV=data.linear.x
        angularV=data.angular.z
        
        # calculate desired velocities for wheels see Georgia Tech mobile robot 2.2 video for equation
        self.left_motor.set_desired_speed( (2*linearV - angularV*self.l)/(2*self.r))
        self.right_motor.set_desired_speed( (2*linearV + angularV*self.l)/(2*self.r))        
        
    def _publish_motor_states(self):
        """ Publish motor voltages and imu readings for other nodes to use """
        imu1 = Imu()
        imu1.angular_velocity.z = self.left_motor._obtain_measurement() 
        self.imu1_pub.publish(imu1)
        imu2 = Imu()
        imu2.angular_velocity.z = self.right_motor._obtain_measurement()
        self.imu2_pub.publish(imu2)
        
        self.v1_pub.publish(self.left_motor.u)
        self.v2_pub.publish(self.right_motor.u)
        
    def _update_true_state(self):
        self.left_motor.update_dt(self.dt)
        self.right_motor.update_dt(self.dt)
        vl = self.left_motor.state
        vr = self.right_motor.state
        
        self.true_state[0,0] += self.r/2 * (vr+vl) * math.cos(self.true_state[2,0]) * self.dt
        self.true_state[1,0] += self.r/2 * (vr+vl) * math.sin(self.true_state[2,0]) * self.dt
        self.true_state[2,0] += self.r/self.l * (vr-vl) * self.dt
        
    def spin(self):
        r = rospy.Rate(1/self.dt)
        i = 0
        np.set_printoptions(2, suppress = True)
        while not rospy.is_shutdown():
            self._publish_motor_states()
            self._update_true_state()
            i+=1
            if i % 20 == 0:
                print self.true_state, self.left_motor.state, self.right_motor.state
            
            r.sleep()
        
my_sim_robot_node = SimulatedRobotNode(.01)
my_sim_robot_node.spin()