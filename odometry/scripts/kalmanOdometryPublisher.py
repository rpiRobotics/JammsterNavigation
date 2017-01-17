#!/usr/bin/env python
import numpy as np
import math
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
from std_msgs.msg import Int16
import copy

class ExtendedWMRKalmanFilter:
    """ Class to predict a wheeled mobile robot state [dtheta_l, dtheta_r,x,y,theta]
        from imu measurements on wheels and base.  Will include SLAM based stuff later"""
    def __init__(self, r, l, Q, R, k1_l, k2_l, k1_r, k2_r, starting_state, dt):
        """ @r radius of wheels
            @l length of axle
            @Q covariance of process noise
            @R covariance of measurement noise
            @starting_state [dtheta_l, dtheta_r,x,y,theta]
            @k1_l, k2_l motor constants for left motor
            @k1_r, k2_r motor constants for right motor
            @dt the time in between predicts """
        print "CREATING EKF"            
        self.r = r
        self.l = l
        self.Q = Q
        self.R = R
        self.k1_l = k1_l
        self.k2_l = k2_l
        self.k1_r = k1_r
        self.k2_r = k2_r
        self.current_state_estimate = copy.deepcopy(starting_state)
        self.current_prob_estimate = np.zeros([starting_state.shape[0], starting_state.shape[0]])
        self.dt = dt
        
        self.A = np.eye(2)
        self.B = np.eye(2)
        self.H = np.array([ [1,0],[0,1],[0,0],[0,0],[0,0] ]).transpose() # assume we can only read wheel velocities for the time being
        
    def predict(self, u):
        """ Predict new state from old state and command
        @u [v_l v_r], Step without sensing """

        dtheta_l = self.current_state_estimate[0,0]
        dtheta_r = self.current_state_estimate[1,0]
        theta = self.current_state_estimate[4,0]
        
        self.A = np.array([[1+self.dt*self.k2_l, 0, 0, 0, 0],\
                           [0, 1+self.dt*self.k2_r, 0, 0, 0],\
                           [self.dt*(self.r)/2*math.cos(theta), self.dt*self.r/2*math.cos(theta), 1, 0, -math.sin(theta)*self.dt*(dtheta_r+dtheta_l)],\
                           [self.dt*(self.r)/2*math.sin(theta), self.dt*self.r/2*math.sin(theta), 0, 1, math.cos(theta)*self.dt*(dtheta_r+dtheta_l)],\
                           [-self.dt*self.r/self.l, self.dt*self.r/self.l, 0, 0, 1]])
                           
        self.B = np.array([[self.dt*self.k1_l, 0],\
                           [0, self.dt*self.k1_r],\
                           [0, 0],\
                           [0, 0],\
                           [0, 0]])
                        
        self.current_state_estimate = np.dot(self.A, self.current_state_estimate) + np.dot(self.B, u)
        self.current_prob_estimate = np.dot(np.dot(self.A, self.current_prob_estimate), np.transpose(self.A)) + self.Q

    def measurement_update(self, measurement_vector):
        """ Update Kalman filter if sensing information is received """
       #--------------------------Observation step-----------------------------
        innovation = measurement_vector - np.dot(self.H, self.current_state_estimate)
        innovation_covariance = np.dot(np.dot(self.H, self.current_prob_estimate), np.transpose(self.H)) + self.R
        #-----------------------------Update step-------------------------------
        kalman_gain = np.dot(np.dot(self.current_prob_estimate, np.transpose(self.H)), np.linalg.inv(innovation_covariance))
        self.current_state_estimate = self.current_state_estimate + np.dot(kalman_gain, innovation)
        # We need the size of the matrix so we can make an identity matrix.
        size = self.current_prob_estimate.shape[0]
        # eye(n) = nxn identity matrix.
        self.current_prob_estimate = np.dot((np.eye(size)-np.dot(kalman_gain,self.H)), self.current_prob_estimate)

class StatePredictionNode:
    """ This class contains the methods for state prediction """
    def __init__(self):

        rospy.init_node('odometry', anonymous=True)
        self.control_voltages = np.zeros([2,1])
        self.vl = 0
        self.vr = 0
        
        Q = np.array([[.1, 0, 0, 0, 0],
              [0, .1, 0, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0]])
        R = np.eye(2) * .15
        starting_state = np.zeros([5,1])
        self.r = .15
        self.l = .55
        self.ekf = ExtendedWMRKalmanFilter(self.r, self.l, Q, R, 2.3364e-04, -.65, 2.0624e-04, -.6189, starting_state, .05)
        self.pub = rospy.Publisher('odometry', Odometry, queue_size=1)
        rospy.Subscriber("/imu1", Imu, self._imu1Callback)
        rospy.Subscriber("/imu2", Imu, self._imu2Callback)
        rospy.Subscriber("left_voltage_pwm", Int16, self._leftMotorCallback) 
        rospy.Subscriber("right_voltage_pwm", Int16, self._rightMotorCallback)

        self.f = open('ekf_data.csv', 'w')
        
    def _leftMotorCallback(self, data):
        self.control_voltages[0] = data.data
        
    def _rightMotorCallback(self, data):
        self.control_voltages[1] = data.data
        
    def _imu1Callback(self, data):
        self.vl = data.angular_velocity.z
        if abs(self.vl) < .1:
            self.vl = 0
        
    def _imu2Callback(self, data):
        self.vr = data.angular_velocity.z
        if abs(self.vr) < .1:
            self.vr = 0        

    def _writeToFile(self, state, measurement):
        self.f.write(str(state[0,0]) + ',' + str(state[1,0]) + ','+  str(state[2,0]) +  ',' + str(state[3,0]) +  ',' + str(state[4,0]) + ',' + str(measurement[0,0]) + ',' + str(measurement[1,0])  +  '\n') 

    def loop(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if abs(self.vl) > .05 and abs(self.vr) > .05:
                print self.ekf.current_state_estimate

            ## USE KALMAN FILTER
            self.ekf.predict(self.control_voltages)
            self.ekf.measurement_update(np.array([[self.vl], [self.vr]]))
            state = copy.deepcopy(self.ekf.current_state_estimate)
            self._writeToFile(state, np.array([[self.vl], [self.vr]]))
            ## BUILD AND SEND MSG
            quaternion = tf.transformations.quaternion_from_euler(0, 0, state[4])
            
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.pose.pose.position.x = state[2]
            msg.pose.pose.position.y = state[3]
            msg.pose.pose.position.z = 0
            msg.pose.pose.orientation.x = quaternion[0]
            msg.pose.pose.orientation.y = quaternion[1]
            msg.pose.pose.orientation.z = quaternion[2]
            msg.pose.pose.orientation.w = quaternion[3]            
            
            msg.twist.twist.linear.x = (self.r/2)*(state[0]\
                + state[1])*math.cos(state[4])
            msg.twist.twist.linear.y = (self.r/2)*(state[0]\
                + state[1])*math.sin(state[4])
            msg.twist.twist.linear.z = 0
            msg.twist.twist.angular.z = (self.r/self.l)*(state[1]\
                - state[0])
            
            self.pub.publish(msg)
            r.sleep()

my_node = StatePredictionNode()
my_node.loop()
