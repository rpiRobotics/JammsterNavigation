#!/usr/bin/env python
import numpy as np
import math
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
from std_msgs.msg import Int16
import copy
from tf import TransformListener
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
import transformations

class ARTAG_landmark:
    """ Class used to represent an ARTAG landmark """
    def __init__(self, tag_num, slam_id, x, y, z, theta_x, theta_y, theta_z, fixed = True):
        self.tag_num = tag_num  # number the AR tag encodes
        self.slam_id = slam_id  # position in the state vector
        self.fixed = fixed      # whether or not the tag's position can be updated (AKA SLAM)
        self.x = x
        self.y = y
        self.z = z
        self.theta_x = theta_x
        self.theta_y = theta_y
        self.theta_z = theta_z
        
class ExtendedWMRKalmanFilter:
    """ Class to predict a wheeled mobile robot state [dtheta_l, dtheta_r, v, w, x,y,theta]
        from imu measurements on wheels and base.  Will include SLAM based stuff later"""
    def __init__(self, r, l, Q, R, k1_l, k2_l, k1_r, k2_r, starting_state, dt):
        """ @r radius of wheels
            @l length of axle
            @Q covariance of process noise
            @R covariance of measurement noise
            @starting_state [dtheta_l, dtheta_r, v, w, x,y,theta]
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
        #self.current_prob_estimate = np.zeros([starting_state.shape[0], starting_state.shape[0]])
        self.current_prob_estimate = np.eye(starting_state.shape[0])*.4
        self.dt = dt
        
        self.A = np.zeros([self.current_state_estimate.shape[0], self.current_state_estimate.shape[0]])
        self.B = np.eye(2)
        self.H = np.array([[1,0,0,0,0,0,0],\
                           [0,1,0,0,0,0,0],\
                           [0,0,0,1,0,0,0]])
        
    def predict(self, u):
        """ Predict new state from old state and command
        @u [v_l v_r], Step without sensing """

        dtheta_l = self.current_state_estimate[0,0]
        dtheta_r = self.current_state_estimate[1,0]
        v = self.current_state_estimate[2,0]
        w = self.current_state_estimate[3,0]
        x = self.current_state_estimate[4,0]
        y = self.current_state_estimate[5,0]
        theta = self.current_state_estimate[6,0]
        vl = u[0,0]
        vr = u[1,0]
        
        ## TRANSITION ESTIMATE
        self.current_state_estimate[0] += self.dt*(self.k2_l*dtheta_l + self.k1_l*vl)
        self.current_state_estimate[1] += self.dt*(self.k2_r*dtheta_r + self.k1_r*vr)
        self.current_state_estimate[2] = self.r/2 * (self.current_state_estimate[0] + self.current_state_estimate[1])
        self.current_state_estimate[3] = self.r/self.l * (self.current_state_estimate[1] - self.current_state_estimate[0])
        self.current_state_estimate[4] += (self.dt)*self.current_state_estimate[2]*math.cos(theta)
        self.current_state_estimate[5] += (self.dt)*self.current_state_estimate[2]*math.sin(theta)
        self.current_state_estimate[6] += (self.dt)*self.current_state_estimate[3]
                           
        ## TRANISTION PROBABILITY
        # only propogate the things that can move
        self.A[0:7, 0:7] = np.array([[1+self.dt*self.k2_l, 0, 0, 0, 0, 0, 0],\
                           [0, 1+self.dt*self.k2_r, 0, 0, 0, 0, 0],\
                           [self.r/2, self.r/2, 0, 0, 0, 0, 0],\
                           [-self.r/self.l, self.r/self.l, 0, 0, 0, 0, 0],\
                           [0, 0, math.cos(theta)*self.dt , 0, 1, 0, -v*math.sin(theta)*self.dt],\
                           [0, 0, math.sin(theta)*self.dt , 0, 0, 1, v*math.cos(theta)*self.dt],\
                           [0, 0, 0, self.dt, 0, 0, 1]])
                        
        self.current_prob_estimate = np.dot(np.dot(self.A, self.current_prob_estimate), np.transpose(self.A)) + self.Q

    def linear_measurement_update(self, measurement_vector, H='null', R = 'null'):
        """ Update Kalman filter if sensing information is received """
        if not H == 'null':
            self.H = H

        if not R == 'null':
            self.R = R
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

    def add_AR_tag(self, ar_obj, certainty):
        """ Add an AR tag to the state of the EKF
        @ar_obj ARTAG_landmark object
        @certainty np.eye(4) * scale for how sure we are of the state of the tag"""
        transform = np.zeros([3,1])
        transform[0] = ar_obj.x
        transform[1] = ar_obj.y
        transform[2] = ar_obj.theta_z
        self.current_state_estimate = np.vstack((self.current_state_estimate, transform))
        self.A = np.zeros([self.current_state_estimate.shape[0], self.current_state_estimate.shape[0]])
        new_prob_estimate = np.zeros([self.current_state_estimate.shape[0], self.current_state_estimate.shape[0]])
        new_prob_estimate[0:self.current_prob_estimate.shape[0], 0:self.current_prob_estimate.shape[0]] = self.current_prob_estimate
        new_prob_estimate[self.current_prob_estimate.shape[0]:, self.current_prob_estimate.shape[0]:] = certainty
        
        new_Q = np.zeros([self.current_state_estimate.shape[0], self.current_state_estimate.shape[0]])
        new_Q[0:self.current_state_estimate.shape[0]-3, 0:self.current_state_estimate.shape[0]-3] = self.Q
        self.Q = new_Q
        
        new_prob_est = np.zeros([self.current_state_estimate.shape[0], self.current_state_estimate.shape[0]])
        new_prob_est[0:self.current_state_estimate.shape[0]-3, 0:self.current_state_estimate.shape[0]-3] = self.current_prob_estimate
        self.current_prob_estimate = new_prob_est

class StatePredictionNode:
    """ This class contains the methods for state prediction """
    def __init__(self):

        rospy.init_node('odometry', anonymous=True)
        self.dt = .02
        self.control_voltages = np.zeros([2,1])
        self.vl = 0     # left wheel velocity
        self.vr = 0     # right wheel velocity
        self.vb = 0     # base angular velocity

        self.base_imu_ready = False # base imu is slower than the other IMU's
                                    # we can't update base velocity all the time

        Q = np.zeros([7,7])
        Q[0,0] = .01
        Q[1,1] = .01
        R = np.eye(3) * .001
        starting_state = np.zeros([7,1])
        self.r = .15
        self.l = .55
        self.ekf = ExtendedWMRKalmanFilter(self.r, self.l, Q, R, 2.3364e-04, -.65, 2.3364e-04, -.65, starting_state, self.dt)
        self.listener = TransformListener()


        self.i = 0 # print index

        self.pub = rospy.Publisher('odometry', Odometry, queue_size=1)
        rospy.Subscriber("/imu1", Imu, self._imu1Callback)
        rospy.Subscriber("/imu2", Imu, self._imu2Callback)
        rospy.Subscriber("/imu3", Imu, self._imu3Callback)
        rospy.Subscriber("left_voltage_pwm", Int16, self._leftMotorCallback) 
        rospy.Subscriber("right_voltage_pwm", Int16, self._rightMotorCallback)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self._arCallback )
        
        self.imus_observed_list = []
        self.ar_observed_list = []
        self.sensed_ar_diff = {}  # map from tag_id sensed to array of np.array([dx, dy, dtheta_z])
        self.landmark_map = {} # map from id to AR TAG obj

        ## ADD PREDEFINED MAPS
        #(tag_num, slam_id, x, y, z, theta_x, theta_y, theta_z, fixed = True)
        self.landmark_map[0] = ARTAG_landmark(0,1,0,1.37,.10,math.pi/2, 0, 0)
        self.landmark_map[1] = ARTAG_landmark(1,2,-1.74,0,.15,math.pi/2, 0, math.pi/2)
        self.landmark_map[3] = ARTAG_landmark(3,3,2.84,.15,.32,math.pi/2, 0, -math.pi/2)
        self.landmark_map[2] = ARTAG_landmark(2,4,1.37,1.88,.06,math.pi/2, 0, 0)
        self.ekf.add_AR_tag(self.landmark_map[0], np.zeros([3,3]))
        self.ekf.add_AR_tag(self.landmark_map[1], np.zeros([3,3]))
        self.ekf.add_AR_tag(self.landmark_map[2], np.zeros([3,3]))
        self.ekf.add_AR_tag(self.landmark_map[3], np.zeros([3,3]))
        
        self.br = tf.TransformBroadcaster()
        
    def _leftMotorCallback(self, data):
        self.control_voltages[0] = data.data
        
    def _rightMotorCallback(self, data):
        self.control_voltages[1] = data.data
        
    def _imu1Callback(self, data):
        self.vl = -data.angular_velocity.z
        if abs(self.vl) < .1:
            self.vl = 0
            return
            
        self.imu_observed_list.append('imu1')
        
    def _imu2Callback(self, data):
        self.vr = -data.angular_velocity.z
        if abs(self.vr) < .1:
            self.vr = 0       
            return
            
        self.imu_observed_list.append('imu2')
            
    def _imu3Callback(self, data):
        self.vb = data.angular_velocity.z-.01
        if abs(self.vb) < .05:
            self.vb = 0   
            return
            
        self.imu_observed_list.append('imu3')
        
    def _arCallback(self, data):
        for marker in data.markers:
            if marker.id in self.landmark_map:
                
                tag_frame = 'ar_marker_'+str(marker.id)
                t = self.listener.getLatestCommonTime(tag_frame, 'base')
                p_tr, q_tr = self.listener.lookupTransform(tag_frame, 'base', t)
                H_tr = self.listener.fromTranslationRotation(p_tr, q_tr)
                
                t = self.listener.getLatestCommonTime(tag_frame+'_ref', 'world')
                p_ot, q_ot = self.listener.lookupTransform('world', tag_frame+'_ref',  t)
                H_ot = self.listener.fromTranslationRotation(p_ot, q_ot)
                
                H_ot = np.dot(H_ot, H_tr)    
                angles = transformations.euler_from_matrix(H_ot[0:3, 0:3], axes='sxyz')
                
                self.sensed_ar_diff[marker.id] = np.array([[H_ot[0,3]], [H_ot[1,3]], [angles[2]]])
                self.ar_observed_list.append(marker.id)  # we observed an AR tag!    
                            
        return
     
       
    def _publish_odom(self):
        state = copy.deepcopy(self.ekf.current_state_estimate)
            
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

    def _update_with_imus(self):
        if len(self.imus_observed_list) == 0:
            return
        
        # Initalize intermediary variables
        measurement_list = []
        H_list = []
        R_list = []
        num_states = self.ekf.current_state_estimate.shape[0]
        
        ## CHECK EACH SENSOR
        if 'imu1' in self.imus_observed_list:
            H_row = np.zeros([1, num_states])
            H_row[0, 0] = 1
            H_list.append(H_row)
            measurement_list.append(self.vl)
            R_list.append(.06)
            
        if 'imu2' in self.imus_observed_list:
            H_row = np.zeros([1, num_states])
            H_row[0, 1] = 1
            H_list.append(H_row)
            measurement_list.append(self.vr)
            R_list.append(.06)
            
        if 'imu3' in self.imus_observed_list:
            H_row = np.zeros([1, num_states])
            H_row[0, 3] = 1
            H_list.append(H_row)
            measurement_list.append(self.vb)
            R_list.append(4e-5)
    
        # BUILD H
        H = H_list[0]
        del H_list[0]
        for h_block in H_list:
            H = np.vstack((H, h_block))
            
        # BUILD MEASUREMENTS
        measurements = measurement_list[0]
        del measurement_list[0]
        for measurement in measurement_list:
            measurements = np.vstack((measurements, measurement))
            
        # BUILD R
        R = np.zeros([H.shape[0], H.shape[0]])
        r_index = 0 # element of diagonal to fill
        for r_elem in R_list:
            R[r_index, r_index] = r_elem
            r_index += 1
            
        # reset things
        self.imus_observed_list = []
        self.ekf.linear_measurement_update(measurement, H, R)
        
    def _update_with_landmarks(self):
        if len(self.ar_observed_list) == 0:
            return
                
        # CHECK ALL LANDMARKS
        for landmark_id in self.landmark_map.keys():
            if landmark_id in self.ar_observed_list:
                measurement = self.sensed_ar_diff[landmark_id]
                
                H = np.zeros([3, self.ekf.current_prob_estimate.shape[0]])
                H[:, 4:7] = np.eye(3)
                R = np.eye(3)* 4E-3
                
                self.ekf.linear_measurement_update(measurement, H, R)
    
        # reset things
        self.ar_observed_list = []
        self.sensed_ar_diff = {}
    
    def _publish_tf(self):
        # ROBOT TRANSFORM
        state = copy.deepcopy(self.ekf.current_state_estimate)
        self.br.sendTransform((state[4], state[5], 0),
                 tf.transformations.quaternion_from_euler(0, 0, state[6]),
                 rospy.Time.now(),
                 "base",
                 "world")
                 
        for landmark_id in self.landmark_map.keys():
            ar_obj = self.landmark_map[landmark_id]
            self.br.sendTransform((ar_obj.x, ar_obj.y, ar_obj.z),
                 tf.transformations.quaternion_from_euler(ar_obj.theta_x, ar_obj.theta_y, ar_obj.theta_z),
                 rospy.Time.now(),
                 "ar_marker_" + str(ar_obj.tag_num) + "_ref",
                 "world")
        return
    
    def _update_ekf(self):
        self.ekf.predict(self.control_voltages)
        self._update_with_imus()
        self._update_with_landmarks()
        
    def _print(self):
        """ Print info to the screen every once and a while """
        self.i += 1
        np.set_printoptions(precision=3, suppress=True)
        if self.i%40 == 0:
            self.i = 0
            print self.ekf.current_state_estimate[4:7]
            
    def spin(self):
        r = rospy.Rate(1/self.dt)
        while not rospy.is_shutdown():
            self._publish_tf()
            self._update_ekf()
            self._print()
            self._publish_odom()
            r.sleep()

my_node = StatePredictionNode()
my_node.spin()