#!/usr/bin/env python
"""
Description: Used to collect data points on AR tags to fit uncertainty model
             Run test with 11cm tags
"""


import numpy as np
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf

class AR_Recorder_Node():
    def __init__(self, file_name):
        rospy.init_node('ar_recorder', anonymous=True)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self._arCallback )
        self.f = open(file_name, 'w+')
        
    def _arCallback(self, data):
        for marker in data.markers:
            print "Recording stuff"
            x = marker.pose.pose.position.x
            y = marker.pose.pose.position.y
            z = marker.pose.pose.position.z
            quat = marker.pose.pose.orientation
            quaternion = (
                quat.x,
                quat.y,
                quat.z,
                quat.w)
            angles = tf.transformations.euler_from_quaternion(quaternion)
            self.f.write(str(x) + ',' + str(y) + ',' + str(z) + ',' + str(angles[0]) + ',' + str(angles[1]) + ',' + str(angles[2]) + '\n')
        return
        
    def spin(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            r.sleep()
        
my_node = AR_Recorder_Node('ar_data.csv')
my_node.spin()