#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
import numpy as np

# Wiimote 1 should correspond to the left wheel
# Wiimote 2 should correspond to the right wheel

class JammsterModel:
    """contains the state of the base and methods for updating position, implemented for wiimote"""
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

        self.prevLeftRotationalPosition = 0
        self.prevRightRotationalPosition = 0

        self.currentLeftRotationalPosition = 0
        self.currentRightRotationalPosition = 0

        # make sure both imus have sent
        self.rightWheelUpdated = False
        self.leftWheelUpdated = False

        # check this member variable to see if odometry information is send-able
        self.readyToPublish = False

        self.L = .57
        self.wheelCircumfrence = .9424;

        self.initalized = False;

    def updateState(self):

        # dont update the odometry until both wiimotes have gotten an update
        if not (self.leftWheelUpdated and self.rightWheelUpdated):
            return

        # set initial wheel angle
        if not self.initalized:
            self.prevLeftRotationalPosition = self.currentLeftRotationalPosition
            self.prevRightRotationalPosition = self.currentRightRotationalPosition
            self.initalized = True

        distanceRight = deltaAngle(self.currentRightRotationalPosition, self.prevRightRotationalPosition) / 6.283 * self.wheelCircumfrence
        distanceLeft = deltaAngle(self.currentLeftRotationalPosition, self.prevLeftRotationalPosition) / 6.283* self.wheelCircumfrence
        distanceCenter = (distanceLeft + distanceRight)/2

        self.x = self.x + distanceCenter*math.cos(self.theta)
        self.y = self.y + distanceCenter*math.sin(self.theta)
        self.theta = self.theta +  (distanceRight - distanceLeft)/self.L

        self.prevRightRotationalPosition = self.currentRightRotationalPosition
        self.prevLeftRotationalPosition = self.currentLeftRotationalPosition
        self.readyToPublish = True
        return

    def published(self):
        self.readyToPublish = False
        robot.leftWheelUpdated = False
        robot.rightWheelUpdated = False


robot = JammsterModel()


def deltaAngle(x, y):
    return math.atan2(math.sin(x-y), math.cos(x-y))

def wiimote1Callback(data):
    robot.currentLeftRotationalPosition=math.atan2(data.linear_acceleration.x, data.linear_acceleration.z)
    robot.leftWheelUpdated = True

def wiimote2Callback(data):
    robot.currentRightRotationalPosition=math.atan2(data.linear_acceleration.x, data.linear_acceleration.z)
    robot.rightWheelUpdated = True

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/leftWheel/imu", Imu, wiimote1Callback)
    rospy.Subscriber("/rightWheel/imu", Imu, wiimote2Callback)

    odomPub = rospy.Publisher("/base_controller/odom", Odometry)

    odom = Odometry()

    while 1:
        br = tf.TransformBroadcaster()
        time.sleep(.05)

        robot.updateState()
        if robot.readyToPublish:

	    quaternion = tf.transformations.quaternion_from_euler(0, 0, robot.theta)
            #tf msg
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),"/odom", "/map")
            br.sendTransform((robot.x, robot.y, 0), tf.transformations.quaternion_from_euler(0, 0, robot.theta),rospy.Time.now(),"/base_link", "/odom")
            br.sendTransform((0, 0, .7), tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),"/base_laser_link", "/base_link")
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),"/camera_link", "/base_laser_link")

            robot.readyToPublish = False

            #odometry msg
            odom.header.stamp = rospy.Time.now()
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = robot.x
            odom.pose.pose.position.y = robot.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation.x = quaternion[0]
	    odom.pose.pose.orientation.y = quaternion[1]
	    odom.pose.pose.orientation.z = quaternion[2]
	    odom.pose.pose.orientation.w = quaternion[3]
	    P = [.0001, 0, 0, 0, 0, 0,
                 0, .0001, 0, 0, 0, 0,
                 0, 0, .0001, 0, 0, 0,
		 0, 0, 0, .0001, 0, 0,
                 0, 0, 0, 0, .0001, 0,
                 0, 0, 0, 0, 0, .0001]
	    odom.pose.covariance = P
            odomPub.publish(odom)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
