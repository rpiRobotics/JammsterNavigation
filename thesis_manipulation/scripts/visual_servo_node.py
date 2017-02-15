# -*- coding: utf-8 -*-
"""
Created on Tue Feb 14 10:56:59 2017

@author: Andrew Cunningham
"""

import numpy as np
import rospy
import baxter_interface
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from baxter_pykdl import baxter_kinematics
import math
import tf
import copy

# convert vector to cross product matrix form
def hat(k):
    khat = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return khat

# returns jacobian relating angular velocity to quaternion velocity    
def quatjacobian(q):
    I = np.identity(3)
    J = 0.5 * np.concatenate((q[1:4].T, q[0]*I - hat(q[1:4])), axis = 0)
    return J
    
# convert 3 x 3 rotation matrix to quaternion
def R2q(R):
    tr = np.trace(R)
    if tr > 0:
        S = 2*math.sqrt(tr + 1)
        q = np.array([[0.25*S], \
                      [(R[2,1] - R[1,2]) / S], \
                      [(R[0,2] - R[2,0]) / S], \
                      [(R[1,0] - R[0,1]) / S]])
                      
    elif (R[0,0] > R[1,1] and R[0,0] > R[2,2]):
        S = 2*math.sqrt(1 + R[0,0] - R[1,1] - R[2,2])
        q = np.array([[(R[2,1] - R[1,2]) / S], \
                      [0.25*S], \
                      [(R[0,1] + R[1,0]) / S], \
                      [(R[0,2] + R[2,0]) / S]])
    elif (R[1,1] > R[2,2]):
        S = 2*math.sqrt(1 - R[0,0] + R[1,1] - R[2,2])
        q = np.array([[(R[0,2] - R[2,0]) / S], \
                      [(R[0,1] + R[1,0]) / S], \
                      [0.25*S], \
                      [(R[1,2] + R[2,1]) / S]])
    else:
        S = 2*math.sqrt(1 - R[0,0] - R[1,1] + R[2,2])
        q = np.array([[(R[1,0] - R[0,1]) / S], \
                      [(R[0,2] + R[2,0]) / S], \
                      [(R[1,2] + R[2,1]) / S], \
                      [0.25*S]])
    return q

def getTransform(listener, frame1, frame2):
    try:
        position, quaternion= listener.lookupTransform(frame1, frame2, rospy.Time(0))
        homogenousMatrix= listener.fromTranslationRotation(position, quaternion)
        
    except:
        print "couldnt transform"
        return (-1,-1)
        
    rotation = homogenousMatrix[0:3,0:3]
    translation = homogenousMatrix[0:3,3]
    return rotation, translation

class VisualServoNode:
    """ Node responsible for driving the arm to tags for manipulation """
    def __init__(self):
        rospy.init_node('servo_node', anonymous=True)
        rospy.Subscriber('/odometry', Odometry, self._odom_callback)
        self.listener= tf.TransformListener()
             
        self.robot_pose = PoseWithCovariance() # where we think our robot is
        
        ## ROBOT CONTROL OBJECTS
        self.limb_dict = {}
        self.gripper_dict = {}
        self.kin_dict = {}
        
        self.limb_dict['right'] = baxter_interface.Limb('right')
        self.kin_dict['right'] = baxter_kinematics('right')
        self.gripper_dict['right'] = baxter_interface.Gripper('right')
        self.gripper_dict['right'].calibrate()
        
        self.limb_dict['left'] = baxter_interface.Limb('left')
        self.kin_dict['left'] = baxter_kinematics('left')
        #self.gripper_dict['left'] = baxter_interface.Gripper('left')
        #self.gripper_dict['left'].calibrate()
        
#        Rtest,Ttest=getTransform(self.listener,'right_hand','base')
#        print Rtest
#        exit()
        
        self.t_desired_dict = {}    # WHERE WE WANT THE ARM TO GO
        self.r_desired_dict = {}    # THE ORIENTATION OF THE ARM
        
        self.t_desired_dict['right']= np.array([[ 0.83728373, -0.39381423,  0.19123955]])
        self.r_desired_dict['right'] = np.array([[ -5.40531349e-02,  -9.93651820e-01,   9.86626584e-02],\
        [ -9.00945180e-04,  -9.87585367e-02, -9.95111019e-01],\
        [  9.98537654e-01,  -5.38777598e-02,   4.44298269e-03]])
        
        self.t_desired_dict['left']= np.array([[ 0.83728373, -0.09381423,  0.19123955]])
        self.r_desired_dict['left'] = np.array([[-0.0228398,  -0.16423468, -0.98615684],\
        [ 0.0376087, 0.9855748,  -0.16500878], \
        [ 0.9990315,  -0.04085684, -0.01633368]])
        
        ## CONTROL VARIABLES
        self.fridge_state = 0
        
        limb = baxter_interface.Limb('right')
        angles = limb.joint_angles()
        
        angles['right_s0']=-0.3148495567134812
        angles['right_s1']=0.35051461003181705
        angles['right_e0']=1.2755050251267215
        angles['right_e1']=2.5648158773444116
        angles['right_w0']=-0.4306651061988299
        angles['right_w1']=-1.533213797491471
        angles['right_w2']=-3.17180584824316633
        limb.move_to_joint_positions(angles, timeout=5.0)

        # ENVIRONMENT INFO        
        self.fridge_handle_t = []
        
    def _odom_callback(self, data):
        """ Update our position belief """
        self.robot_pose = data.pose
        return
        
    def _move_arm_towards(self, arm, velocity, orientation = None):
        """ Move the arm in the desired direction while maintaining orientation
        @arm string, either 'l' or 'r'
        @orientation orientation in quaternion TODO
        @velocity *LINEAR" direction to travel in (numpy 3x1)"""
        
        R,T=getTransform(self.listener,'base',arm+'_hand')
        R_current,T_current=getTransform(self.listener,arm+'_hand','base')
        
        maxJointVelocity = .2               
        J=self.kin_dict[arm].jacobian()
        
        e = R2q(np.dot(np.transpose(self.r_desired_dict[arm]),R_current))
        
        Jq = quatjacobian(e)
       
        errorW=np.dot(np.transpose(Jq),e)
        
        desiredV = np.concatenate((velocity,errorW),axis = 0)
        jacobianInv=self.kin_dict[arm].jacobian_pseudo_inverse() 
        jointVelocities = np.dot(jacobianInv,desiredV)     
        
        # SAFETY!
        if abs(np.max(jointVelocities)) > maxJointVelocity:
            jointVelocities=jointVelocities/(float(np.max(jointVelocities)/maxJointVelocity)) 
        
        angular_velocity_dict = self.limb_dict[arm].joint_angles()
        jointVelocities=np.clip(jointVelocities,-maxJointVelocity,maxJointVelocity)
        angular_velocity_dict[arm+'_s0']=jointVelocities[0,0]
        angular_velocity_dict[arm+'_s1']=jointVelocities[1,0]
        angular_velocity_dict[arm+'_e0']=jointVelocities[2,0]
        angular_velocity_dict[arm+'_e1']=jointVelocities[3,0]
        angular_velocity_dict[arm+'_w0']=jointVelocities[4,0]
        angular_velocity_dict[arm+'_w1']=jointVelocities[5,0]
        angular_velocity_dict[arm+'_w2']=jointVelocities[6,0]
        
        #print angular_velocity_dict
        self.limb_dict[arm].set_joint_velocities(angular_velocity_dict)
        
    def _open_fridge(self):
        """ State for attempting to open the fridge
        THE PLAN: ALIGN SELF WITH TAG THEN GO FORWARD"""
        if self.fridge_state == 0:
            fridge_handle_R,self.fridge_handle_t=getTransform(self.listener,'base','ar_marker_0_ref')
            
        current_pose = self.limb_dict['right'].endpoint_pose()
        current_t = np.array([[ current_pose['position'].x, current_pose['position'].y, current_pose['position'].z]])
        
        ## ALIGN
        # SET Y AND Z POSITIONS APPROPRIATELY
        if self.fridge_state == 0:
            if np.linalg.norm(np.multiply(current_t-self.fridge_handle_t, np.array([0, 1, 1]))) < .1:
                self.fridge_state +=1
                return
                
            velocity = -np.multiply(current_t-self.fridge_handle_t, np.array([0, 1, 1]))
            self._move_arm_towards('right', np.transpose(velocity))
            print "PANNING"

        ## GO FORWARD
        elif self.fridge_state == 1:    
            self.gripper_dict['right'].open()
            if np.linalg.norm(np.multiply(current_t-self.fridge_handle_t, np.array([1, 2, 2]))) < .12:     
                self.fridge_state +=1
                return
                
            velocity = -np.multiply(current_t-self.fridge_handle_t, np.array([1, 2, 2]))
            self._move_arm_towards('right', np.transpose(velocity))
            print "GOING FORWARD"
        
        ## GRAB HANDLE
        elif self.fridge_state == 2:
            self.gripper_dict['right'].close()
            self.fridge_state += 1
            print "GRABBING HANDLE"
            
        elif self.fridge_state == 3:
            open_fridge_t = np.array([[ 0.7728373, -0.19381423,  0.19123955]])
            if np.linalg.norm(np.multiply(current_t-open_fridge_t, np.array([1, 2, 2]))) < .05:     
                self.fridge_state +=1
                return
                
            velocity = -np.multiply(current_t-open_fridge_t, np.array([1, 2, 2]))
            print np.linalg.norm(np.multiply(current_t-open_fridge_t, np.array([1, 2, 2])))
            self._move_arm_towards('right', np.transpose(velocity))
            print "OPENING"
            
        return
        
    def _grab_food(self):
        return
        
    def _close_fridge(self):
        return
        
    def _execute_command(self):
        return
        
    def spin(self):
        r = rospy.Rate(10)
        i = 0
        while not rospy.is_shutdown():
            i+=1
            if i > 200:
                return
            self._open_fridge()
            r.sleep()

def main():
    my_servo_node = VisualServoNode()
    my_servo_node.spin()
            
if __name__ == '__main__':
    main()