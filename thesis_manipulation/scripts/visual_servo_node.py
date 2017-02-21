#!/usr/bin/env python
"""
Created on Tue Feb 14 10:56:59 2017

@author: Andrew Cunningham
"""

import numpy as np
import rospy
import baxter_interface
from nav_msgs.msg import Odometry
from baxter_pykdl import baxter_kinematics
import math
import tf
import copy

from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    PoseWithCovariance,
)

# convert vector to cross product matrix form
def hat(k):
    khat = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return khat

# returns jacobian relating angular velocity to quaternion velocity    
def quatjacobian(quat):
    I = np.identity(3)
    J = 0.5 * np.concatenate((quat[1:4].T, quat[0]*I - hat(quat[1:4])), axis = 0)
    return J
    

class VisualServoNode:
    """ Node responsible for driving the arm to tags for manipulation """
    def __init__(self):
        rospy.init_node('servo_node', anonymous=True)
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
        
        ## MOVE ARM TO START
        limb = baxter_interface.Limb('right')
        angles = limb.joint_angles()
        
        angles['right_s0']=-0.3148495567134812
        angles['right_s1']=0.35051461003181705
        angles['right_e0']=1.2755050251267215
        angles['right_e1']=2.5648158773444116
        angles['right_w0']=-0.4306651061988299
        angles['right_w1']=-1.333213797491471
        angles['right_w2']=-1.67180584824316633
        limb.move_to_joint_positions(angles, timeout=10.0)
        
        pose = limb.endpoint_pose()
        quat = pose['orientation']
        
        self.t_desired_dict = {}    # WHERE WE WANT THE ARM TO GO
        self.r_desired_dict = {}    # THE ORIENTATION OF THE ARM
    
        self.t_desired_dict['right']= np.array([[ 0.83728373, -0.39381423,  0.19123955]])
        self.r_desired_dict['right'] = [quat.w, quat.x, quat.y, quat.z]
        
        self.t_desired_dict['left']= np.array([[ 0.83728373, -0.09381423,  0.19123955]])
        self.r_desired_dict['left'] = np.array([[-0.0228398,  -0.16423468, -0.98615684],\
        [ 0.0376087, 0.9855748,  -0.16500878], \
        [ 0.9990315,  -0.04085684, -0.01633368]])
        
        ## CONTROL VARIABLES
        self.fridge_state = 0
        
        # ENVIRONMENT INFO        
        self.fridge_handle_t = []
              
    def _ik_move(self, pose_desired, arm = 'right'):
        """ Move robot arm to pose_desired (if possible) """
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        
        pose_stamped = PoseStamped(
            header=hdr,
            pose=pose_desired,
        )
            
        print "IK MOVING"
        ns = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')      
        
        ikreq.pose_stamp.append(pose_stamped)
        
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1
            
        if (resp.isValid[0]):
            print("SUCCESS - Valid Joint Solution Found:")
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            self.limb_dict[arm].move_to_joint_positions(limb_joints)
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
        
    def _move_arm_towards(self, arm, velocity, orientation = None):
        """ Move the arm in the desired direction while maintaining orientation
        @arm string, either 'l' or 'r'
        @orientation orientation in quaternion TODO
        @velocity *LINEAR" direction to travel in (numpy 3x1)"""
        
        if not orientation == None:
            self.r_desired_dict[arm] = orientation
        
        maxJointVelocity = .2               
        J=self.kin_dict[arm].jacobian()
        
        current_pose = self.limb_dict[arm].endpoint_pose()
        quat = current_pose['orientation']
        q_current = [quat.x, quat.y, quat.z, quat.w]

        ## COMPUTE ROTATION VELOCITIES
        e = tf.transformations.quaternion_multiply(self.r_desired_dict[arm], tf.transformations.quaternion_conjugate(q_current))
        e = e.reshape([1,4])
        Jq = quatjacobian(np.transpose(e))    
        errorW=-10*np.dot(np.transpose(Jq),e.transpose())
        print errorW
        
        ## PERFORM TASK SPACE CONTROL
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
 
        current_pose = self.limb_dict['right'].endpoint_pose()
        current_t = np.array([[ current_pose['position'].x, current_pose['position'].y, current_pose['position'].z]])
        
        ## USE IK TO POSITION IN FRONT OF TAG
        if self.fridge_state == 0:
            fridge_handle_t, fridge_handle_rq = self.listener.lookupTransform('base', 'ar_marker_0', rospy.Time(0))
            self.fridge_handle_t = np.asarray(fridge_handle_t)
            
            euler = tf.transformations.euler_from_quaternion(fridge_handle_rq)
            euler = list(euler)
            euler[2] += math.pi
            self.gripper_r = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
            
            offset_ar = np.array([0,0,.3, 1]).reshape([4,1]) # offset in ar frame
            H_ar_to_base = self.listener.asMatrix('base', Header(stamp=rospy.Time(0), frame_id='ar_marker_0'))
            offset_base = np.dot(H_ar_to_base, offset_ar)
            
            desired_pose = Pose(
                position=Point(
                    x=offset_base[0],
                    y=offset_base[1],
                    z=offset_base[2],
                ),
                orientation=Quaternion(
                    x=self.gripper_r[0],
                    y=self.gripper_r[1],
                    z=self.gripper_r[2],
                    w=self.gripper_r[3],
                )
            )
            try:
                self._ik_move(desired_pose)
                self.fridge_state+=1
            except:
                print "Unable to move"
        
        ## ALIGN
        # SET Y AND Z POSITIONS APPROPRIATELY
        if self.fridge_state == 1:
            fridge_handle_t, fridge_handle_rq = self.listener.lookupTransform('base', 'ar_marker_0', rospy.Time(0))
            self.fridge_handle_t = np.asarray(fridge_handle_t)
            self.fridge_handle_t += np.array([0,-.02,-.16])
            
            euler = tf.transformations.euler_from_quaternion(fridge_handle_rq)
            euler = list(euler)
            euler[2] += math.pi
            self.fridge_handle_r = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])            
            
            if np.linalg.norm(np.multiply(current_t-self.fridge_handle_t, np.array([0, 1, 1]))) < .1:
                self.fridge_state +=1
                return
                
            velocity = -np.multiply(current_t-self.fridge_handle_t, 5*np.array([0, 1, 1]))
            print velocity
            self._move_arm_towards('right', np.transpose(velocity), self.gripper_r)
            print "PANNING"

        ## GO FORWARD
        elif self.fridge_state == 2:    
            self.gripper_dict['right'].open()
            if np.linalg.norm(np.multiply(current_t-self.fridge_handle_t, np.array([1, 2, 2]))) < .125:     
                self.fridge_state +=1
                return
                
            velocity = -np.multiply(current_t-self.fridge_handle_t, 5*np.array([1, 2, 2]))
            self._move_arm_towards('right', np.transpose(velocity))
            print np.linalg.norm(np.multiply(current_t-self.fridge_handle_t, np.array([1, 2, 2])))
            print "GOING FORWARD"
        
        ## GRAB HANDLE
        elif self.fridge_state == 3:
            self.gripper_dict['right'].close()
            self.fridge_state += 1
            print "GRABBING HANDLE"
            
        elif self.fridge_state == 4:
            open_fridge_t = np.array([[ 0.7728373, -0.19381423,  0.19123955]])
            if np.linalg.norm(np.multiply(current_t-open_fridge_t, np.array([1, 2, 2]))) < .05:     
                self.fridge_state +=1
                return
                
            velocity = -np.multiply(current_t-open_fridge_t, 5*np.array([1, 2, 2]))
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
            if i > 600:
                return
            self._open_fridge()
            r.sleep()

def main():
    my_servo_node = VisualServoNode()
    my_servo_node.spin()
            
if __name__ == '__main__':
    main()
