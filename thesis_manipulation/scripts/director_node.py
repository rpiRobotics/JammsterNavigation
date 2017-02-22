# -*- coding: utf-8 -*-
"""
Created on Wed Feb 22 09:38:41 2017

@author: Andrew Cunningham
"""
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
import std_msgs.msg
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

class DirectorNode:
    def __init__(self):
        rospy.init_node('director_node')
        self.base_goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.manipulation_pub = rospy.Publisher('manipulation_state', std_msgs.msg.String, queue_size=1)
        
        rospy.Subscriber('move_base/status', GoalStatusArray, self._action_server_callback)
        
        self.at_goal = False
        
    def send_fridge_goal(self):
        """ Send move_base a pose goal to go to the fridge using action server """
        goal_action = MoveBaseActionGoal()
        goal = PoseStamped()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now() 
        pose = Pose()
        goal.header = header
        
        goal.pose = Pose(
            position=Point(
                x=100.236801147,
                y=99.8681869507,
                z=0.0
            ),
            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=-0.491244894362,
                w=0.871021500173
            )
        )
        goal_action.header = header
        goal_action.goal.target_pose = goal
        self.base_goal_pub.publish(goal_action)
        
    def _action_server_callback(self, data):
        if GoalStatusArray.status_list[0] > 1:
            self.at_goal = True
            
    def open_fridge_command(self):
        command = std_msgs.msg.String('open_fridge')
        self.manipulation_pub.publish(command)
            
    def spin(self):
        self.send_fridge_goal()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()
            print 'Waiting for goal'
            if self.at_goal:
                self.open_fridge_command()
            
my_node = DirectorNode()
my_node.spin()