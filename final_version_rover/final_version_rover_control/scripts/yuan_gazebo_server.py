#! /usr/bin/env python

from gazebo_msgs.srv import ApplyJointEffort
import rospy

rospy.init_node('yuan_gazebo_server')
rospy.wait_for_service('/gazebo/apply_joint_effort')
yuan_gazebo_server_client=rospy.ServiceProxy('/gazebo/apply_joint_effort',ApplyJointEffort)
start_t=rospy.Time(0)
duration_t=rospy.Duration(10, 0)
resp1=yuan_gazebo_server_client('front_left_joint',-10.0,start_t,duration_t)
resp2=yuan_gazebo_server_client('front_right_joint',-10.0,start_t,duration_t)



