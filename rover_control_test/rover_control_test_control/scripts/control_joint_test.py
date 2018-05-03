#! /usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64

rospy.init_node('yuan_test')
pub1 = rospy.Publisher('/rover_control_test/rear_right_joint_position_controller/command', Float64, queue_size=1)
pub2 = rospy.Publisher('/rover_control_test/rear_left_joint_position_controller/command', Float64, queue_size=1)
pub3 = rospy.Publisher('/rover_control_test/front_right_joint_position_controller/command', Float64, queue_size=1)
pub4 = rospy.Publisher('/rover_control_test/front_left_joint_position_controller/command', Float64, queue_size=1)
rate = rospy.Rate(2)
count = 0
sin_yuan=math.sin(count)

while not rospy.is_shutdown():
    pub1.publish(count)
    pub2.publish(count)
    pub3.publish(count)
    pub4.publish(count)
    count+=1
    sin_yuan=math.sin(count)
    rate.sleep()