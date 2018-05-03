#! /usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64

rospy.init_node('rover_final_test')
pub1 = rospy.Publisher('/rover_final/front_right_joint_position_controller/command', Float64, queue_size=1)
pub1 = rospy.Publisher('/rover_final/front_left_joint_position_controller/command', Float64, queue_size=1)
rate = rospy.Rate(10) #10hz
count = 0

while not rospy.is_shutdown():
    pub1.publish(count)
    pub1.publish(count)
    count-=0.5
    rate.sleep()