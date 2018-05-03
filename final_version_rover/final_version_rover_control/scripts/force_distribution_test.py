#! /usr/bin/env python

# define some constant parameters
r = 0.140  # unit m
r_rear = 0.125
R = 0.150  # r+h
R_rear = 0.125
rs = 0.1465
b = 0.144
c1 = 0.5  # from Ding
c2 = 0  # from Ding
c3 = 0  # from Ding
c = 0.2  # fake
phy = 30.0  # fake unit:degree
K = 0.010  # fake unit:m
Ks = 2500000  # fake unit:Pa/m**N
nk0 = 0.75  # from Yang
nk1 = 0.2  # from Yang
RC0 = 0.4  #fake
RC1 = 0.2  #fake
m = 28.78 #kg
time_step = 0.01 #10hz
z0 = 0.0001
slop = 0.0
G=100

from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import GetLinkState
import geometry_msgs.msg 

import rospy
import math

rospy.init_node('force_distribution_test')
rospy.wait_for_service('/gazebo/apply_body_wrench')
rospy.wait_for_service('/gazebo/get_link_state')
client_rover_link=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
client_rover_wrench=rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)

def get_velocity():
    request_right=client_rover_link('front_right_wheel', 'world')
    request_left=client_rover_link('front_left_wheel', 'world')
    request_rear=client_rover_link('rear_wheel', 'world')
    vz_right=request_right.link_state.twist.linear.z
    print('++++++++++++++++++++++++vz_right')
    print(vz_right)
    vz_left=request_left.link_state.twist.linear.z
    print('++++++++++++++++++++++++vz_left')
    print(vz_left)
    vz_rear=request_rear.link_state.twist.linear.z
    print('++++++++++++++++++++++++vz_rear')
    print(vz_rear)    
    return {'vz_right':vz_right, 'vz_left':vz_left, 'vz_rear':vz_rear}

def get_sinkage():
    request_right=client_rover_link('front_right_wheel', 'world')
    request_left=client_rover_link('front_left_wheel', 'world')
    request_rear=client_rover_link('rear_wheel', 'world')
    sinkage_right=request_right.link_state.pose.position.z
    print('++++++++++++++++++++++++sinkage_right')
    print(2+rs-sinkage_right)
    sinkage_left=request_left.link_state.pose.position.z
    print('++++++++++++++++++++++++sinkage_left')
    print(2+rs-sinkage_left)
    sinkage_rear=request_rear.link_state.pose.position.z
    print('++++++++++++++++++++++++sinkage_rear')
    print(2+r_rear-sinkage_rear)    
    return {'z_right':2+rs-sinkage_right, 'z_left':2+rs-sinkage_left, 'z_rear':2+r_rear-sinkage_rear}

def set_force1():
    z_dict = get_sinkage()
    z_right=z_dict['z_right']
    z_left=z_dict['z_left']
    z_rear=z_dict['z_rear']    

    vz=get_velocity()
    vz_right=vz['vz_right']
    vz_left=vz['vz_left']
    vz_rear=vz['vz_rear']

    set_point_front_left = geometry_msgs.msg.Point()
    set_point_front_left.z = 0#-0.1465
    #set_point_front_left.x= -0.074  #will rotate 
    set_point_front_right = geometry_msgs.msg.Point()
    set_point_front_right.z = 0#-0.1465 
    #set_point_front_right.x = 0.074  
    set_point_rear = geometry_msgs.msg.Point()
    set_point_rear.z = 0#-0.125

    FN_right=100.0*z_right
    FN_left=100.0*z_left

    set_force_front_right = geometry_msgs.msg.Wrench()
    set_force_front_right.force.z = 20.48*6.5 - 21 * vz_right
    #set_force_front_right.force.z = FN_right - 21 * vz_right
 
    print('*******************front_right*******************up')
    print(set_force_front_right.force.z)
    print('*******************front_right*******************down')
    set_force_front_left = geometry_msgs.msg.Wrench()
    #set_force_front_left.force.z = FN_left - 21 * vz_left
    set_force_front_left.force.z = 20.48*6.5 - 21 * vz_left

    print('*******************front_left*******************up')
    print(set_force_front_left.force.z) 
    print('*******************front_left*******************down')   
    set_force_rear = geometry_msgs.msg.Wrench()
    set_force_rear.force.z = 6.0*6.5 - 21 * vz_rear
    print('*******************rear*******************up')
    print(set_force_rear.force.z)    
    print('*******************front_left*******************down')

    request_force1=client_rover_wrench('front_right_wheel','world',set_point_front_right,set_force_front_right,start_time,duration_time)
    request_force2=client_rover_wrench('front_left_wheel','world',set_point_front_left,set_force_front_left,start_time,duration_time)
    request_force3=client_rover_wrench('rear_wheel','world',set_point_rear,set_force_rear,start_time,duration_time)

rate=rospy.Rate(10)
start_time=rospy.Time(0)
duration_time=rospy.Duration(0, 100000000)

while not rospy.is_shutdown():
    set_force1()
    rate.sleep()