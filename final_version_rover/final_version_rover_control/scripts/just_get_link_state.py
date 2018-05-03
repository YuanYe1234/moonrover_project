#! /usr/bin/env python

from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import SetLinkState
import geometry_msgs.msg 

import rospy
import math

rospy.init_node('yuan_get_link_state')
rospy.wait_for_service('/gazebo/apply_body_wrench')
rospy.wait_for_service('/gazebo/get_link_state')
#rospy.wait_for_service('/gazebo/set_link_state')
client_rover_link=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
client_rover=rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)
#client_rover_set_link_state=rospy.ServiceProxy('/gazebo/set_link_state',SetLinkState)

rate=rospy.Rate(1000) #10Hz

def get_z(): # assume that 1m is the ground 
    request_z=client_rover_link('link_0','world')
    z=request_z.link_state.pose.position.z
    print('****z')
    print(z)
    sinkage=0-z
    print('****sinkage')
    print(sinkage)
    return sinkage

def get_velocity():
    request_link=client_rover_link('link_0','world')
    vz=request_link.link_state.twist.linear.z
    print('****vz')
    print(vz)
    return vz
def get_FN():
    z=get_z()
    if z>0.0:
        FN=10000.0*z
    else:
        FN=0.0
    print('****FN****from****z')
    print(FN)
    return FN


def set_force():
    vz=get_velocity()
    set_point = geometry_msgs.msg.Point()
    set_force = geometry_msgs.msg.Wrench()
    FN = get_FN()

    set_force.force.z=FN-50*vz
    #set_force.torque.x=2
    #print('****force')
    #print(set_force.force.z)
    start_time=rospy.Time(0)
    duration_time=rospy.Duration(0, 1000000)
    request_force=client_rover('link_0','world',set_point,set_force,start_time,duration_time)


while not rospy.is_shutdown():
    set_force()
    rate.sleep()








