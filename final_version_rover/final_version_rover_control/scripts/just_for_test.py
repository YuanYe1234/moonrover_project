#! /usr/bin/env python

from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import GetLinkState
import geometry_msgs.msg 

import rospy
import math

rospy.init_node('yuan_apply_body_wrench')
rospy.wait_for_service('/gazebo/apply_body_wrench')
rospy.wait_for_service('/gazebo/get_link_state')
rospy.wait_for_service('/gazebo/apply_joint_effort')
client_rover_link=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
client_rover_wrench=rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)
client_rover_effort=rospy.ServiceProxy('/gazebo/apply_joint_effort',ApplyJointEffort)

rate=rospy.Rate(1000) #10

# define some constant parameters
r = 0.140  # unit m
r_rear = 0.125
R = 0.150  # r+h
R_rear = 0.125
rs = 0.1465
b = 0.144
b_rear=0.10
c1 = 0.5  # from Ding
c2 = 0  # from Ding
c3 = 0  # from Ding
c = 0.2  # fake
phy = 30.0  # fake unit:degree
K = 0.010  # fake unit:m
Ks = 820000  # fake unit:Pa/m**N
nk0 = 0.75  # from Yang
nk1 = 0.2  # from Yang
RC0 = 0.4  #fake
RC1 = 0.2  #fake
m = 28.78 #kg
time_step = 0.01 #10hz
z0 = 0.0001
slop = 0.0
G=100
v_parameter = 250
omega = 1.0

def get_sinkage():
    request_right = client_rover_link('front_right_wheel', 'world')
    request_left = client_rover_link('front_left_wheel', 'world')
    request_rear = client_rover_link('rear_wheel', 'world')

    sinkage_right = request_right.link_state.pose.position.z
    position_right_y = request_right.link_state.pose.position.y
    # print('++++++++++++++++++++++++sinkage_right')
    # print(0.011-sinkage_right)
    sinkage_left = request_left.link_state.pose.position.z
    position_left_y = request_left.link_state.pose.position.y
    # print('++++++++++++++++++++++++sinkage_left')
    # print(0.011-sinkage_left)
    sinkage_rear = request_rear.link_state.pose.position.z
    position_rear_y = request_rear.link_state.pose.position.y
    # print('++++++++++++++++++++++++sinkage_rear')
    print(0.011-0.025-sinkage_rear)  
    if (position_right_y+position_left_y)/2.0 >= 2.5 and (position_right_y+position_left_y)/2.0 <= 8.5:
        sinkage_right = sinkage_right + 0.5*math.sin((position_right_y-2.5)/3.0*math.pi)  
        sinkage_left = sinkage_left + 0.5*math.sin((position_left_y-2.5)/3.0*math.pi)
        # print('if**if**if**if')
        # print(sinkage_left)
        # print(sinkage_right)
        # print(sinkage_rear)
    else:
        pass

    if position_rear_y >= 2.5 and position_rear_y <= 8.5:
        sinkage_rear = sinkage_rear + 0.5*math.sin((position_rear_y-2.5)/3.0*math.pi)
    else:
        pass

    return {'z_right':0.0-sinkage_right, 'z_left':0.0-sinkage_left, 'z_rear':0.0-0.025-sinkage_rear}

def get_velocity():
    request_right=client_rover_link('front_right_wheel', 'world')
    request_left=client_rover_link('front_left_wheel', 'world')
    request_rear=client_rover_link('rear_wheel', 'world')
    vz_right=request_right.link_state.twist.linear.z
    # print('++++++++++++++++++++++++vz_right')
    # print(vz_right)
    vz_left=request_left.link_state.twist.linear.z
    # print('++++++++++++++++++++++++vz_left')
    # print(vz_left)
    vz_rear=request_rear.link_state.twist.linear.z
    # print('++++++++++++++++++++++++vz_rear')
    # print(vz_rear)    
    return {'vz_right':vz_right, 'vz_left':vz_left, 'vz_rear':vz_rear}
def get_vy_read():
    request_right=client_rover_link('front_right_wheel', 'world')
    request_left=client_rover_link('front_left_wheel', 'world')
    request_rear=client_rover_link('rear_wheel', 'world')

    vy_rear=request_rear.link_state.twist.linear.y
    vy_right=request_right.link_state.twist.linear.y
    vy_left=request_left.link_state.twist.linear.y

    return {'vy_right':vy_right, 'vy_left':vy_left, 'vy_rear':vy_rear}

def get_FN_func(z, s):
    if z>0:
        # print('****z***')
        # print(z)
        # print('****r-z***')
        # print(r-z)
        # print('****r***')
        # print(r)       
        theta1=math.acos((r-z)/r) # what about z > 2*r ??
        theta1dot=math.acos((r-z)/R)
        thetam=(c1+c2*s)*theta1
        theta2=c3*theta1
        A=(math.cos(thetam)-math.cos(theta2))/(thetam-theta2)+(math.cos(thetam)-math.cos(theta1))/(theta1-thetam)
        B=(math.sin(thetam)-math.sin(theta2))/(thetam-theta2)+(math.sin(thetam)-math.sin(theta1))/(theta1-thetam)
        N=nk0+nk1*s
        sigmam=Ks*(r**N)*((math.cos(thetam)-math.cos(theta1))**N)
        taum=(1-math.exp(-rs*((theta1dot-thetam)-(1-s)*(math.sin(theta1dot)-math.sin(thetam)))/K))*(c+sigmam*math.tan(phy/180*math.pi))
        FN=r*b*sigmam*A+rs*b*taum*B
    else:
        FN=0
    # print('*****************FN_front from z & s')
    # print(FN)
    return FN   

def get_FN_rear_func(z, s):
    if z>0:
        theta1=math.acos((r_rear-z)/r_rear)
        theta1dot=math.acos((r_rear-z)/R_rear)
        thetam=(c1+c2*s)*theta1
        theta2=c3*theta1
        A=(math.cos(thetam)-math.cos(theta2))/(thetam-theta2)+(math.cos(thetam)-math.cos(theta1))/(theta1-thetam)
        B=(math.sin(thetam)-math.sin(theta2))/(thetam-theta2)+(math.sin(thetam)-math.sin(theta1))/(theta1-thetam)
        N=nk0+nk1*s
        sigmam=Ks*(r**N)*((math.cos(thetam)-math.cos(theta1))**N)
        taum=(1-math.exp(-rs*((theta1dot-thetam)-(1-s)*(math.sin(theta1dot)-math.sin(thetam)))/K))*(c+sigmam*math.tan(phy/180*math.pi))
        #FN=1000*b*sigmam*A+rs*b*taum*B
        FN=r_rear*b_rear*sigmam*A+r_rear*b_rear*taum*B
    else:
        FN=0
    # print('*****************FN_rear from z & s')
    # print(FN)
    return FN

def get_FDP_func(z, s):
    if z>0:
        theta1=math.acos((r-z)/r)
        theta1dot=math.acos((r-z)/R)
        thetam=(c1+c2*s)*theta1
        theta2=c3*theta1
        A=(math.cos(thetam)-math.cos(theta2))/(thetam-theta2)+(math.cos(thetam)-math.cos(theta1))/(theta1-thetam)
        B=(math.sin(thetam)-math.sin(theta2))/(thetam-theta2)+(math.sin(thetam)-math.sin(theta1))/(theta1-thetam)
        N=nk0+nk1*s
        sigmam=Ks*(r**N)*((math.cos(thetam)-math.cos(theta1))**N)
        taum=(1-math.exp(-rs*((theta1dot-thetam)-(1-s)*(math.sin(theta1dot)-math.sin(thetam)))/K))*(c+sigmam*math.tan(phy/180*math.pi))
        FDP=rs*b*taum*A-r*b*sigmam*B
    else:
        FDP=0
    # print('*****************FDP_front from z & s')
    # print(FDP)
    return FDP
def get_FDP_rear_func(z, s):
    if z>0:
        theta1=math.acos((r_rear-z)/r_rear)
        theta1dot=math.acos((r_rear-z)/R_rear)
        thetam=(c1+c2*s)*theta1
        theta2=c3*theta1
        A=(math.cos(thetam)-math.cos(theta2))/(thetam-theta2)+(math.cos(thetam)-math.cos(theta1))/(theta1-thetam)
        B=(math.sin(thetam)-math.sin(theta2))/(thetam-theta2)+(math.sin(thetam)-math.sin(theta1))/(theta1-thetam)
        N=nk0+nk1*s
        sigmam=Ks*(r**N)*((math.cos(thetam)-math.cos(theta1))**N)
        taum=(1-math.exp(-rs*((theta1dot-thetam)-(1-s)*(math.sin(theta1dot)-math.sin(thetam)))/K))*(c+sigmam*math.tan(phy/180*math.pi))
        FDP=rs*b*taum*A-r*b*sigmam*B
    else:
        FDP=0
    # print('*****************FDP_rear from z & s')
    # print(FDP)
    return FDP

def get_FN(z_dict, s):
    FN_right=get_FN_func(z_dict['z_right'],s)
    FN_left=get_FN_func(z_dict['z_left'],s)
    FN_rear=get_FN_rear_func(z_dict['z_rear'],s)
    return {'FN_right':FN_right, 'FN_left':FN_left, 'FN_rear':FN_rear}

def get_FDP(z_dict, s):
    FDP_right=get_FDP_func(z_dict['z_right'],s)
    FDP_left=get_FDP_func(z_dict['z_left'],s)
    return {'FDP_right':FDP_right, 'FDP_left':FDP_left}

def get_v(s, omega):
    if s>0:
        v=(1-s)*rs*omega
    else:
        v=rs*omega/(s+1)
    return v

def get_RC(s):
    RC = RC0*math.fabs(s+RC1)
    return RC

def set_force(FN_dict, FDP_dict, vz_dict, z_dict, s):
    now = rospy.Time.now()
    set_point_front_left = geometry_msgs.msg.Point()
    set_point_front_left.z = 0#-0.1465
    #set_point_front_left.x= -0.074  #will rotate 
    set_point_front_right = geometry_msgs.msg.Point()
    set_point_front_right.z = 0#-0.1465 
    #set_point_front_right.x = 0.074  
    set_point_rear = geometry_msgs.msg.Point()
    set_point_rear.z = 0#-0.125
    
    set_force_front_right = geometry_msgs.msg.Wrench()
    set_force_front_right.force.z = FN_dict['FN_right'] - v_parameter * vz_dict['vz_right']
    if now.secs >= 2:
        set_force_front_right.force.y = FDP_dict['FDP_right']
        #set_force_front_right.force.y = 0.5
    else:
        pass
    #set_force_front_right.force.y = 0.5
    #set_force_front_right.force.y = FDP_dict['FDP_right']
    # print('*******************set*FN********front_right')
    # print(set_force_front_right.force.z)
    #print(set_force_front_right.force.y)
    set_force_front_left = geometry_msgs.msg.Wrench()
    set_force_front_left.force.z = FN_dict['FN_left'] - v_parameter * vz_dict['vz_left']
    if now.secs >= 2:
        set_force_front_left.force.y = FDP_dict['FDP_left']
        #set_force_front_left.force.y = 0.5
    else:
        pass
    #set_force_front_left.force.y = FDP_dict['FDP_left']
    # print('*******************set*FN********front_left')
    # print(set_force_front_left.force.z)    
    #print(set_force_front_left.force.y)
    set_force_rear = geometry_msgs.msg.Wrench()
    set_force_rear.force.z = FN_dict['FN_rear'] - v_parameter * vz_dict['vz_rear']
    if now.secs >= 2:
      set_force_rear.force.y = -0.05*set_force_rear.force.z
      print('FDP_rear')
      print(set_force_rear.force.y)
    else:
        pass
    #set_force_rear.force.y = 1
    # set_force_rear.force.y = FDP_dict['FDP_rear']
    # print('*******************set*FN********rear')
    # print(set_force_rear.force.z)    

    request_force=client_rover_wrench('front_right_wheel','world',set_point_front_right,set_force_front_right,start_time,duration_time)
    request_force=client_rover_wrench('front_left_wheel','world',set_point_front_left,set_force_front_left,start_time,duration_time)
    request_force=client_rover_wrench('rear_wheel','world',set_point_rear,set_force_rear,start_time,duration_time)
    
    return {'set_front_right_FN':set_force_front_right.force.z, 'set_front_left_FN':set_force_front_right.force.z, 'set_rear_FN':set_force_rear.force.z,}

def set_force1():
    vz=get_velocity()
    vz_right=vz['vz_right']
    vz_left=vz['vz_left']
    vz_rear=vz['vz_rear']
    set_point_front_left = geometry_msgs.msg.Point()
    set_point_front_left.z = -0.1465
    #set_point_front_left.x= -0.074  #will rotate 
    set_point_front_right = geometry_msgs.msg.Point()
    set_point_front_right.z = -0.1465 
    #set_point_front_right.x = 0.074  
    set_point_rear = geometry_msgs.msg.Point()
    set_point_rear.z = -0.125

    set_force_front_right = geometry_msgs.msg.Wrench()
    set_force_front_right.force.z = 20.4*8 - 21 * vz_right
    #set_force_front_right.force.y = 0.1
    print('*******************front_right*******************up')
    print(set_force_front_right.force.z)
    print('*******************front_right*******************down')
    set_force_front_left = geometry_msgs.msg.Wrench()
    set_force_front_left.force.z = 20.4*8 - 21 * vz_left
    #set_force_front_left.force.y = 0.1
    print('*******************front_left*******************up')
    print(set_force_front_left.force.z) 
    print('*******************front_left*******************down')   
    set_force_rear = geometry_msgs.msg.Wrench()
    set_force_rear.force.z = 5.1*8 - 21 * vz_rear
    print('*******************rear')
    print(set_force_rear.force.z)    

    request_force=client_rover_wrench('front_right_wheel','world',set_point_front_right,set_force_front_right,start_time,duration_time)
    request_force=client_rover_wrench('front_left_wheel','world',set_point_front_left,set_force_front_left,start_time,duration_time)
    request_force=client_rover_wrench('rear_wheel','world',set_point_rear,set_force_rear,start_time,duration_time)

def get_FN_fake(z_dict, s):
    z=get_sinkage()
    FN_right=z_dict['z_right']*s*10000
    FN_left=z_dict['z_left']*s*10000
    FN_rear=z_dict['z_rear']*s*10000
    # print('get_FN***********FN_right')
    # print(FN_right)
    return {'FN_right':FN_right, 'FN_left':FN_left, 'FN_rear':FN_rear}


def get_FDP_fake(z_dict, s):
    FDP_right=0
    FDP_left=0
    return {'FDP_right':FDP_right, 'FDP_left':FDP_left}

def get_s(vy_dict, omega):
    if rs*omega >= vy_dict['vy_right']:
        s_right = ((rs*omega) - vy_dict['vy_right']) / (rs*omega)
    else:
        s_right = ((rs*omega) - vy_dict['vy_right']) / vy_dict['vy_right']
    
    if rs*omega >= vy_dict['vy_left']:
        s_left = ((rs*omega) - vy_dict['vy_left']) / (rs*omega)
    else:
        s_left = ((rs*omega) - vy_dict['vy_left']) / vy_dict['vy_left']
    
    s=(s_left+s_right)/2
    print('s_avg')
    print(s)
    if s >= 1:
        s=1
    elif s <= 0:
        s=0

    return s



def main(s, omega):
    z_dict = get_sinkage()
    vz_dict = get_velocity()
    FN_dict = get_FN(z_dict, s)
    #FN_dict = get_FN_fake(z_dict, s)
    FDP_dict = get_FDP(z_dict,s)
    #FDP_dict = get_FDP_fake(z_dict,s)
    # vy = get_v(s, omega)
    # vy_dict = get_vy_read()
    print('z_dict*********')
    print(z_dict)    
    print('vz_dict*********')
    print(vz_dict)
    print('FN_dict*********')
    print(FN_dict)
    print('FDP_dict*********')
    print(FDP_dict)
    set_FN=set_force(FN_dict, FDP_dict, vz_dict, z_dict, s)
    print('set_FN*********')
    print(set_FN)
    # print('vy*********')
    # print(vy)
    # print('vy_read*********')
    # print(vy_dict)

rate=rospy.Rate(1000)
start_time=rospy.Time(0)
duration_time=rospy.Duration(0, 100000000)

while not rospy.is_shutdown():
    #set_force1()
    vy_dict=get_vy_read()
    s=get_s(vy_dict, omega)
    print('ssssssssssssssss')
    print(s)
    # print('vy_read*********')
    # print(vy_dict)
    # s=0.1
    main(s, omega)
    # now = rospy.Time.now()
    # rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
    print("\033[0;31m%s\033[0m" %'------------------------------------------')
    rate.sleep()