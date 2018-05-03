#! /usr/bin/env python

from gazebo_msgs.srv import ApplyJointEffort
import rospy
import math
import matplotlib.pyplot as plt

rospy.init_node('yuan_gazebo_server')
rospy.wait_for_service('/gazebo/apply_joint_effort')
client_rover=rospy.ServiceProxy('/gazebo/apply_joint_effort',ApplyJointEffort)

# define some constant parameters
r = 0.140  # unit m
R = 0.150  # r+h
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


# get z
def loop_z_FDP(z, s):
    theta1=math.acos((r-z)/r)
    theta1dot=math.acos((r-z)/R)
    thetam=(c1+c2*s)*theta1
    theta2=c3*theta1
    A=(math.cos(thetam)-math.cos(theta2))/(thetam-theta2)+(math.cos(thetam)-math.cos(theta1))/(theta1-thetam)
    B=(math.sin(thetam)-math.sin(theta2))/(thetam-theta2)+(math.sin(thetam)-math.sin(theta1))/(theta1-thetam)
    N=nk0+nk1*s
    sigmam=Ks*(r**N)*((math.cos(thetam)-math.cos(theta1))**N)
    taum=(1-math.exp(-rs*((theta1dot-thetam)-(1-s)*(math.sin(theta1dot)-math.sin(thetam)))/K))*(c+sigmam*math.tan(phy/180*math.pi))
    FN0=r*b*sigmam*A+rs*b*taum*B
    FDP=rs*b*taum*A-r*b*sigmam*B
    return [FN0, FDP]

def get_z_FDP(z, s, FN):
    FN0_FDP = loop_z_FDP(z, s)
    FN0 = FN0_FDP[0]
    while math.fabs(FN0-FN)>0.1:
        FN0_FDP = loop_z_FDP(z, s)
        FN0 = FN0_FDP[0]
        FDP = FN0_FDP[1]
        z+=0.00001
    else:
        pass
        #print(z)
        #print(FN0)
        #print(FDP)
    return [z, FDP]

def get_FN(z, s):
    theta1=math.acos((r-z)/r)
    theta1dot=math.acos((r-z)/R)
    thetam=(c1+c2*s)*theta1
    theta2=c3*theta1
    A=(math.cos(thetam)-math.cos(theta2))/(thetam-theta2)+(math.cos(thetam)-math.cos(theta1))/(theta1-thetam)
    B=(math.sin(thetam)-math.sin(theta2))/(thetam-theta2)+(math.sin(thetam)-math.sin(theta1))/(theta1-thetam)
    N=nk0+nk1*s
    sigmam=Ks*(r**N)*((math.cos(thetam)-math.cos(theta1))**N)
    taum=(1-math.exp(-rs*((theta1dot-thetam)-(1-s)*(math.sin(theta1dot)-math.sin(thetam)))/K))*(c+sigmam*math.tan(phy/180*math.pi))
    FN=r*b*sigmam*A+rs*b*taum*B
    return FN

def get_v(s, omega):
    if s>0:
        v=(1-s)*rs*omega
    else:
        v=rs*omega/(s+1)
    return v

def get_RC(s):
    RC = RC0*math.fabs(s+RC1)
    return RC

def get_TD(RC, FDP, FN):
    PC = FDP / FN
    TC = RC + PC
    TD = TC * r * FN
    return TD

# get TD and Z
def terrainmechanic(z=0.001,FN=(G*math.cos(slop/180*math.pi)),s=0.5,omega=0.5):
    print('+++++++++++++++++++++++++++ FN_init')
    print(FN)
    z_FDP = get_z_FDP(z, s, FN)
    z = z_FDP[0]
    FDP = z_FDP[1]
    print('********************** FDP')
    print(FDP)
    v = get_v(s, omega)
    print('********************** v')
    print(v)
    RC = get_RC(s)
    TD = get_TD(RC, FDP, FN)
    print('********************** TD')
    print(TD)

    while math.fabs(FDP) > (G*math.sin(slop/180*math.pi)+1):
        v = FDP / m *time_step+v
        print('********************** v')
        print(v)
        if rs * omega >= v:
            s = (rs * omega -v) / (rs * omega)
        else:
            s = (rs * omega -v) / v
        print('********************** s')
        print(s)
        z_FDP = get_z_FDP(z0, s, FN)
        z = z_FDP[0]
        print('********************** z')
        print(z)
        FDP = z_FDP[1]
        FN = get_FN(z, s)
        print('********************** FN')
        print(FN)
        RC = get_RC(s)
        print('********************** RC')
        print(RC)
        TD = get_TD(RC, FDP, FN)
        print('********************** TD')
        print(TD)

    else:
        print('---------------------- s')
        print(s)
        print('---------------------- FN')
        print(FN)
        print('---------------------- z')
        print(z)
        print('---------------------- TD')
        print(TD)
     
    return TD


rate=rospy.Rate(1)
TD=-terrainmechanic()
start_time=rospy.Time(0)
duration_time=rospy.Duration(1, 0)

while not rospy.is_shutdown():
    resquest_left=client_rover('front_left_joint',TD,start_time,duration_time)
    resquest_right=client_rover('front_right_joint',TD,start_time,duration_time)
    rate.sleep()
