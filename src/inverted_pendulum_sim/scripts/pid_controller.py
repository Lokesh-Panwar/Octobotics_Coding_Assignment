#!/usr/bin/env python
# license removed for brevity

import rospy
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.msg import CurrentState
import time
import math


global t_init
global strt
global t_now

t_init = 0
strt = True
t_now = 0

pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)

def limit(x,x_max,x_min):
    if x > x_max:
        x = x_max
    elif x < x_min:
        x = x_min
    return x

global I_
global e_pre
I_ = 0
e_pre = 0

global x, x_dot, x_dot2
global th, th_dot, th_dot2
x = 0
x_dot = 0
x_dot2 = 0
th = 0
th_dot = 0
th_dot2 = 0

def callback(data):
    global x, x_dot, x_dot2
    global th, th_dot, th_dot2
    x = data.curr_x
    x_dot = data.curr_x_dot
    x_dot2 = data.curr_x_dot_dot
    th = data.curr_theta
    if(th>=0):
        th = th - math.pi
        while(th>=math.pi):
            th = th - math.pi
    elif(th<0):
        th = th + math.pi
        while(th<-math.pi):
            th = th + math.pi
    th_dot = data.curr_theta_dot
    th_dot2 = data.curr_theta_dot_dot


def f_pid(x0=0, x=0, dt=1.0, Kp=0, Ki=0, Kd=0):
    u_max = 100000.0
    u_min = -u_max
    global I_
    global e_pre

    e = x0 - x
    I_ += e*dt
    I_ = limit(I_, u_max, u_min)
    u = Kp*e + Ki*I_ + Kd*(e-e_pre)/dt
    u = limit(u, u_max, u_min)
    e_pre = e
    return u

global II
II = 0.0

def sin_force():
    rospy.init_node('sin_force', anonymous=False)
    rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, callback)

    rate = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():
        global t_init
        global strt
        if(strt==True):
            global t_init
            global strt
            t_init = time.time()
            strt = False
        t_now = time.time() - t_init
        #print(t_now)
        dt = 0.01

        global x, x_dot, x_dot2
        global th, th_dot, th_dot2
        # print(x, x_dot, x_dot2, th, th_dot, th_dot2)
        #print(th*180.0/math.pi)
        print(x)

        # position controller
        x0 = 0.0
        x_dot_0 = 0.025*(x0 - x)

        # velocity controller
        kp_x = 0.0015
        ki_x = 0.005
        err = x_dot_0-x_dot
        global II
        II += err*dt
        II = limit(II, math.pi/6.0, -math.pi/6.0)
        th0 = -kp_x*err -ki_x*II
        th0 = limit(th0, math.pi/6.0, -math.pi/6.0)

        kp_th = 200.0
        ki_th = 100.0
        kd_th = 75.0
        f = f_pid(th0, th, dt, kp_th, ki_th, kd_th)
        # rospy.loginfo(f)
        pub.publish(f)
        rate.sleep()

if __name__ == '__main__':
    try:
        sin_force()
    except rospy.ROSInterruptException:
        pass
