#!/usr/bin/env python
# license removed for brevity

import rospy
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.msg import CurrentState
import time
import math


class Pid:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, dt=0.01):
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.dt = 0.01
        self.u_max = 9999999.9
        self.u_min = -9999999.9
        self.I_ = 0
        self.e_pre = 0

    def set_gains(self,Kp_,Ki_,Kd_):
        self.Kp = Kp_
        self.Ki = Ki_
        self.Kd = Kd_

    def set_max(self, u_max_):
        self.u_max = u_max_
        self.u_min = -u_max_

    def limit(self, x,x_max,x_min):
        if x > x_max:
            x = x_max
        elif x < x_min:
            x = x_min
        return x

    def calc(self, x0=0, x=0):
        e = x0 - x
        self.I_ += e*self.dt
        self.I_ = self.limit(self.I_, self.u_max, self.u_min)
        u = self.Kp*e + self.Ki*self.I_ + self.Kd*(e-self.e_pre)/self.dt
        u = self.limit(u, self.u_max, self.u_min)
        self.e_pre = e
        return u

ctrl_x  = Pid(Kp=0.0250, Ki=0.000, Kd=00.0, dt=0.01)
ctrl_xd = Pid(Kp=-0.0015, Ki=-0.005, Kd=00.0, dt=0.01)
ctrl_th = Pid(Kp=200.0 , Ki=100.0, Kd=75.0, dt=0.01)

ctrl_x.set_gains(0.0250 , 0.000, 00.0)
ctrl_xd.set_gains(-0.0015,-0.005, 00.0)
ctrl_th.set_gains(200.0 , 100.0, 75.0)

ctrl_x.set_max(20.0)
ctrl_xd.set_max(math.pi/6.0)
ctrl_th.set_max(100.0)


global t_init, strt, t_now
t_init = 0
strt = True
t_now = 0

pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)

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

def main():
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
        
        x0 = 20.0 #User input

        x_dot_0 = ctrl_x.calc(x0, x)
        print(x)
        th0 = ctrl_xd.calc(x_dot_0, x_dot)
        # print(x_dot)

        f = ctrl_th.calc(th0, th)
        #print(th*180.0/math.pi)

        pub.publish(f)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
