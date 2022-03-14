#!/usr/bin/env python
# license removed for brevity

from re import I
import rospy
import math
from inverted_pendulum_sim.msg import ControlForce
import  time

global t_init
global strt
global t_now

t_init = 0
strt = True
t_now = 0

pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)

def sin_force():
    rospy.init_node('sin_force', anonymous=False)

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

        Amp = 20.0
        freq = 1
        w = math.pi*freq
        f = Amp*math.sin(w*t_now)
        rospy.loginfo(f)
        pub.publish(f)
        rate.sleep()

if __name__ == '__main__':
    try:
        sin_force()
    except rospy.ROSInterruptException:
        pass
