#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import sys
from std_msgs.msg import Float64MultiArray

def talker():
    pub = rospy.Publisher('/servo_controller/command', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    		smallArray = [[40,10,20],[30,50,60]]
    		array = [smallArray[0][1], 60, 60, 30, 60, 60, 30, 60, 60, 60, 60, 50]
    		msg = Float64MultiArray(data=array)
    		rospy.loginfo(msg)
    		pub.publish(msg)
    		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
