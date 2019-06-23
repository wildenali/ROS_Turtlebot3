#!/usr/bin/env python
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist



def move_forward():
    twist = Twist()
    twist.linear.x = 0.15
    pub.publish(twist)
    print("Maju pantat mundur")

def move_stop():
    twist = Twist()
    twist.linear.x = 0.0
    pub.publish(twist)
    print("Berhenti")

def move_kanan():
    twist = Twist()
    twist.angular.z = 0.5
    pub.publish(twist)
    print("Belok coy")

def main():
    while True:
        move_kanan()
        

rospy.init_node("Avaoider")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


if __name__ == '__main__':
    main()