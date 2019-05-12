#!/usr/bin/env python
import rospy
import numpy as np
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(data):
    # laser_data = np.array(data.ranges)
    # laser_data = laser_data[-np.isinf(laser_data)]
    # print(min(laser_data))
    # move_forward()
    # time.sleep(2)
    # move_kanan()
    # laser_data = 1
    # time.sleep(2)
    # if min(laser_data) < 0.5:
    #     move_stop()
    # else:
    #     move_forward()

    print(data.ranges[0])
    if data.ranges[0] < 0.8:
        move_kanan()
        time.sleep(0.5)
    else:
        move_forward()




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
    sub=rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

rospy.init_node("Avaoider")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


if __name__ == '__main__':
    main()
