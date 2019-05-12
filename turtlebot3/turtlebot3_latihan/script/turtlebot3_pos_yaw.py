#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
roll = pitch = yaw = 0.0

def posisiCallback(posisi_message):
    print('posisi x  = {}'.format(posisi_message.pose.pose.position.x))           # cara penulisan dengan python3
    print('posisi y  = {}'.format(posisi_message.pose.pose.position.y))

    global roll, pitch, yaw
    orientation_q = posisi_message.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw_derajat = yaw * 180/math.pi
    print("sudut yaw = {}".format(yaw_derajat))
    # yaw_rad = yaw_derajat * math.pi/180
    # print("Radian = {}  Derajat:{}".format(yaw_rad, yaw_derajat))



if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot3_pos_yaw', anonymous=True)
        posisi_topic = "/odom"
        pose_subscriber = rospy.Subscriber(posisi_topic, Odometry, posisiCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
