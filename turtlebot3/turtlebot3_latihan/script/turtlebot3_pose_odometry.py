#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def posisiCallback(posisi_message):
    print ('pos x = {}'.format(posisi_message.pose.pose.position.x))           # cara penulisan dengan python3
    print ('pos y = {}'.format(posisi_message.pose.pose.position.y))
    print ('pos z = {}'.format(posisi_message.pose.pose.position.z))
    print ('orien x = {}'.format(posisi_message.pose.pose.orientation.x))           # cara penulisan dengan python3
    print ('orien y = {}'.format(posisi_message.pose.pose.orientation.y))
    print ('orien z = {}'.format(posisi_message.pose.pose.orientation.z))
    print ('orien w = {}'.format(posisi_message.pose.pose.orientation.w))
    # print "posisi si kura-kura:"
    # print ('x = {}'.format(posisi_message.x))           # cara penulisan dengan python3
    # print ('y = %f' % posisi_message.y)                 # cara penulisan dengan python2
    # print ('z = {}'.format(posisi_message.z))   # cara penulisan dengan python3
    #
    # if posisi_message.theta < 0:
    #     print "muter kiri terus"

if __name__ == '__main__':
    try:
        rospy.init_node('sub_turtlesim_pose', anonymous=True)
        posisi_topic = "/odom"
        pose_subscriber = rospy.Subscriber(posisi_topic, Odometry, posisiCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
