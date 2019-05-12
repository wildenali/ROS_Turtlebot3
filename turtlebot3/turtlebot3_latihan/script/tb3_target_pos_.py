#!/usr/bin/env python


# cara eksekusi program ini
# roslaunch turtlebot3_gazebo turtlebot3_world.launch
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/ROS_Turtlebot3/turtlebot3/turtlebot3_latihan/peta/map_kura_segienam.yaml
# cd ~/catkin_ws/src/ROS_Turtlebot3/turtlebot3/turtlebot3_latihan/script
    # python tb3_target_pos_.py


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
roll = pitch = yaw = current_yaw = 0.0
posisi_x = posisi_y = 0.0
target_x = 0.0
target_y = 0.0
target_yaw = 0

error_x = error_y = error_yaw = 0.0
jarak_tempuh = 0.0
desired_angle_goal = 0.0

def posisiCallback(posisi_message):
    global roll, pitch, yaw, posisi_x, posisi_y, current_yaw, target_x, target_y, target_yaw
    global jarak_tempuh, desired_angle_goal
    posisi_x = posisi_message.pose.pose.position.x
    posisi_y = posisi_message.pose.pose.position.y
    orientation_q = posisi_message.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw = yaw * 180/math.pi

    error_x = target_x - posisi_x
    error_y = target_y - posisi_y

    jarak_tempuh = abs(0.5 * math.sqrt(((error_x) ** 2) + ((error_y) ** 2)))
    desired_angle_goal = math.atan2(error_y, error_x)
    target_yaw = desired_angle_goal * 180 / math.pi

    error_yaw = target_yaw - current_yaw
    if error_x >= 0 and error_y >= 0 and error_yaw > 180:   #Q1
        error_yaw = error_yaw - 360
    elif error_x < 0 and error_y >= 0 and error_yaw > 180:   #Q2
        error_yaw = error_yaw - 360
    elif error_x < 0 and error_y < 0 and error_yaw < -180:  #Q3
        error_yaw = 360 + error_yaw
    elif error_x >= 0 and error_y < 0 and error_yaw < -180:  #Q4
        error_yaw = 360 + error_yaw


    # if error_yaw < -180:
    #     error_yaw = 360 + error_yaw

    print("posisi x     = {:.3f}\t target x   = {:.3f}\t error x   = {:.3f}".format(posisi_x, target_x, error_x))
    print("posisi y     = {:.3f}\t target y   = {:.3f}\t error y   = {:.3f}".format(posisi_y, target_y, error_y))
    print("current yaw  = {:.3f}\t target yaw = {:.3f}\t error yaw = {:.3f}".format(current_yaw, target_yaw, error_yaw))
    print("jarak tempuh = {:.3f}".format(jarak_tempuh))





if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot3_pos_yaw', anonymous=True)
        posisi_topic = "/odom"
        pose_subscriber = rospy.Subscriber(posisi_topic, Odometry, posisiCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")




# print("posisi x  = {} \t target x   = {} \t error x   = {}".format(posisi_x, target_x, error_x))
# print("posisi y  = {} \t target y   = {} \t error y   = {}".format(posisi_y, target_y, error_y))
# print("sudut yaw = {} \t target yaw = {} \t error yaw = {}".format(current_yaw, target_yaw, error_yaw))
