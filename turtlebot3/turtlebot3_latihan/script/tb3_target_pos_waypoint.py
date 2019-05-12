#!/usr/bin/env python

# cara eksekusi program ini
# roslaunch turtlebot3_gazebo turtlebot3_world.launch
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/ROS_Turtlebot3/turtlebot3/turtlebot3_latihan/peta/map_kura_segienam.yaml
# cd ~/catkin_ws/src/ROS_Turtlebot3/turtlebot3/turtlebot3_latihan/script
    # python tb3_target_pos_waypoint.py


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import time
from geometry_msgs.msg import Twist

roll = pitch = yaw = current_yaw = 0.0
posisi_x = posisi_y = 0.0
target_x = target_y = 0.0
target_yaw = 0

target_x1 = 0.5
target_y1 = -0.5

target_x2 = 0.5
target_y2 = 0.5

target_x3 = -0.5
target_y3 = 0.5

target_x4 = -0.5
target_y4 = -0.5

error_x = error_y = error_yaw = 0.0
jarak_tempuh = 0.0
desired_angle_goal = 0.0

# kP_yaw = 0.03
kP_yaw = 0.1
kP_jarak = 0.01

waypoint = 1
kecepatan_awal = 0.15

def posisiCallback(posisi_message):
    global roll, pitch, yaw, posisi_x, posisi_y, current_yaw

    posisi_x = posisi_message.pose.pose.position.x
    posisi_y = posisi_message.pose.pose.position.y
    orientation_q = posisi_message.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_yaw = yaw * 180/math.pi



def move():
    global roll, pitch, yaw, posisi_x, posisi_y, current_yaw
    global target_x, target_y, target_yaw, target_x1, target_y1
    global jarak_tempuh, desired_angle_goal
    global kP_yaw, kP_jarak
    global waypoint, kecepatan_awal

    loop_rate = rospy.Rate(100)  # publish message 10 times per seconds
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    waypoint = input("waypoint ke : ")
    while True:
        if waypoint > 4 or waypoint < 1:
            break
        if waypoint == 1:
            target_x = target_x1
            target_y = target_y1
        if waypoint == 2:
            target_x = target_x2
            target_y = target_y2
        if waypoint == 3:
            target_x = target_x3
            target_y = target_y3
        if waypoint == 4:
            target_x = target_x4
            target_y = target_y4

        error_x = target_x - posisi_x
        error_y = target_y - posisi_y




        jarak_tempuh = abs(0.5 * math.sqrt(((error_x) ** 2) + ((error_y) ** 2)))
        desired_angle_goal = math.atan2(error_y, error_x)
        target_yaw = desired_angle_goal * 180 / math.pi

        error_yaw = target_yaw - current_yaw
        if error_x >= 0 and error_y >= 0 and error_yaw > 180:       #Q1
            error_yaw = error_yaw - 360
        elif error_x < 0 and error_y >= 0 and error_yaw > 180:      #Q2
            error_yaw = error_yaw - 360
        elif error_x < 0 and error_y < 0 and error_yaw < -180:      #Q3
            error_yaw = 360 + error_yaw
        elif error_x >= 0 and error_y < 0 and error_yaw < -180:     #Q4
            error_yaw = 360 + error_yaw

        pid_jarak = kP_jarak * jarak_tempuh
        pid_yaw = kP_yaw * error_yaw


        twist = Twist()

        # if jarak_tempuh <= 0.20:
        #     kecepatan = jarak_tempuh - 0.10
        # elif jarak_tempuh <= 0.10:
        #     kecepatan = jarak_tempuh/2
        # else:
        #     kecepatan = kecepatan_awal

        kecepatan = jarak_tempuh

        speed_x = kecepatan + pid_jarak
        speed_yaw = pid_yaw

        if speed_x > 0.15:
            speed_x = 0.15


        if error_yaw > 75 or error_yaw < -75 :      # si robot di suruh muter dulu(kecepatan linear.x nya di 0 kan), kalau error_yaw lebih besar dari 10 derajar atau lebih kecil -10 derajar
            speed_x = 0.0


        if speed_yaw > 1.2:
            speed_yaw = 1.2
        if speed_yaw < -1.2:
            speed_yaw = -1.2

        if jarak_tempuh < 0.02:     # 5cm
            speed_yaw = 0.0
            speed_x = 0.0
            twist.linear.x = speed_x
            twist.angular.z = speed_yaw
            pub.publish(twist)
            break
        else:
            twist.linear.x = speed_x
            twist.angular.z = speed_yaw
            pub.publish(twist)
            loop_rate.sleep()

        print("posisi x     = {:.3f}\t target x   = {:.3f}\t error x   = {:.3f}".format(posisi_x, target_x, error_x))
        print("posisi y     = {:.3f}\t target y   = {:.3f}\t error y   = {:.3f}".format(posisi_y, target_y, error_y))
        print("current yaw  = {:.3f}\t target yaw = {:.3f}\t error yaw = {:.3f}".format(current_yaw, target_yaw, error_yaw))
        print("jarak tempuh = {:.3f}\t pid yaw    = {:.3f}\t pid jarak = {:.3f}".format(jarak_tempuh, pid_yaw, pid_jarak))
        print("speed x      = {:.3f}\t speed yaw  = {:.3f}\t waypoint  = {}".format(speed_x, speed_yaw, waypoint))






if __name__ == '__main__':
    try:
        hajarbleh = True
        while hajarbleh == True:
            rospy.init_node('turtlebot3_pos_yaw', anonymous=True)
            posisi_topic = "/odom"
            pose_subscriber = rospy.Subscriber(posisi_topic, Odometry, posisiCallback)
            time.sleep(1)
            move()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
