#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi


def move_forward(pub, forward_duration, linear_speed):
    twist_msg = Twist()
    twist_msg.linear.x = linear_speed
    twist_msg.angular.z = 0.0
    start_time = rospy.Time.now().to_sec()
    rospy.loginfo(f"Moving forward: {twist_msg}")
    while rospy.Time.now().to_sec() - start_time < forward_duration:
        pub.publish(twist_msg)
    stop_robot(pub)


def turn(pub, turn_duration, angular_speed):
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = angular_speed
    start_time = rospy.Time.now().to_sec()
    rospy.loginfo(f"Turning for {turn_duration} seconds")
    while rospy.Time.now().to_sec() - start_time < turn_duration:
        pub.publish(twist_msg)
    stop_robot(pub)


def stop_robot(pub):
    twist_msg = Twist()
    pub.publish(twist_msg)
    rospy.loginfo("Stopping the robot")
    rospy.sleep(0.5)


def mover():
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.init_node("mover", anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    linear_speed = 0.1
    angular_speed = 0.5
    forward_time = 1 / linear_speed
    turn_time = (pi / 2) / angular_speed

    while not rospy.is_shutdown():
        move_forward(pub, forward_time, linear_speed)
        turn(pub, turn_time, angular_speed)
        rate.sleep()


if __name__ == "__main__":
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
