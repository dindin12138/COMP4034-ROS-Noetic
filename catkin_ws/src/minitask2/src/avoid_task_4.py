#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def laser_callback(msg):
    front_distances = [msg.ranges[i] for i in range(-10, 11)]
    min_front_distance = min(front_distances)

    safe_distance = 1.0

    rospy.loginfo(f"Min front distance: {min_front_distance:.2f} m")

    twist_msg = Twist()
    if min_front_distance < safe_distance:
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
    else:
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = 0.0

    pub.publish(twist_msg)


if __name__ == "__main__":
    rospy.init_node("avoid_obstacle_node")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("scan", LaserScan, laser_callback)
    rospy.spin()
