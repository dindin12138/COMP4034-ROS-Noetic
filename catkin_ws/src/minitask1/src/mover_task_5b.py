#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from math import pi, sqrt, pow


class SquareDriver:
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.init_node("square_driver", anonymous=True)
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.rate = rospy.Rate(10)

        self.pose = Pose()

        self.target_distance = 1.0
        self.target_angle = pi / 2

        self.linear_speed = 0.1
        self.angular_speed = 0.2

        self.start_x = 0
        self.start_y = 0
        self.start_theta = 0

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose.position.x = msg.pose.pose.position.x
        self.pose.position.y = msg.pose.pose.position.y
        self.pose.orientation.z = yaw

    def euclidean_distance(self):
        return sqrt(
            pow((self.pose.position.x - self.start_x), 2)
            + pow((self.pose.position.y - self.start_y), 2)
        )

    def move_forward(self):
        self.start_x = self.pose.position.x
        self.start_y = self.pose.position.y
        distance_moved = 0.0

        rospy.loginfo("Starting to move forward")

        while distance_moved < self.target_distance:
            move_msg = Twist()
            move_msg.linear.x = self.linear_speed
            self.pub.publish(move_msg)
            distance_moved = self.euclidean_distance()
            self.rate.sleep()

        self.stop()

    def rotate(self):
        self.start_theta = self.pose.orientation.z
        angle_rotated = 0.0

        rospy.loginfo("Starting rotation")

        while angle_rotated < self.target_angle:
            rotate_msg = Twist()
            rotate_msg.angular.z = self.angular_speed
            self.pub.publish(rotate_msg)
            angle_rotated = abs(self.pose.orientation.z - self.start_theta)
            self.rate.sleep()

        self.stop()

    def stop(self):
        stop_msg = Twist()
        self.pub.publish(stop_msg)
        rospy.loginfo("Stopping the robot")

    def run(self):
        for _ in range(4):
            self.move_forward()
            self.rotate()
        rospy.loginfo("Square path complete. Stop robot")
        self.stop()


if __name__ == "__main__":
    try:
        driver = SquareDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
