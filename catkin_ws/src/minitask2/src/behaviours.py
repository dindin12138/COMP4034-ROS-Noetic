#!/usr/bin/env python3

import rospy
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Behaviour:
    def __init__(self):
        rospy.init_node("behavior", anonymous=True)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)

        self.move_cmd = Twist()

        self.rate = rospy.Rate(10)

        self.state = "RANDOM_WALK"

        self.front_distance = 0.0
        self.right_distance = 0.0
        self.left_distance = 0.0

    def laser_scan_callback(self, msg):
        self.front_distance = min(min(msg.ranges[0:30]), min(msg.ranges[330:360]))
        self.right_distance = min(msg.ranges[240:300])
        self.left_distance = min(msg.ranges[60:120])

    def obstacle_avoidance(self):
        self.move_cmd.linear.x = 0.0
        if abs(self.right_distance - self.left_distance) > 0.2:
            if self.right_distance > self.left_distance:
                self.move_cmd.angular.z = -0.5
                rospy.loginfo("Obstacle, rotate to the right")
            else:
                self.move_cmd.angular.z = 0.5
                rospy.loginfo("Obstacle, rotate to the left")
        else:
            self.move_cmd.angular.z = 0.5 if self.move_cmd.angular.z >= 0 else -0.5
        self.pub.publish(self.move_cmd)

    def right_hand_wall_following(self):
        rospy.loginfo("Following the Right-hand Wall")
        self.move_cmd.linear.x = 0.1
        self.move_cmd.angular.z = 0.1 if self.right_distance < 0.3 else -0.1
        self.pub.publish(self.move_cmd)

    def random_walk(self):
        rospy.loginfo("Random Walking")

        forward_duration = rospy.Duration(5.0)
        rotate_duration = rospy.Duration(2.0)

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < forward_duration:
            if self.front_distance < 0.3:
                self.state = "OBSTACLE_AVOIDANCE"
                return
            self.move_cmd.linear.x = 0.2
            self.move_cmd.angular.z = 0.0
            self.pub.publish(self.move_cmd)
            self.rate.sleep()

        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time) < rotate_duration:
            if self.front_distance < 0.3:
                self.state = "OBSTACLE_AVOIDANCE"
                return
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = random.choice([-0.5, 0.5])
            self.pub.publish(self.move_cmd)
            self.rate.sleep()

        self.state = "RANDOM_WALK"

    def run(self):
        while not rospy.is_shutdown():
            if self.state == "OBSTACLE_AVOIDANCE":
                self.obstacle_avoidance()
            elif self.state == "RIGHT_HAND_WALL_FOLLOWING":
                self.right_hand_wall_following()
            else:
                self.random_walk()

            if self.front_distance < 0.3:
                self.state = "OBSTACLE_AVOIDANCE"
            elif self.right_distance < 0.5:
                self.state = "RIGHT_HAND_WALL_FOLLOWING"
            else:
                self.state = "RANDOM_WALK"


if __name__ == "__main__":
    try:
        behavior = Behaviour()
        rospy.sleep(2)
        behavior.run()
    except rospy.ROSInterruptException:
        pass
