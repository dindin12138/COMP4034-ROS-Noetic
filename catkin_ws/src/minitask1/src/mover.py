#!/usr/bin/env python

import rospy

# from std_msgs.msg import String
from geometry_msgs.msg import Twist


def mover():
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.init_node("mover", anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()

        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0

        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        rospy.loginfo(f"Publishing: {twist_msg}")
        pub.publish(twist_msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
