#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Vision:
    def __init__(self):
        rospy.init_node("vision_node")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.image_callback
        )


if __name__ == "__main__":
    try:
        Vision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
