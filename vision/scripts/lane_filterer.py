#!/usr/bin/python3

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class LaneFilterer:
    def __init__(self):
        rospy.init_node("lane_filter_node", anonymous=True)

        self.hh = rospy.get_param("~higher_h", 180)
        self.hs = rospy.get_param("~higher_s", 255)
        self.hv = rospy.get_param("~higher_v", 255)
        self.lh = rospy.get_param("~lower_h", 0)
        self.ls = rospy.get_param("~lower_s", 0)
        self.lv = rospy.get_param("~lower_v", 0)
        self.kernel_size = rospy.get_param("~kernel_size", 3)
        self.iterations = rospy.get_param("~iterations", 3)

        self.image_pub = rospy.Publisher("mask_topic_pub", Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            "rgb_image_topic_sub", Image, self.image_callback
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Could not convert image: %s", e)
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)

        lower_bound = np.array([self.lh, self.ls, self.lv])
        upper_bound = np.array([self.hh, self.hs, self.hv])

        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        mask = cv2.erode(mask, kernel, iterations=3)
        mask = cv2.dilate(mask, kernel, iterations=3)

        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            self.image_pub.publish(mask_msg)
        except CvBridgeError as e:
            rospy.logerr("Could not convert mask image: %s", e)

    def run(self):
        rospy.loginfo("Lane Filterer Node Started")
        rospy.spin()


if __name__ == "__main__":
    try:
        lane_filterer = LaneFilterer()
        lane_filterer.run()
    except rospy.ROSInterruptException:
        pass
