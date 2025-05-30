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
        self.target_v = rospy.get_param("~target_v", 140)

        self.image_pub = rospy.Publisher("mask_topic_pub", Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            "rgb_image_topic_sub", Image, self.image_callback
        )

        self.bridge = CvBridge()

    def make_image_brighter(self, image, target_v):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_image)
        avg_v = np.mean(v)
        scale_factor = target_v / avg_v if avg_v > 0 else 1
        v = np.clip(v * scale_factor, 0, 255).astype(np.uint8)
        adjusted_hsv = cv2.merge((h, s, v))
        brighter_image = cv2.cvtColor(adjusted_hsv, cv2.COLOR_HSV2BGR)

        return brighter_image

    def image_callback(self, msg):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("[filterer] Could not convert image: %s", e)
            return

        brighter_bgr_image = self.make_image_brighter(cv_image, self.target_v)

        # gray_image = cv2.cvtColor(brighter_bgr_image, cv2.COLOR_BGR2GRAY)
        # # blur_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        # canny_image = cv2.Canny(gray_image, 50, 150)
        #
        # kernel = np.ones((1,1), np.uint8)
        # # canny_image = cv2.dilate(canny_image, kernel=kernel, iterations=1)
        # canny_image = cv2.erode(canny_image, kernel=kernel, iterations=0)
        #
        #
        # contours, _ = cv2.findContours(canny_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # min_area = 50
        # filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        #
        # filtered_contour_mask = np.zeros_like(canny_image)
        # cv2.drawContours(filtered_contour_mask, filtered_contours, -1, 255, thickness=cv2.FILLED)       # contours,_ = cv2.findContours(canny_image, )

        hsv_image = cv2.cvtColor(brighter_bgr_image, cv2.COLOR_BGR2HSV)

        lower_white = np.array([self.lh, self.ls, self.lv])
        upper_white = np.array([self.hh, self.hs, self.hv])
        mask = cv2.inRange(hsv_image, lower_white, upper_white)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel=kernel, iterations=1)
        mask = cv2.dilate(mask, kernel=kernel, iterations=1)

        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            self.image_pub.publish(mask_msg)
        except CvBridgeError as e:
            rospy.logerr("[filterer] Could not convert mask image: %s", e)

    def run(self):
        rospy.loginfo("[filterer] Lane Filterer Node Started")
        rospy.spin()


if __name__ == "__main__":
    try:
        lane_filterer = LaneFilterer()
        lane_filterer.run()
    except rospy.ROSInterruptException:
        pass
