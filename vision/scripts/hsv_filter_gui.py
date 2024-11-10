#!/usr/bin/python3

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class HSVFilterNode:
    def __init__(self):
        rospy.init_node("hsv_gui_node", anonymous=True)
        rospy.loginfo("HSV Filter Node initialized")

        self.bridge = CvBridge()

        self.lower_hsv = np.array([0, 0, 0])
        self.upper_hsv = np.array([179, 255, 255])

        cv2.namedWindow("HSV Controls", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("HSV Controls", 400, 300)
        self.create_trackbars()

        self.filtered_pub = rospy.Publisher("/hsv_filtered_image", Image, queue_size=10)

        rospy.sleep(1)

        self.image_sub = rospy.Subscriber(
            "/zed_node/rgb/image_rect_color",
            Image,
            self.image_callback,
            queue_size=10,
            buff_size=2**24,
        )
        rospy.loginfo("Subscribed to image topic")

        self.image_received = False

    def create_trackbars(self):

        cv2.createTrackbar("L_H", "HSV Controls", 0, 179, self.nothing)
        cv2.createTrackbar("L_S", "HSV Controls", 0, 255, self.nothing)
        cv2.createTrackbar("L_V", "HSV Controls", 0, 255, self.nothing)

        cv2.createTrackbar("U_H", "HSV Controls", 179, 179, self.nothing)
        cv2.createTrackbar("U_S", "HSV Controls", 255, 255, self.nothing)
        cv2.createTrackbar("U_V", "HSV Controls", 255, 255, self.nothing)

    def nothing(self, x):
        pass

    def update_hsv_values(self):
        self.lower_hsv = np.array(
            [
                cv2.getTrackbarPos("L_H", "HSV Controls"),
                cv2.getTrackbarPos("L_S", "HSV Controls"),
                cv2.getTrackbarPos("L_V", "HSV Controls"),
            ]
        )

        self.upper_hsv = np.array(
            [
                cv2.getTrackbarPos("U_H", "HSV Controls"),
                cv2.getTrackbarPos("U_S", "HSV Controls"),
                cv2.getTrackbarPos("U_V", "HSV Controls"),
            ]
        )

    def image_callback(self, msg):
        try:
            self.image_received = True

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            self.update_hsv_values()

            mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)

            filtered_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            filtered_msg = self.bridge.cv2_to_imgmsg(filtered_image, "bgr8")
            filtered_msg.header.stamp = rospy.Time.now()
            filtered_msg.header.frame_id = msg.header.frame_id
            self.filtered_pub.publish(filtered_msg)

            rospy.loginfo_throttle(5, "Publishing filtered image")
            rospy.loginfo_throttle(
                5, f"HSV values - Lower: {self.lower_hsv}, Upper: {self.upper_hsv}"
            )

        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if not self.image_received:
                rospy.logwarn_throttle(5, "No images received yet!")

            topics = rospy.get_published_topics()
            filtered_topics = [t for t, _ in topics if t == "/hsv_filtered_image"]
            if not filtered_topics:
                rospy.logwarn_throttle(
                    5, "Filtered image topic is not being published!"
                )

            cv2.waitKey(1)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = HSVFilterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
