#!/usr/bin/python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class HSVFilterNode(Node):
    def __init__(self):
        super().__init__("hsv_gui_node")
        self.get_logger().info("HSV Filter Node initialized")

        self.bridge = CvBridge()

        self.lower_hsv = np.array([0, 0, 0])
        self.upper_hsv = np.array([179, 255, 255])

        cv2.namedWindow("HSV Controls", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("HSV Controls", 400, 300)
        self.create_trackbars()

        self.filtered_pub = self.create_publisher(Image, "/hsv_filtered_image", 10)

        self.image_received = False

        self.image_sub = self.create_subscription(
            Image,
            "zed/zed_node/rgb/image_rect_color",
            self.image_callback,
            10
        )
        self.get_logger().info("Subscribed to image topic")

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
            filtered_msg.header.stamp = self.get_clock().now().to_msg()
            filtered_msg.header.frame_id = msg.header.frame_id
            self.filtered_pub.publish(filtered_msg)

            self.get_logger().info(f"HSV values - Lower: {self.lower_hsv}, Upper: {self.upper_hsv}")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def run(self):
        rate = self.create_rate(30)

        while rclpy.ok():
            if not self.image_received:
                self.get_logger().warn("No images received yet!")

            topics = self.get_topic_names_and_types()
            filtered_topics = [t for t, _ in topics if t == "/hsv_filtered_image"]
            if not filtered_topics:
                self.get_logger().warn("Filtered image topic is not being published!")

            cv2.waitKey(1)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = HSVFilterNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
