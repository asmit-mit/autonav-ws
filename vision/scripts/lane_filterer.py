#!/usr/bin/python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class LaneFilterer(Node):
    def __init__(self):
        super().__init__("lane_filter_node")

        # Declare and retrieve parameters
        self.declare_parameter("higher_h", 180)
        self.declare_parameter("higher_s", 255)
        self.declare_parameter("higher_v", 255)
        self.declare_parameter("lower_h", 0)
        self.declare_parameter("lower_s", 0)
        self.declare_parameter("lower_v", 0)
        self.declare_parameter("kernel_size", 3)
        self.declare_parameter("iterations", 3)

        self.hh = self.get_parameter("higher_h").get_parameter_value().integer_value
        self.hs = self.get_parameter("higher_s").get_parameter_value().integer_value
        self.hv = self.get_parameter("higher_v").get_parameter_value().integer_value
        self.lh = self.get_parameter("lower_h").get_parameter_value().integer_value
        self.ls = self.get_parameter("lower_s").get_parameter_value().integer_value
        self.lv = self.get_parameter("lower_v").get_parameter_value().integer_value
        self.kernel_size = self.get_parameter("kernel_size").get_parameter_value().integer_value
        self.iterations = self.get_parameter("iterations").get_parameter_value().integer_value

        self.image_pub = self.create_publisher(Image, "mask_topic_pub", 10)
        self.image_sub = self.create_subscription(
            Image, "rgb_image_topic_sub", self.image_callback, 10
        )

        self.bridge = CvBridge()

        self.get_logger().info("Lane Filterer Node Started")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error("Could not convert image: %s" % e)
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)

        lower_bound = np.array([self.lh, self.ls, self.lv])
        upper_bound = np.array([self.hh, self.hs, self.hv])

        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        mask = cv2.erode(mask, kernel, iterations=self.iterations)
        mask = cv2.dilate(mask, kernel, iterations=self.iterations)

        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            self.image_pub.publish(mask_msg)
        except CvBridgeError as e:
            self.get_logger().error("Could not convert mask image: %s" % e)

def main(args=None):
    rclpy.init(args=args)
    lane_filterer = LaneFilterer()
    rclpy.spin(lane_filterer)
    lane_filterer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
