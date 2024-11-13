#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        # Create a publisher to publish Image messages
        self.publisher_ = self.create_publisher(Image, 'monocular_camera/image_raw', 10)
        # Use OpenCV to access the webcam
        self.cap = cv2.VideoCapture("/dev/video4")
        self.bridge = CvBridge()
        # Define a timer callback to capture and publish frames at a set rate
        self.timer = self.create_timer(1/100, self.timer_callback)  # Adjust the timer for desired frame rate
        self.get_logger().info("Webcam Publisher Node has started")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert the frame (numpy array) to an Image message and publish
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_message)
            self.get_logger().info("Published a webcam frame")
        else:
            self.get_logger().warning("Failed to capture frame from webcam")

    def destroy_node(self):
        super().destroy_node()
        # Release the webcam when the node is destroyed
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        webcam_publisher.get_logger().info("Shutting down Webcam Publisher Node")
    finally:
        webcam_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
