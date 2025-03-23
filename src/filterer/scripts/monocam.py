#!/usr/bin/python3

import argparse
import sys

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraPublisher:
    def __init__(self, camera_id):
        # Initialize the node
        rospy.init_node("camera_publisher", anonymous=True)

        # Initialize publisher
        self.image_pub = rospy.Publisher("/mono/rgb/image", Image, queue_size=10)

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Open camera
        self.camera = cv2.VideoCapture(camera_id)

        # Check if camera opened successfully
        if not self.camera.isOpened():
            rospy.logerr(f"Error: Could not open camera {camera_id}")
            sys.exit(1)

        # Set publishing rate (30 Hz)
        self.rate = rospy.Rate(30)

        rospy.loginfo(f"Started camera publisher with camera ID: {camera_id}")
        rospy.loginfo("Publishing on topic: /mono/rgb/image")

    def publish_frames(self):
        while not rospy.is_shutdown():
            # Read frame
            ret, frame = self.camera.read()

            if ret:
                try:
                    # Convert OpenCV image to ROS message
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")

                    # Publish the image
                    self.image_pub.publish(ros_image)

                except Exception as e:
                    rospy.logerr(f"Error converting/publishing image: {str(e)}")
            else:
                rospy.logerr("Failed to capture frame")
                break

            self.rate.sleep()

        # Release camera when done
        self.camera.release()


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Publish camera feed to ROS topic")
    parser.add_argument(
        "camera_id", type=int, help="Camera ID (usually 0 for built-in webcam)"
    )

    args = parser.parse_args()

    try:
        # Create and run the camera publisher
        publisher = CameraPublisher(args.camera_id)
        publisher.publish_frames()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
