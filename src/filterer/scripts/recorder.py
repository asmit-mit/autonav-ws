#!/usr/bin/python3

import datetime
import threading

import rosbag
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, PointCloud2
from tf2_msgs.msg import TFMessage


class BagRecorder:
    def __init__(self):
        rospy.init_node("bag_recorder", anonymous=True)

        # Create timestamp for filename
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.bag_filename = f"recording_{timestamp}.bag"

        # Initialize ROS bag
        self.bag = rosbag.Bag(self.bag_filename, "w")

        # Lock for thread-safe writing
        self.write_lock = threading.Lock()

        # Initialize subscribers
        self.subscribers = [
            rospy.Subscriber(
                "/lidar/points", PointCloud2, self.lidar_callback, queue_size=10
            ),
            rospy.Subscriber(
                "/mono/rgb/image",
                Image,
                self.rgb_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/zed_node/depth/depth_registered",
                Image,
                self.depth_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/robot/dlo/odom_node/odom", Odometry, self.odom_callback, queue_size=10
            ),
            rospy.Subscriber("/tf", TFMessage, self.tf_callback, queue_size=10),
        ]

        self.rate = rospy.Rate(15)  # 5 Hz recording rate

    def write_message(self, topic, msg):
        with self.write_lock:
            try:
                self.bag.write(topic, msg, rospy.Time.now())
            except Exception as e:
                rospy.logerr(f"Error writing message on topic {topic}: {str(e)}")

    def lidar_callback(self, msg):
        self.write_message("/lidar/points", msg)

    def rgb_callback(self, msg):
        self.write_message("/mono/rgb/image", msg)

    def depth_callback(self, msg):
        self.write_message("/zed_node/depth/depth_registered", msg)

    def odom_callback(self, msg):
        self.write_message("/robot/dlo/odom_node/odom", msg)

    def tf_callback(self, msg):
        self.write_message("/tf", msg)

    def record(self):
        rospy.loginfo("Started recording to: %s", self.bag_filename)

        try:
            while not rospy.is_shutdown():
                self.rate.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("Stopping recording...")

        finally:
            with self.write_lock:
                self.bag.close()
            rospy.loginfo("Recording saved to: %s", self.bag_filename)


if __name__ == "__main__":
    try:
        recorder = BagRecorder()
        recorder.record()
    except rospy.ROSInterruptException:
        pass
