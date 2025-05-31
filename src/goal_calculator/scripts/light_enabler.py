#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class OdomBoolPublisher:
    def __init__(self):
        rospy.init_node("odom_bool_publisher", anonymous=True)

        self.bool_pub = rospy.Publisher(
            "/sisyphus/autonomous", Bool, queue_size=10
        )

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        rospy.on_shutdown(self.shutdown_hook)

        initial_msg = Bool()
        initial_msg.data = False
        self.bool_pub.publish(initial_msg)

        rospy.loginfo("OdomBoolPublisher initialized")

    def odom_callback(self, msg):
        """Callback function for odometry messages"""
        # Publish true when receiving odometry data
        bool_msg = Bool()
        bool_msg.data = True
        self.bool_pub.publish(bool_msg)

        rospy.logdebug("Published true to /odom_status")

    def shutdown_hook(self):
        """Function called when node is shutting down"""
        rospy.loginfo("Shutdown requested, publishing false...")

        # Publish false message before shutdown
        bool_msg = Bool()
        bool_msg.data = False
        self.bool_pub.publish(bool_msg)

        # Give time for message to be sent
        rospy.sleep(0.1)

        rospy.loginfo("Shutdown complete")


def main():
    try:
        # Create the publisher object
        odom_bool_pub = OdomBoolPublisher()

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
