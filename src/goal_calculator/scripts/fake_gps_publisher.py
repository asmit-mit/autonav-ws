#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class GoalPublisher:
    def __init__(self):
        rospy.init_node("fake_gps_publisher")

        self.goals = [
            (11.722544846516783, 20.963197274850558, 1.0),
            (6.1009241092590925, 10.276119492082477, 2.0),
            (-6.080009506543892, 10.12216460955621, 3.0),
            (-12.753273858654046, 21.868191619181516, 4.0),
            (0, 0, 5.0),
        ]
        self.current_goal_index = 0

        self.goal_pub = rospy.Publisher(
            "/nav/gps_goal", PoseStamped, queue_size=10
        )
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.rate = rospy.Rate(5)  # 5 Hz

    def publish_current_goal(self):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "odom"
        goal_msg.pose.position.x = self.goals[self.current_goal_index][0]
        goal_msg.pose.position.y = self.goals[self.current_goal_index][1]
        goal_msg.pose.position.z = self.goals[self.current_goal_index][2]
        self.goal_pub.publish(goal_msg)

    def odom_callback(self, odom_msg):
        current_x = odom_msg.pose.pose.position.x
        current_y = odom_msg.pose.pose.position.y

        goal_x, goal_y, goal_z = self.goals[self.current_goal_index]

        distance = math.sqrt(
            (goal_x - current_x) ** 2 + (goal_y - current_y) ** 2
        )

        if distance < 1.0:
            self.current_goal_index = (self.current_goal_index + 1) % len(
                self.goals
            )
            rospy.loginfo(
                "Switching to next goal: {}".format(
                    self.goals[self.current_goal_index]
                )
            )

    def run(self):
        while not rospy.is_shutdown():
            self.publish_current_goal()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        goal_publisher = GoalPublisher()
        goal_publisher.run()
    except rospy.ROSInterruptException:
        pass
