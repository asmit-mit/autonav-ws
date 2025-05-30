#!/usr/bin/env python

import signal
import sys

import rospy
from geometry_msgs.msg import Twist


class CmdVelMonitor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("cmd_vel_monitor", anonymous=True)

        # Previous values for acceleration calculation
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0
        self.prev_time = rospy.Time.now()

        # Maximum values tracking
        self.max_linear_vel = 0.0
        self.max_angular_vel = 0.0
        self.max_linear_accel = 0.0
        self.max_angular_accel = 0.0

        # For average calculations
        self.total_linear_vel = 0.0
        self.total_angular_vel = 0.0
        self.total_linear_accel = 0.0
        self.total_angular_accel = 0.0
        self.sample_count = 0

        # Subscribe to cmd_vel topic
        self.subscriber = rospy.Subscriber(
            "/cmd_vel", Twist, self.cmd_vel_callback
        )

        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.loginfo(
            "CMD_VEL Monitor started. Listening to /cmd_vel topic..."
        )

    def cmd_vel_callback(self, msg):
        # Get current time
        current_time = rospy.Time.now()

        # Calculate time difference
        dt = (current_time - self.prev_time).to_sec()

        # Extract current velocities
        linear_vel = abs(msg.linear.x)  # Use absolute value for max tracking
        angular_vel = abs(msg.angular.z)  # Use absolute value for max tracking

        # Calculate accelerations (only if dt > 0 to avoid division by zero)
        if dt > 0:
            linear_accel = abs((msg.linear.x - self.prev_linear_vel) / dt)
            angular_accel = abs((msg.angular.z - self.prev_angular_vel) / dt)
        else:
            linear_accel = 0.0
            angular_accel = 0.0

        # Update maximum values
        self.max_linear_vel = max(self.max_linear_vel, linear_vel)
        self.max_angular_vel = max(self.max_angular_vel, angular_vel)
        self.max_linear_accel = max(self.max_linear_accel, linear_accel)
        self.max_angular_accel = max(self.max_angular_accel, angular_accel)

        # Update totals for average calculation
        self.total_linear_vel += linear_vel
        self.total_angular_vel += angular_vel
        self.total_linear_accel += linear_accel
        self.total_angular_accel += angular_accel
        self.sample_count += 1

        # Display real-time data
        print("=" * 70)
        print("REAL-TIME CMD_VEL MONITORING")
        print("=" * 70)
        print("Linear Velocity (X):     {:.4f} m/s".format(msg.linear.x))
        print("Angular Velocity (Z):    {:.4f} rad/s".format(msg.angular.z))
        print("Linear Acceleration:     {:.4f} m/s²".format(linear_accel))
        print("Angular Acceleration:    {:.4f} rad/s²".format(angular_accel))
        print("-" * 70)
        print("**MAXIMUM VALUES**")
        print(
            "Max Linear Velocity:     {:.4f} m/s".format(self.max_linear_vel)
        )
        print(
            "Max Angular Velocity:    {:.4f} rad/s".format(
                self.max_angular_vel
            )
        )
        print(
            "Max Linear Acceleration: {:.4f} m/s²".format(
                self.max_linear_accel
            )
        )
        print(
            "Max Angular Acceleration:{:.4f} rad/s²".format(
                self.max_angular_accel
            )
        )
        print("-" * 70)
        print("Time Delta:              {:.4f} s".format(dt))
        print("Timestamp:               {:.4f}".format(current_time.to_sec()))
        print("Sample Count:            {}".format(self.sample_count))
        print("\n")

        # Update previous values (store original signed values)
        self.prev_linear_vel = msg.linear.x
        self.prev_angular_vel = msg.angular.z
        self.prev_time = current_time

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully and show statistics"""
        self.show_final_statistics()
        sys.exit(0)

    def show_final_statistics(self):
        """Display final statistics when program exits"""
        print("\n" + "=" * 70)
        print("FINAL STATISTICS - CMD_VEL MONITOR")
        print("=" * 70)

        if self.sample_count > 0:
            avg_linear_vel = self.total_linear_vel / self.sample_count
            avg_angular_vel = self.total_angular_vel / self.sample_count
            avg_linear_accel = self.total_linear_accel / self.sample_count
            avg_angular_accel = self.total_angular_accel / self.sample_count

            print("**MAXIMUM VALUES REACHED**")
            print(
                "Max Linear Velocity:     {:.4f} m/s".format(
                    self.max_linear_vel
                )
            )
            print(
                "Max Angular Velocity:    {:.4f} rad/s".format(
                    self.max_angular_vel
                )
            )
            print(
                "Max Linear Acceleration: {:.4f} m/s²".format(
                    self.max_linear_accel
                )
            )
            print(
                "Max Angular Acceleration:{:.4f} rad/s²".format(
                    self.max_angular_accel
                )
            )
            print()
            print("**AVERAGE VALUES**")
            print("Avg Linear Velocity:     {:.4f} m/s".format(avg_linear_vel))
            print(
                "Avg Angular Velocity:    {:.4f} rad/s".format(avg_angular_vel)
            )
            print(
                "Avg Linear Acceleration: {:.4f} m/s²".format(avg_linear_accel)
            )
            print(
                "Avg Angular Acceleration:{:.4f} rad/s²".format(
                    avg_angular_accel
                )
            )
            print()
            print("Total Samples Processed: {}".format(self.sample_count))
        else:
            print("No data samples were collected.")

        print("=" * 70)
        rospy.loginfo("CMD_VEL Monitor shutting down...")


if __name__ == "__main__":
    try:
        monitor = CmdVelMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        monitor.show_final_statistics()
