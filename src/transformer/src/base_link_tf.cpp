#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class BaseFrameCorrector {
private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher corrected_odom_pub_;

  geometry_msgs::Quaternion ground_orientation_;
  double reference_height_;
  bool imu_received_;
  bool odom_received_;
  nav_msgs::Odometry latest_odom_;

public:
  BaseFrameCorrector()
      : tf_listener_(tf_buffer_), imu_received_(false), odom_received_(false),
        reference_height_(0.0) {

    imu_sub_ =
        nh_.subscribe("/lidar/imu", 10, &BaseFrameCorrector::imuCallback, this);
    odom_sub_ =
        nh_.subscribe("dlo/odom", 10, &BaseFrameCorrector::odomCallback, this);

    // Publisher for corrected odometry
    corrected_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

    ground_orientation_.x = 0.0;
    ground_orientation_.y = 0.0;
    ground_orientation_.z = 0.0;
    ground_orientation_.w = 1.0;

    ROS_INFO("[base_link_tf] Base frame corrector node initialized");
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    tf2::Quaternion imu_quat;
    tf2::fromMsg(msg->orientation, imu_quat);

    double roll, pitch, yaw;
    tf2::Matrix3x3(imu_quat).getRPY(roll, pitch, yaw);

    tf2::Quaternion ground_quat;
    ground_quat.setRPY(0.0, 0.0, yaw);

    ground_orientation_ = tf2::toMsg(ground_quat);
    imu_received_       = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    reference_height_ = msg->pose.pose.position.z;
    latest_odom_      = *msg; // Store the latest odometry message
    odom_received_    = true;
  }

  void publishCorrectedOdometry() {
    if (!imu_received_ || !odom_received_) {
      return;
    }

    // Create corrected odometry message
    nav_msgs::Odometry corrected_odom = latest_odom_;

    // Update header
    corrected_odom.header.stamp    = ros::Time::now();
    corrected_odom.header.frame_id = "odom";
    corrected_odom.child_frame_id  = "base_link";

    // Keep original X and Y position, set Z to reference height
    corrected_odom.pose.pose.position.z = reference_height_;

    // Use ground-parallel orientation from IMU
    corrected_odom.pose.pose.orientation = ground_orientation_;

    // Zero out roll and pitch components of twist (angular velocity)
    // Keep only yaw component for angular velocity
    tf2::Quaternion twist_quat;
    tf2::fromMsg(latest_odom_.pose.pose.orientation, twist_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(twist_quat).getRPY(roll, pitch, yaw);

    // Assuming the angular velocity around z-axis corresponds to yaw rate
    corrected_odom.twist.twist.angular.x = 0.0; // No roll rate
    corrected_odom.twist.twist.angular.y = 0.0; // No pitch rate
    // Keep original yaw rate (angular.z)

    // Publish corrected odometry
    corrected_odom_pub_.publish(corrected_odom);
  }

  void publishCorrectedBaseLink() {
    if (!imu_received_ || !odom_received_) {
      return;
    }

    try {
      geometry_msgs::TransformStamped odom_to_base_transform;
      odom_to_base_transform = tf_buffer_.lookupTransform(
          "odom", "robot/base_link", ros::Time(0), ros::Duration(0.1)
      );

      geometry_msgs::TransformStamped corrected_transform;
      corrected_transform.header.stamp    = ros::Time::now();
      corrected_transform.header.frame_id = "odom";
      corrected_transform.child_frame_id  = "base_link";

      corrected_transform.transform.translation.x =
          odom_to_base_transform.transform.translation.x;
      corrected_transform.transform.translation.y =
          odom_to_base_transform.transform.translation.y;

      corrected_transform.transform.translation.z = reference_height_;

      corrected_transform.transform.rotation = ground_orientation_;

      tf_broadcaster_.sendTransform(corrected_transform);

    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(
          1.0,
          "[base_link_tf] Could not get transform from odom to "
          "robot/base_link: %s",
          ex.what()
      );
    }
  }

  void spin() {
    ros::Rate rate(50); // 50 Hz

    while (ros::ok()) {
      ros::spinOnce();
      publishCorrectedBaseLink();
      publishCorrectedOdometry();
      rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_link_tf");

  BaseFrameCorrector corrector;
  corrector.spin();

  return 0;
}
