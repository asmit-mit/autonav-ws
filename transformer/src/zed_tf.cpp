#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class ZedTfNode : public rclcpp::Node {
public:
  ZedTfNode() : Node("zed_tf"), tf_broadcaster_(this) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/zed_node/imu/data", 10,
      std::bind(&ZedTfNode::imuCallBack, this, std::placeholders::_1));
  }

private:
  void imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = "robot/base_link";
    transform_stamped.child_frame_id = "zed2i_base_link";

    // Extract orientation from the IMU message
    tf2::Quaternion imu_orientation(msg->orientation.x, msg->orientation.y,
                                    msg->orientation.z, msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(imu_orientation).getRPY(roll, pitch, yaw);

    // Set roll and yaw to zero
    yaw = 0;
    roll = 0;
    pitch = 0;

    tf2::Quaternion corrected_orientation;
    corrected_orientation.setRPY(roll, pitch, yaw);

    // Set the transform's rotation
    transform_stamped.transform.rotation.x = corrected_orientation.x();
    transform_stamped.transform.rotation.y = corrected_orientation.y();
    transform_stamped.transform.rotation.z = corrected_orientation.z();
    transform_stamped.transform.rotation.w = corrected_orientation.w();

    // Set the transform's translation
    transform_stamped.transform.translation.x = -0.114;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 1.32;

    // Broadcast the transform
    tf_broadcaster_.sendTransform(transform_stamped);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedTfNode>());
  rclcpp::shutdown();
  return 0;
}
