#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class PointCloudTransformer : public rclcpp::Node {
public:
  PointCloudTransformer() : Node("point_cloud_transformer") {
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "zed/zed_node/point_cloud/cloud_registered", 1,
      std::bind(&PointCloudTransformer::pointcloudCallback, this, std::placeholders::_1));

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("odom/point_cloud", 1);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform("odom", cloud_msg->header.frame_id, rclcpp::Time(0),
                                       rclcpp::Duration::from_seconds(1.0));

      sensor_msgs::msg::PointCloud2 transformed_cloud;
      tf2::doTransform(*cloud_msg, transformed_cloud, transform_stamped);

      pc_pub_->publish(transformed_cloud);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
