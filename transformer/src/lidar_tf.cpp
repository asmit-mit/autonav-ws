#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarTfNode : public rclcpp::Node {
public:
  LidarTfNode() : Node("lidar_tf") {
    pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points", 10);
    pub_laserscan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/lidar/scan", 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/lidar/imu", 10);

    sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster/points", 10, std::bind(&LidarTfNode::pointCloudCallback, this, std::placeholders::_1));

    sub_laserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/ouster/scan", 10, std::bind(&LidarTfNode::laserScanCallback, this, std::placeholders::_1));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/ouster/imu", 10, std::bind(&LidarTfNode::imuCallback, this, std::placeholders::_1));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2 new_msg = *msg;
    new_msg.header.stamp = this->now(); // Set current time

    pub_pointcloud_->publish(new_msg);
  }

  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    sensor_msgs::msg::LaserScan new_msg = *msg;
    new_msg.header.stamp = this->now(); // Set current time

    pub_laserscan_->publish(new_msg);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    sensor_msgs::msg::Imu new_msg = *msg;
    new_msg.header.stamp = this->now(); // Set current time

    pub_imu_->publish(new_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laserscan_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarTfNode>());
  rclcpp::shutdown();
  return 0;
}
