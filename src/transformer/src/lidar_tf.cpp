#include "ros/publisher.h"
#include "ros/time.h"
#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub_pointcloud;
ros::Publisher pub_laserscan;
ros::Publisher pub_imu;
ros::Time last_pointcloud_time;
ros::Time last_laserscan_time;
ros::Time last_imu_time;
bool received_pointcloud      = false;
bool received_laserscan       = false;
bool received_imu             = false;
const double TIMEOUT_DURATION = 1.0;

const int IMU_WINDOW_SIZE = 30;

std::deque<geometry_msgs::Vector3> linear_acc_window;
std::deque<geometry_msgs::Vector3> angular_vel_window;
std::deque<geometry_msgs::Quaternion> orientation_window;

struct Vector3Sum {
  double x = 0;
  double y = 0;
  double z = 0;
};

struct QuaternionSum {
  double x = 0;
  double y = 0;
  double z = 0;
  double w = 0;
};

Vector3Sum linear_acc_sum;
Vector3Sum angular_vel_sum;
QuaternionSum orientation_sum;

geometry_msgs::Quaternion
getAveragedQuaternion(const QuaternionSum &sum, int size) {
  geometry_msgs::Quaternion avg;
  avg.x = sum.x / size;
  avg.y = sum.y / size;
  avg.z = sum.z / size;
  avg.w = sum.w / size;

  double norm =
      sqrt(avg.x * avg.x + avg.y * avg.y + avg.z * avg.z + avg.w * avg.w);
  avg.x /= norm;
  avg.y /= norm;
  avg.z /= norm;
  avg.w /= norm;

  return avg;
}

geometry_msgs::Vector3 getAveragedVector3(const Vector3Sum &sum, int size) {
  geometry_msgs::Vector3 avg;
  avg.x = sum.x / size;
  avg.y = sum.y / size;
  avg.z = sum.z / size;
  return avg;
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  received_pointcloud              = true;
  last_pointcloud_time             = ros::Time::now();
  sensor_msgs::PointCloud2 new_msg = *msg;
  new_msg.header.stamp             = ros::Time::now();
  pub_pointcloud.publish(new_msg);
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  received_laserscan             = true;
  last_laserscan_time            = ros::Time::now();
  sensor_msgs::LaserScan new_msg = *msg;
  new_msg.header.stamp           = ros::Time::now();
  pub_laserscan.publish(new_msg);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  received_imu  = true;
  last_imu_time = ros::Time::now();

  linear_acc_sum.x += msg->linear_acceleration.x;
  linear_acc_sum.y += msg->linear_acceleration.y;
  linear_acc_sum.z += msg->linear_acceleration.z;

  angular_vel_sum.x += msg->angular_velocity.x;
  angular_vel_sum.y += msg->angular_velocity.y;
  angular_vel_sum.z += msg->angular_velocity.z;

  orientation_sum.x += msg->orientation.x;
  orientation_sum.y += msg->orientation.y;
  orientation_sum.z += msg->orientation.z;
  orientation_sum.w += msg->orientation.w;

  linear_acc_window.push_back(msg->linear_acceleration);
  angular_vel_window.push_back(msg->angular_velocity);
  orientation_window.push_back(msg->orientation);

  if (linear_acc_window.size() > IMU_WINDOW_SIZE) {
    const auto &old_linear = linear_acc_window.front();
    linear_acc_sum.x -= old_linear.x;
    linear_acc_sum.y -= old_linear.y;
    linear_acc_sum.z -= old_linear.z;

    const auto &old_angular = angular_vel_window.front();
    angular_vel_sum.x -= old_angular.x;
    angular_vel_sum.y -= old_angular.y;
    angular_vel_sum.z -= old_angular.z;

    const auto &old_orientation = orientation_window.front();
    orientation_sum.x -= old_orientation.x;
    orientation_sum.y -= old_orientation.y;
    orientation_sum.z -= old_orientation.z;
    orientation_sum.w -= old_orientation.w;

    linear_acc_window.pop_front();
    angular_vel_window.pop_front();
    orientation_window.pop_front();
  }

  sensor_msgs::Imu new_msg = *msg;
  new_msg.header.stamp     = ros::Time::now();

  if (linear_acc_window.size() >= 3) {
    int window_size = linear_acc_window.size();
    new_msg.linear_acceleration =
        getAveragedVector3(linear_acc_sum, window_size);
    new_msg.angular_velocity = getAveragedVector3(angular_vel_sum, window_size);
    new_msg.orientation = getAveragedQuaternion(orientation_sum, window_size);
  }

  pub_imu.publish(new_msg);
}

void checkDataReception(const ros::TimerEvent &) {
  ros::Time current_time = ros::Time::now();
  if (!received_pointcloud) {
    ROS_WARN_THROTTLE(
        5.0, "[lidar_tf] No PointCloud data has been received yet on topic "
             "/ouster/points"
    );
  } else if ((current_time - last_pointcloud_time).toSec() > TIMEOUT_DURATION) {
    ROS_WARN_THROTTLE(
        5.0, "[lidar_tf] No PointCloud data received for %.1f seconds",
        (current_time - last_pointcloud_time).toSec()
    );
  }
  if (!received_laserscan) {
    ROS_WARN_THROTTLE(
        5.0, "[lidar_tf] No LaserScan data has been received yet on topic "
             "/ouster/scan"
    );
  } else if ((current_time - last_laserscan_time).toSec() > TIMEOUT_DURATION) {
    ROS_WARN_THROTTLE(
        5.0, "[lidar_tf] No LaserScan data received for %.1f seconds",
        (current_time - last_laserscan_time).toSec()
    );
  }
  if (!received_imu) {
    ROS_WARN_THROTTLE(
        5.0, "[lidar_tf] No IMU data has been received yet on topic /ouster/imu"
    );
  } else if ((current_time - last_imu_time).toSec() > TIMEOUT_DURATION) {
    ROS_WARN_THROTTLE(
        5.0, "[lidar_tf] No IMU data received for %.1f seconds",
        (current_time - last_imu_time).toSec()
    );
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_tf");
  ros::NodeHandle nh;

  pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/lidar/points", 10);
  pub_laserscan  = nh.advertise<sensor_msgs::LaserScan>("/lidar/scan", 10);
  pub_imu        = nh.advertise<sensor_msgs::Imu>("/lidar/imu", 10);

  ros::Subscriber sub_pointcloud =
      nh.subscribe("/ouster/points", 10, pointCloudCallback);
  ros::Subscriber sub_laserscan =
      nh.subscribe("/ouster/scan", 10, laserScanCallback);
  ros::Subscriber sub_imu = nh.subscribe("/ouster/imu", 10, imuCallback);

  ros::Timer timer = nh.createTimer(ros::Duration(0.5), checkDataReception);

  ROS_INFO(
      "[lidar_tf] Lidar TF node started with IMU smoothing. Window size: %d",
      IMU_WINDOW_SIZE
  );
  ros::spin();
  return 0;
}
