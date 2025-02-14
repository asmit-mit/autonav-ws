#include "ros/publisher.h"
#include "ros/time.h"
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

bool received_pointcloud = false;
bool received_laserscan = false;
bool received_imu = false;

const double TIMEOUT_DURATION = 1.0;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  received_pointcloud = true;
  last_pointcloud_time = ros::Time::now();

  sensor_msgs::PointCloud2 new_msg = *msg;
  new_msg.header.stamp = ros::Time::now();
  pub_pointcloud.publish(new_msg);
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  received_laserscan = true;
  last_laserscan_time = ros::Time::now();

  sensor_msgs::LaserScan new_msg = *msg;
  new_msg.header.stamp = ros::Time::now();
  pub_laserscan.publish(new_msg);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  received_imu = true;
  last_imu_time = ros::Time::now();

  sensor_msgs::Imu new_msg = *msg;
  new_msg.header.stamp = ros::Time::now();
  pub_imu.publish(new_msg);
}

void checkDataReception(const ros::TimerEvent &) {
  ros::Time current_time = ros::Time::now();

  if (!received_pointcloud) {
    ROS_WARN_THROTTLE(
        5.0,
        "No PointCloud data has been received yet on topic /ouster/points");
  } else if ((current_time - last_pointcloud_time).toSec() > TIMEOUT_DURATION) {
    ROS_WARN_THROTTLE(5.0, "No PointCloud data received for %.1f seconds",
                      (current_time - last_pointcloud_time).toSec());
  }

  if (!received_laserscan) {
    ROS_WARN_THROTTLE(
        5.0, "No LaserScan data has been received yet on topic /ouster/scan");
  } else if ((current_time - last_laserscan_time).toSec() > TIMEOUT_DURATION) {
    ROS_WARN_THROTTLE(5.0, "No LaserScan data received for %.1f seconds",
                      (current_time - last_laserscan_time).toSec());
  }

  if (!received_imu) {
    ROS_WARN_THROTTLE(5.0,
                      "No IMU data has been received yet on topic /ouster/imu");
  } else if ((current_time - last_imu_time).toSec() > TIMEOUT_DURATION) {
    ROS_WARN_THROTTLE(5.0, "No IMU data received for %.1f seconds",
                      (current_time - last_imu_time).toSec());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_tf");
  ros::NodeHandle nh;

  pub_pointcloud =
      nh.advertise<sensor_msgs::PointCloud2>("/ouster/points2", 10);
  pub_laserscan = nh.advertise<sensor_msgs::LaserScan>("/ouster/scan2", 10);
  pub_imu = nh.advertise<sensor_msgs::Imu>("ouster/imu2", 10);

  ros::Subscriber sub_pointcloud =
      nh.subscribe("/ouster/points", 10, pointCloudCallback);
  ros::Subscriber sub_laserscan =
      nh.subscribe("/ouster/scan", 10, laserScanCallback);
  ros::Subscriber sub_imu = nh.subscribe("/ouster/imu", 10, imuCallback);

  ros::Timer timer = nh.createTimer(ros::Duration(0.5), checkDataReception);

  ROS_INFO("Lidar TF node started. Waiting for data...");

  ros::spin();
  return 0;
}
