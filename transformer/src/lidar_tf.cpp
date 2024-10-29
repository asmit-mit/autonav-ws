#include "ros/publisher.h"
#include "ros/time.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub_pointcloud;
ros::Publisher pub_laserscan;
ros::Publisher pub_imu;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  sensor_msgs::PointCloud2 new_msg = *msg;
  new_msg.header.stamp = ros::Time::now();

  pub_pointcloud.publish(new_msg);
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  sensor_msgs::LaserScan new_msg = *msg;
  new_msg.header.stamp = ros::Time::now();

  pub_laserscan.publish(new_msg);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  sensor_msgs::Imu new_msg = *msg;
  new_msg.header.stamp = ros::Time::now();

  pub_imu.publish(new_msg);
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

  ros::spin();
  return 0;
}
