#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

ros::Publisher imu_pub;
tf2_ros::TransformBroadcaster *tf_broadcaster;

void imuCallBack(const sensor_msgs::Imu::ConstPtr &msg) {
  geometry_msgs::TransformStamped transform_stamped;

  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = "robot/base_link";
  transform_stamped.child_frame_id = "zed2i_base_link";

  tf2::Quaternion imu_orientation(msg->orientation.x, msg->orientation.y,
                                  msg->orientation.z, msg->orientation.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(imu_orientation).getRPY(roll, pitch, yaw);

  yaw = 0;
  roll = 0;

  tf2::Quaternion corrected_orientation;
  corrected_orientation.setRPY(roll, pitch, yaw);

  transform_stamped.transform.rotation.x = corrected_orientation.x();
  transform_stamped.transform.rotation.y = corrected_orientation.y();
  transform_stamped.transform.rotation.z = corrected_orientation.z();
  transform_stamped.transform.rotation.w = corrected_orientation.w();

  transform_stamped.transform.translation.x = -0.114;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 1.32;

  tf_broadcaster->sendTransform(transform_stamped);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "zed_tf");
  ros::NodeHandle nh;

  tf_broadcaster = new tf2_ros::TransformBroadcaster();

  ros::Subscriber imu_sub = nh.subscribe("/zed_node/imu/data", 10, imuCallBack);

  ros::spin();
  delete tf_broadcaster;
  return 0;
}
