#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class PointCloudTransformer {
public:
  PointCloudTransformer() {
    ros::NodeHandle nh;

    pc_sub = nh.subscribe("/zed_node/point_cloud/cloud_registered", 1,
                          &PointCloudTransformer::pointcloudCallback, this);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("nav/point_cloud", 1);

    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    last_pointcloud_time = ros::Time::now();
    received_pointcloud = false;

    monitor_timer = nh.createTimer(
        ros::Duration(0.5), &PointCloudTransformer::monitorCallback, this);

    ROS_INFO("PointCloud Transformer initialized. Waiting for data...");
  }

  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    received_pointcloud = true;
    last_pointcloud_time = ros::Time::now();

    try {
      geometry_msgs::TransformStamped transform_stamped;
      transform_stamped =
          tf_buffer->lookupTransform("robot/odom", cloud_msg->header.frame_id,
                                     ros::Time(0), ros::Duration(1.0));

      sensor_msgs::PointCloud2 transformed_cloud;
      tf2::doTransform(*cloud_msg, transformed_cloud, transform_stamped);
      pc_pub.publish(transformed_cloud);

      last_successful_transform_time = ros::Time::now();
      transform_available = true;

    } catch (tf2::TransformException &ex) {
      ROS_WARN("Could not transform pointcloud: %s", ex.what());
      transform_available = false;
    }
  }

private:
  void monitorCallback(const ros::TimerEvent &) {
    ros::Time current_time = ros::Time::now();

    if (!received_pointcloud) {
      ROS_WARN_THROTTLE(5.0, "No PointCloud data has been received yet from "
                             "/zed_node/point_cloud/cloud_registered");
    } else {
      double time_since_last_pc = (current_time - last_pointcloud_time).toSec();
      if (time_since_last_pc > 1.0) {
        ROS_WARN_THROTTLE(5.0, "No PointCloud data received for %.1f seconds",
                          time_since_last_pc);
      }
    }

    if (!transform_available) {
      ROS_WARN_THROTTLE(
          5.0, "Transform from %s to robot/base_link not available",
          received_pointcloud ? "point cloud frame" : "unknown frame");
    } else {
      double time_since_last_transform =
          (current_time - last_successful_transform_time).toSec();
      if (time_since_last_transform > 1.0) {
        ROS_WARN_THROTTLE(5.0, "No successful transform for %.1f seconds",
                          time_since_last_transform);
      }
    }
  }

  ros::Subscriber pc_sub;
  ros::Publisher pc_pub;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  ros::Timer monitor_timer;
  ros::Time last_pointcloud_time;
  ros::Time last_successful_transform_time;
  bool received_pointcloud{false};
  bool transform_available{false};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_pc");
  PointCloudTransformer transformer;
  ros::spin();
  return 0;
}
