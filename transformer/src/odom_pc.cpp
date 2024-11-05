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
  }

  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    try {
      geometry_msgs::TransformStamped transform_stamped;
      transform_stamped = tf_buffer->lookupTransform(
          "robot/base_link", cloud_msg->header.frame_id, ros::Time(0),
          ros::Duration(1.0));

      sensor_msgs::PointCloud2 transformed_cloud;
      tf2::doTransform(*cloud_msg, transformed_cloud, transform_stamped);

      pc_pub.publish(transformed_cloud);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Could not transform pointcloud: %s", ex.what());
    }
  }

private:
  ros::Subscriber pc_sub;
  ros::Publisher pc_pub;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_pc");
  PointCloudTransformer transformer;
  ros::spin();
  return 0;
}
