#include <Eigen/Core>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class PointCloudFilter {
public:
  PointCloudFilter() : tf_listener_(tf_buffer_) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.getParam("robot_radius", robot_radius_);
    private_nh.param("reference_elevation", reference_elevation_, 0.0);

    sub_ =
        nh.subscribe("pointcloud_sub", 1, &PointCloudFilter::filterCloud, this);
    odom_sub_ = nh.subscribe("/odom", 1, &PointCloudFilter::odomCallback, this);
    pub_      = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud_pub", 1);

    current_elevation_ = 0.0;
    has_odom_          = false;

    ROS_INFO(
        "PointCloudFilter initialized with reference elevation: %.2f",
        reference_elevation_
    );
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    current_elevation_ = odom_msg->pose.pose.position.z;
    has_odom_          = true;
  }

  void filterCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    if (!has_odom_) {
      ROS_WARN_THROTTLE(
          1.0, "No odometry data received yet, skipping point cloud processing"
      );
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    double elevation_offset = reference_elevation_ - current_elevation_;

    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    for (size_t i = 0; i < cloud.size(); i++) {
      pcl::PointXYZ adjusted_point = cloud.points[i];
      adjusted_point.z += elevation_offset;

      float distance = std::sqrt(
          adjusted_point.x * adjusted_point.x +
          adjusted_point.y * adjusted_point.y +
          adjusted_point.z * adjusted_point.z
      );

      if (distance > robot_radius_) {
        filtered_cloud.push_back(adjusted_point);
      }
    }

    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(filtered_cloud, filtered_msg);
    filtered_msg.header = cloud_msg->header;
    pub_.publish(filtered_msg);
  }

private:
  ros::Subscriber sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double robot_radius_;
  double reference_elevation_;
  double current_elevation_;
  bool has_odom_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_cloud_node");
  PointCloudFilter filter;
  ros::spin();
  return 0;
}
