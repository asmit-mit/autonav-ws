#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudFilter {
public:
  PointCloudFilter() {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.getParam("robot_radius", robot_radius_);
    sub_ =
        nh.subscribe("pointcloud_sub", 1, &PointCloudFilter::filterCloud, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("obstacle_cloud_pub", 1);
  }

  void filterCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    for (size_t i = 0; i < cloud.size(); i++) {
      float distance = std::sqrt(
          cloud.points[i].x * cloud.points[i].x +
          cloud.points[i].y * cloud.points[i].y +
          cloud.points[i].z * cloud.points[i].z
      );

      if (distance > robot_radius_) {
        filtered_cloud.push_back(cloud[i]);
      }
    }

    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(filtered_cloud, filtered_msg);
    filtered_msg.header = cloud_msg->header;
    pub_.publish(filtered_msg);
  }

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;
  double robot_radius_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_cloud_node");
  PointCloudFilter filter;
  ros::spin();
  return 0;
}
