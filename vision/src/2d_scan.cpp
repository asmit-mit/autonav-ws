#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudToScan {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher scan_pub_;

  double upper_height_;
  double lower_height_;

public:
  PointCloudToScan() : private_nh_("~") {

    private_nh_.param("upper_height", upper_height_, 2.0);
    private_nh_.param("lower_height", lower_height_, -2.0);

    cloud_sub_ =
        nh_.subscribe("input_cloud", 1, &PointCloudToScan::cloudCallback, this);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("output_scan", 1);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    sensor_msgs::LaserScan scan;
    scan.header = cloud_msg->header;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 180.0;
    scan.time_increment = 0.0;
    scan.scan_time = 0.1;
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    int num_readings =
        ceil((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.resize(num_readings);
    std::fill(scan.ranges.begin(), scan.ranges.end(),
              std::numeric_limits<float>::infinity());

    for (const auto &point : cloud->points) {

      if (point.z >= lower_height_ && point.z <= upper_height_) {

        double angle = atan2(point.y, point.x);
        double range = sqrt(point.x * point.x + point.y * point.y);

        int index = (angle - scan.angle_min) / scan.angle_increment;
        if (index >= 0 && index < num_readings) {

          if (range < scan.ranges[index] || std::isinf(scan.ranges[index])) {
            scan.ranges[index] = range;
          }
        }
      }
    }

    scan_pub_.publish(scan);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_pub_node");
  PointCloudToScan node;
  ros::spin();
  return 0;
}
