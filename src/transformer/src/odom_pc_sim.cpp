#include <mutex>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/PointCloud2.h>

struct BotPosition {
  double x, y, z;
  double yaw;
};

class PointCloudTransformer {
public:
  PointCloudTransformer() {
    point_cloud_sub_ = nh_.subscribe(
        "zed2i/zed_node/point_cloud/cloud_registered", 1,
        &PointCloudTransformer::pointCloudCallback, this
    );
    odom_sub_ =
        nh_.subscribe("/odom", 1, &PointCloudTransformer::odomCallback, this);
    transformed_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/nav/point_cloud", 1);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher transformed_cloud_pub_;

  std::mutex odom_mutex_;
  BotPosition bot_pose;

  pcl::PointXYZRGB cloudPointToGlobalPoint(
      const pcl::PointXYZRGB &cloud_point, const BotPosition &bot_pos
  ) {
    pcl::PointXYZRGB global_point;

    global_point.x =
        (bot_pos.x + cloud_point.x * sin(bot_pos.yaw) +
         cloud_point.z * cos(bot_pos.yaw));
    global_point.y =
        (bot_pos.y + cloud_point.z * sin(bot_pos.yaw) -
         cloud_point.x * cos(bot_pos.yaw));
    global_point.z = -1 * cloud_point.y + 1.5;
    global_point.r = cloud_point.r;
    global_point.g = cloud_point.g;
    global_point.b = cloud_point.b;

    return global_point;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    bot_pose.x = msg->pose.pose.position.x;
    bot_pose.y = msg->pose.pose.position.y;
    bot_pose.z = msg->pose.pose.position.z;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w
    );
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, bot_pose.yaw);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>()
    );
    pcl::fromROSMsg(*msg, *input_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>()
    );

    BotPosition bot_pos;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      bot_pos = bot_pose;
    }

    for (const auto &point : input_cloud->points) {
      transformed_cloud->points.push_back(
          cloudPointToGlobalPoint(point, bot_pos)
      );
    }

    transformed_cloud->header.frame_id = "odom";
    transformed_cloud->width           = transformed_cloud->points.size();
    transformed_cloud->height          = 1;
    transformed_cloud->is_dense        = true;

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_cloud, output_msg);
    output_msg.header.stamp = ros::Time::now();
    transformed_cloud_pub_.publish(output_msg);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_pc_sim");
  PointCloudTransformer transformer;
  ros::spin();
  return 0;
}
