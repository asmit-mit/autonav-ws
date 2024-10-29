#include <cmath>
#include <cstdint>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>

class PCMapper {
private:
  struct GridCell {
    int x, y;
  };

  struct GlobalPoint {
    double x, y;
  };

  struct BotPosition {
    GlobalPoint global_pose;
    GridCell map_pose;
    double yaw, roll, pitch;
  };

  struct RGB {
    uint8_t r, g, b;
  };

  struct HSV {
    float h, s, v;
  };

  struct OccupancyGrid {
    double resolution;
    double origin_x, origin_y;
    int width, height;
  };

  ros::NodeHandle nh_;
  ros::Publisher map_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher pc_pub_;
  ros::Publisher debug_cloud_pub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber mask_sub_;

  float prior_;
  float prob_hit_;
  float prob_miss_;
  float min_prob_;
  float max_prob_;
  float octree_resolution_;
  double resolution_;
  int width_;
  int height_;

  BotPosition current_bot_pose_;
  OccupancyGrid grid_;
  std::vector<float> logOddsMap_;
  cv::Mat current_mask_;
  bool mask_received_;
  std::mutex mask_mutex_;

  pcl::octree::OctreePointCloud<pcl::PointXYZ> obstacle_octree_;
  pcl::octree::OctreePointCloud<pcl::PointXYZ> free_octree_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud_;

  float probToLogOdds(float prob) { return log(prob / (1.0 - prob)); }

  float logOddsToProb(float logOdds) {
    return 1.0 - (1.0 / (1.0 + exp(logOdds)));
  }

  GridCell globalPointToPixelIndex(const GlobalPoint &global_point,
                                   const OccupancyGrid &grid) {
    int x = static_cast<int>(
        round((global_point.x - grid.origin_x) / grid.resolution));
    int y = static_cast<int>(
        round((global_point.y - grid.origin_y) / grid.resolution));
    return {x, y};
  }

  GlobalPoint cloudPointToGlobal(const pcl::PointXYZRGB &cloud_point,
                                 const BotPosition &bot_pose) {
    double cos_yaw = cos(bot_pose.yaw);
    double sin_yaw = sin(bot_pose.yaw);

    double rotated_x = cloud_point.x * cos_yaw - cloud_point.y * sin_yaw;
    double rotated_y = cloud_point.x * sin_yaw + cloud_point.y * cos_yaw;

    return {rotated_x + bot_pose.global_pose.x,
            rotated_y + bot_pose.global_pose.y};
  }

  void maskCallback(const sensor_msgs::ImageConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mask_mutex_);
    try {
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      current_mask_ = cv_ptr->image;
      mask_received_ = true;
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (!mask_received_) {
      ROS_WARN_THROTTLE(
          1.0, "No mask received yet. Skipping point cloud processing.");
      return;
    }

    std::lock_guard<std::mutex> lock(mask_mutex_);

    obstacle_cloud_->clear();
    free_cloud_->clear();
    obstacle_octree_.deleteTree();
    free_octree_.deleteTree();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *current_cloud);

    int cloud_width = msg->width;
    int cloud_height = msg->height;

    if (cloud_height != current_mask_.rows ||
        cloud_width != current_mask_.cols) {
      ROS_ERROR_THROTTLE(1.0,
                         "Mask and point cloud dimensions don't match! Mask: "
                         "%dx%d, Cloud: %dx%d",
                         current_mask_.rows, current_mask_.cols, cloud_height,
                         cloud_width);
      return;
    }

    int i = 0;

    for (int row = 0; row < cloud_height; ++row) {
      for (int col = 0; col < cloud_width; ++col) {
        i++;
        int index = row * cloud_width + col;
        const auto &point = current_cloud->points[index];

        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
          continue;
        }

        GlobalPoint global_point = cloudPointToGlobal(point, current_bot_pose_);

        pcl::PointXYZ pcl_point;
        pcl_point.x = global_point.x;
        pcl_point.y = global_point.y;
        pcl_point.z = 0.0;

        uchar mask_value = current_mask_.at<uchar>(row, col);

        if (point.x < 4) {
          if (mask_value > 127) {
            obstacle_cloud_->push_back(pcl_point);
          } else {
            if (i % 2 == 0) {
              free_cloud_->push_back(pcl_point);
            }
          }
        }
      }
    }

    obstacle_octree_.setInputCloud(obstacle_cloud_);
    obstacle_octree_.addPointsFromInputCloud();

    free_octree_.setInputCloud(free_cloud_);
    free_octree_.addPointsFromInputCloud();

    sensor_msgs::PointCloud2 debug_cloud;
    pcl::toROSMsg(*obstacle_cloud_, debug_cloud);
    debug_cloud.header.frame_id = "robot/odom";
    debug_cloud.header.stamp = ros::Time::now();
    debug_cloud_pub_.publish(debug_cloud);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    current_bot_pose_.global_pose.x = msg->pose.pose.position.x;
    current_bot_pose_.global_pose.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(current_bot_pose_.roll, current_bot_pose_.pitch,
                             current_bot_pose_.yaw);
  }

  void updateMap(nav_msgs::OccupancyGrid &map) {
    std::unordered_map<int, int> free_count;
    std::unordered_map<int, int> obstacle_count;
    std::unordered_map<int, float> free_accumulated;
    std::unordered_map<int, float> obstacle_accumulated;

    for (const auto &point : free_cloud_->points) {
      GlobalPoint global_point{point.x, point.y};
      GridCell cell = globalPointToPixelIndex(global_point, grid_);

      if (cell.x >= 0 && cell.x < grid_.width && cell.y >= 0 &&
          cell.y < grid_.height) {
        int index = cell.y * grid_.width + cell.x;
        free_count[index]++;
        free_accumulated[index] += probToLogOdds(prob_miss_);
      }
    }

    for (const auto &point : obstacle_cloud_->points) {
      GlobalPoint global_point{point.x, point.y};
      GridCell cell = globalPointToPixelIndex(global_point, grid_);

      if (cell.x >= 0 && cell.x < grid_.width && cell.y >= 0 &&
          cell.y < grid_.height) {
        int index = cell.y * grid_.width + cell.x;
        obstacle_count[index]++;
        obstacle_accumulated[index] += probToLogOdds(prob_hit_);
      }
    }

    for (const auto &[index, count] : free_count) {
      float average_update = free_accumulated[index] / count;
      logOddsMap_[index] += average_update;

      float prob = logOddsToProb(logOddsMap_[index]);
      prob = std::max(min_prob_, std::min(max_prob_, prob));
      logOddsMap_[index] = probToLogOdds(prob);
      map.data[index] = (prob >= max_prob_) ? 100 : 0;
    }

    for (const auto &[index, count] : obstacle_count) {
      float average_update = obstacle_accumulated[index] / count;
      logOddsMap_[index] += average_update;

      float prob = logOddsToProb(logOddsMap_[index]);
      prob = std::max(min_prob_, std::min(max_prob_, prob));
      logOddsMap_[index] = probToLogOdds(prob);
      map.data[index] = (prob >= max_prob_) ? 100 : 0;
    }
  }

  void loadParameters() {
    nh_.param<float>("prior", prior_, 0.5);
    nh_.param<float>("prob_hit", prob_hit_, 0.6);
    nh_.param<float>("prob_miss", prob_miss_, 0.3);
    nh_.param<float>("min_prob", min_prob_, 0.12);
    nh_.param<float>("max_prob", max_prob_, 0.97);
    nh_.param<float>("octree_resolution", octree_resolution_, 0.01);
    nh_.param<double>("resolution", resolution_, 0.01);
    nh_.param<int>("width", width_, 3500);
    nh_.param<int>("height", height_, 3500);
  }

public:
  PCMapper()
      : obstacle_octree_(0.01), free_octree_(0.01),
        obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
        free_cloud_(new pcl::PointCloud<pcl::PointXYZ>), mask_received_(false) {

    loadParameters();

    cloud_sub_ = nh_.subscribe("pointcloud_topic_sub", 1,
                               &PCMapper::pointCloudCallback, this);
    odom_sub_ =
        nh_.subscribe("odom_topic_sub", 1, &PCMapper::odomCallback, this);
    mask_sub_ =
        nh_.subscribe("mask_topic_sub", 1, &PCMapper::maskCallback, this);

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_pub", 1, true);
    debug_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud_pub", 1, true);

    grid_ = {resolution_, -(width_ / 2.0) * resolution_,
             -(height_ / 2.0) * resolution_, width_, height_};

    logOddsMap_.resize(grid_.width * grid_.height, probToLogOdds(prior_));
  }

  void run() {
    ros::Rate loop_rate(10);
    nav_msgs::OccupancyGrid map;

    map.info.resolution = grid_.resolution;
    map.info.width = grid_.width;
    map.info.height = grid_.height;
    map.info.origin.position.x = grid_.origin_x;
    map.info.origin.position.y = grid_.origin_y;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(grid_.width * grid_.height, -1);

    while (ros::ok()) {
      map.header.stamp = ros::Time::now();
      map.header.frame_id = "robot/odom";

      updateMap(map);
      map_pub_.publish(map);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pc_mapper_node");

  ROS_INFO("PointCloud Mapper Node Started.");

  PCMapper mapper;
  mapper.run();

  return 0;
}
