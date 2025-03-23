#include <chrono>
#include <cmath>
#include <cstdint>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <limits>
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
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
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
  ros::NodeHandle private_nh_{"~"};
  ros::Publisher map_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher pc_pub_;
  ros::Publisher modify_pub_;
  ros::Publisher goal_sleep_pub_;
  ros::Publisher debug_cloud_pub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber mask_sub_;
  ros::Subscriber modify_sub_;

  float prior_;
  float prob_hit_;
  float prob_miss_;
  float min_prob_;
  float max_prob_;
  float obstacle_thresh_;
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

  bool map_resizing = false;

  pcl::octree::OctreePointCloud<pcl::PointXYZ> obstacle_octree_;
  pcl::octree::OctreePointCloud<pcl::PointXYZ> free_octree_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud_;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

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

  void resizeMap() {
    ROS_INFO("Creating new map centered at robot position");

    grid_.origin_x =
        current_bot_pose_.global_pose.x - (width_ * resolution_ / 2.0);
    grid_.origin_y =
        current_bot_pose_.global_pose.y - (height_ * resolution_ / 2.0);

    ROS_INFO("Map initialized with origin as x %f and y %f", grid_.origin_x,
             grid_.origin_y);

    logOddsMap_.clear();
    logOddsMap_.resize(grid_.width * grid_.height, probToLogOdds(prior_));

    nav_msgs::OccupancyGrid new_map;
    new_map.info.resolution = grid_.resolution;
    new_map.info.width = grid_.width;
    new_map.info.height = grid_.height;
    new_map.info.origin.position.x = grid_.origin_x;
    new_map.info.origin.position.y = grid_.origin_y;
    new_map.info.origin.position.z = 0.0;
    new_map.info.origin.orientation.w = 1.0;

    new_map.data.clear();
    new_map.data.resize(grid_.width * grid_.height, -1);

    new_map.header.stamp = ros::Time::now();
    new_map.header.frame_id = "odom";

    updateMap(new_map);
    map_pub_.publish(new_map);
    ROS_INFO("New map created at robot position (%.2f, %.2f)",
             current_bot_pose_.global_pose.x, current_bot_pose_.global_pose.y);
  }

  bool isPointOutOfBounds(const GlobalPoint &point) {
    GridCell cell = globalPointToPixelIndex(point, grid_);
    return (cell.x < 0 || cell.x >= grid_.width || cell.y < 0 ||
            cell.y >= grid_.height);
  }

  void resizeMapCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "resize") {
      map_resizing = true;

      std_msgs::String sleep_msg;
      sleep_msg.data = "sleep";
      goal_sleep_pub_.publish(sleep_msg);

      resizeMap();
    }
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (map_resizing) {
      ROS_WARN_THROTTLE(1.0, "Map resizing, skipping cloud data");
      return;
    }

    if (!mask_received_) {
      ROS_WARN_THROTTLE(
          1.0, "No mask received yet. Skipping point cloud processing.");
      return;
    }

    std::lock_guard<std::mutex> lock(mask_mutex_);

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

    bool needs_resize = false;
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

        if (point.x < 4 && point.z < 0.1) {

          if (isPointOutOfBounds(global_point)) {
            needs_resize = true;
            break;
          }

          if (mask_value > 127) {
            obstacle_cloud_->push_back(pcl_point);
          } else {
            if (i % 10 == 0) {
              free_cloud_->push_back(pcl_point);
            }
          }
        }
      }
      if (needs_resize)
        break;
    }

    if (needs_resize) {
      std_msgs::String resize_msg;
      resize_msg.data = "resize";
      modify_pub_.publish(resize_msg);
      return;
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
    if (map_resizing)
      return;

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
      /* logOddsMap_[index] = */
      /*     std::max(static_cast<float>(-10), */
      /*              std::min(logOddsMap_[index], static_cast<float>(10))); */

      float prob = logOddsToProb(logOddsMap_[index]);
      prob = std::max(min_prob_, std::min(max_prob_, prob));
      logOddsMap_[index] = probToLogOdds(prob);

      map.data[index] = (prob >= obstacle_thresh_) ? 100 : 0;
    }

    for (const auto &[index, count] : obstacle_count) {
      float average_update = obstacle_accumulated[index] / count;
      logOddsMap_[index] += average_update;
      /* logOddsMap_[index] = */
      /*     std::max(static_cast<float>(-10), */
      /*              std::min(logOddsMap_[index], static_cast<float>(10))); */

      float prob = logOddsToProb(logOddsMap_[index]);
      prob = std::max(min_prob_, std::min(max_prob_, prob));
      logOddsMap_[index] = probToLogOdds(prob);

      map.data[index] = (prob >= obstacle_thresh_) ? 100 : 0;
    }
  }

  void loadParameters() {
    private_nh_.param<float>("prior", prior_, 0.5);
    private_nh_.param<float>("prob_hit", prob_hit_, 0.6);
    private_nh_.param<float>("prob_miss", prob_miss_, 0.3);
    private_nh_.param<float>("min_prob", min_prob_, 0.12);
    private_nh_.param<float>("max_prob", max_prob_, 0.97);
    private_nh_.param<float>("obstacle_threshold", obstacle_thresh_, 0.6);
    private_nh_.param<float>("octree_resolution", octree_resolution_, 0.01);
    private_nh_.param<double>("resolution", resolution_, 0.01);
    private_nh_.param<int>("width", width_, 3500);
    private_nh_.param<int>("height", height_, 3500);
  }

public:
  PCMapper()
      : obstacle_octree_(0.01), free_octree_(0.01),
        obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
        free_cloud_(new pcl::PointCloud<pcl::PointXYZ>), mask_received_(false),
        tf_listener(tf_buffer) {

    loadParameters();

    cloud_sub_ = nh_.subscribe("pointcloud_topic_sub", 1,
                               &PCMapper::pointCloudCallback, this);
    odom_sub_ =
        nh_.subscribe("odom_topic_sub", 1, &PCMapper::odomCallback, this);
    mask_sub_ =
        nh_.subscribe("mask_topic_sub", 1, &PCMapper::maskCallback, this);

    modify_sub_ =
        nh_.subscribe("map_modify_sub", 1, &PCMapper::resizeMapCallback, this);

    modify_pub_ = nh_.advertise<std_msgs::String>("map_modify_pub", 1);

    goal_sleep_pub_ =
        nh_.advertise<std_msgs::String>("goal_sleep_pub", 1, true);

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_pub", 1, true);
    debug_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("debug_cloud_pub", 1, true);

    grid_ = {resolution_, -(width_ / 2.0) * resolution_,
             -(height_ / 2.0) * resolution_, width_, height_};

    logOddsMap_.resize(grid_.width * grid_.height, probToLogOdds(prior_));
  }

  void run() {
    ros::Rate loop_rate(20);
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
      try {
        if (tf_buffer.canTransform("robot/odom", "robot/base_link",
                                   ros::Time(0), ros::Duration(1.0))) {

          map.header.stamp = ros::Time::now();
          map.header.frame_id = "robot/odom";

          map.info.width = grid_.width;
          map.info.height = grid_.height;
          map.info.origin.position.x = grid_.origin_x;
          map.info.origin.position.y = grid_.origin_y;

          if (map_resizing) {
            obstacle_cloud_->clear();
            free_cloud_->clear();
            obstacle_octree_.deleteTree();
            free_octree_.deleteTree();
            map.data.clear();
            map_resizing = false;
          }

          map.data.resize(grid_.width * grid_.height, -1);

          updateMap(map);
          map_pub_.publish(map);

          obstacle_cloud_->clear();
          free_cloud_->clear();
          obstacle_octree_.deleteTree();
          free_octree_.deleteTree();

        } else {
          ROS_WARN_THROTTLE(1.0, "Transform between 'robot/odom' and "
                                 "'robot/base_link' is not available yet.");
        }
      } catch (const tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "Transform lookup failed: %s", ex.what());
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pc_mapper_node");

  ROS_INFO("PointCloud Mapper Node Started In sim.");

  PCMapper mapper;
  mapper.run();

  return 0;
}
