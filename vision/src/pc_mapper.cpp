#include <cmath>
#include <cstdint>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unordered_map>
#include <visualization_msgs/msg/marker.hpp>

class PCMapper : public rclcpp::Node {
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

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

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

  void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mask_mutex_);
    try {
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      current_mask_ = cv_ptr->image;
      mask_received_ = true;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!mask_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "No mask received yet. Skipping point cloud processing.");
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
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
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

        if (point.x < 4 && point.z < 0.1) {
          if (mask_value > 127) {
            obstacle_cloud_->push_back(pcl_point);
          } else {
            if (i % 4 == 0) {
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

    sensor_msgs::msg::PointCloud2 debug_cloud;
    pcl::toROSMsg(*obstacle_cloud_, debug_cloud);
    debug_cloud.header.frame_id = "odom";
    debug_cloud.header.stamp = this->get_clock()->now();
    debug_cloud_pub_->publish(debug_cloud);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_bot_pose_.global_pose.x = msg->pose.pose.position.x;
    current_bot_pose_.global_pose.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(current_bot_pose_.roll, current_bot_pose_.pitch,
                             current_bot_pose_.yaw);
  }

  void updateMap(nav_msgs::msg::OccupancyGrid &map) {
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
      map.data[index] = (prob >= obstacle_thresh_) ? 100 : 0;
    }

    for (const auto &[index, count] : obstacle_count) {
      float average_update = obstacle_accumulated[index] / count;
      logOddsMap_[index] += average_update;

      float prob = logOddsToProb(logOddsMap_[index]);
      prob = std::max(min_prob_, std::min(max_prob_, prob));
      logOddsMap_[index] = probToLogOdds(prob);
      map.data[index] = (prob >= obstacle_thresh_) ? 100 : 0;
    }
  }

  void loadParameters() {
    this->declare_parameter<float>("prior", 0.5);
    this->declare_parameter<float>("prob_hit", 0.6);
    this->declare_parameter<float>("prob_miss", 0.3);
    this->declare_parameter<float>("min_prob", 0.12);
    this->declare_parameter<float>("max_prob", 0.97);
    this->declare_parameter<float>("obstacle_threshold", 0.6);
    this->declare_parameter<float>("octree_resolution", 0.01);
    this->declare_parameter<double>("resolution", 0.01);
    this->declare_parameter<int>("width", 3500);
    this->declare_parameter<int>("height", 3500);

    this->get_parameter("prior", prior_);
    this->get_parameter("prob_hit", prob_hit_);
    this->get_parameter("prob_miss", prob_miss_);
    this->get_parameter("min_prob", min_prob_);
    this->get_parameter("max_prob", max_prob_);
    this->get_parameter("obstacle_threshold", obstacle_thresh_);
    this->get_parameter("octree_resolution", octree_resolution_);
    this->get_parameter("resolution", resolution_);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
  }

public:
  PCMapper()
      : Node("pc_mapper_node"),
        obstacle_octree_(0.01), free_octree_(0.01),
        obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
        free_cloud_(new pcl::PointCloud<pcl::PointXYZ>), mask_received_(false) {

    loadParameters();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic_sub", rclcpp::SensorDataQoS(),
        std::bind(&PCMapper::pointCloudCallback, this, std::placeholders::_1));
    odom_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_topic_sub", rclcpp::QoS(10),
            std::bind(&PCMapper::odomCallback, this, std::placeholders::_1));
    mask_sub_ =
        this->create_subscription<sensor_msgs::msg::Image>(
            "mask_topic_sub", rclcpp::SensorDataQoS(),
            std::bind(&PCMapper::maskCallback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_pub", 1);
    debug_cloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("debug_cloud_pub", 1);

    grid_ = {resolution_, -(width_ / 2.0) * resolution_,
             -(height_ / 2.0) * resolution_, width_, height_};

    logOddsMap_.resize(grid_.width * grid_.height, probToLogOdds(prior_));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&PCMapper::timerCallback, this));
  }

  void timerCallback() {
    nav_msgs::msg::OccupancyGrid map;

    map.info.resolution = grid_.resolution;
    map.info.width = grid_.width;
    map.info.height = grid_.height;
    map.info.origin.position.x = grid_.origin_x;
    map.info.origin.position.y = grid_.origin_y;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(grid_.width * grid_.height, -1);

    map.header.stamp = this->get_clock()->now();
    map.header.frame_id = "odom";

    updateMap(map);
    map_pub_->publish(map);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("pc_mapper_node"), "PointCloud Mapper Node Started.");

  auto node = std::make_shared<PCMapper>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
