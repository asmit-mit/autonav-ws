#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
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
#include <visualization_msgs/Marker.h>

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

class PCMapper {
private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  ros::Publisher map_pub;
  ros::Publisher marker_pub_;
  ros::Publisher modify_pub;
  ros::Publisher debug_cloud_pub;
  ros::Subscriber cloud_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber mask_sub;
  ros::Subscriber modify_sub;

  float prior;
  float prob_hit;
  float prob_miss;
  float min_prob;
  float max_prob;
  float obstacle_thresh;
  double resolution;
  int width;
  int height;

  BotPosition current_pose;
  OccupancyGrid grid;
  std::vector<float> log_odds_map;
  cv::Mat current_mask;
  bool got_mask;
  std::mutex mask_mutex;

  bool map_resizing = false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  float probToLogOdds(float prob) { return log(prob / (1.0 - prob)); }

  float logOddsToProb(float log_odds) {
    return 1.0 - (1.0 / (1.0 + exp(log_odds)));
  }

  GridCell globalPointToPixelIndex(const GlobalPoint &global_point,
                                   const OccupancyGrid &grid) {
    int x = static_cast<int>(
        round((global_point.x - grid.origin_x) / grid.resolution));
    int y = static_cast<int>(
        round((global_point.y - grid.origin_y) / grid.resolution));
    return {x, y};
  }

  void maskCallback(const sensor_msgs::ImageConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mask_mutex);
    try {
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      current_mask = cv_ptr->image;
      got_mask = true;
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void resizeMap() {
    ROS_INFO("Creating new map centered at robot position");

    grid.origin_x = current_pose.global_pose.x - (width * resolution / 2.0);
    grid.origin_y = current_pose.global_pose.y - (height * resolution / 2.0);

    ROS_INFO("Map initialized with origin as x %f and y %f", grid.origin_x,
             grid.origin_y);

    log_odds_map.clear();
    log_odds_map.resize(grid.width * grid.height, probToLogOdds(prior));

    nav_msgs::OccupancyGrid new_map;
    new_map.info.resolution = grid.resolution;
    new_map.info.width = grid.width;
    new_map.info.height = grid.height;
    new_map.info.origin.position.x = grid.origin_x;
    new_map.info.origin.position.y = grid.origin_y;
    new_map.info.origin.position.z = 0.0;
    new_map.info.origin.orientation.w = 1.0;

    new_map.data.clear();
    new_map.data.resize(grid.width * grid.height, -1);

    new_map.header.stamp = ros::Time::now();
    new_map.header.frame_id = "odom";

    updateMap(new_map);
    map_pub.publish(new_map);
    ROS_INFO("New map created at robot position (%.2f, %.2f)",
             current_pose.global_pose.x, current_pose.global_pose.y);
  }

  bool isPointOutOfBounds(const GlobalPoint &point) {
    GridCell cell = globalPointToPixelIndex(point, grid);
    return (cell.x < 0 || cell.x >= grid.width || cell.y < 0 ||
            cell.y >= grid.height);
  }

  void resizeMapCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "resize") {
      map_resizing = true;

      resizeMap();
    }
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (map_resizing) {
      ROS_WARN_THROTTLE(1.0, "Map resizing, skipping cloud data");
      return;
    }

    if (!got_mask) {
      ROS_WARN_THROTTLE(
          1.0, "No mask received yet. Skipping point cloud processing.");
      return;
    }

    std::lock_guard<std::mutex> lock(mask_mutex);

    obstacle_cloud->clear();
    free_cloud->clear();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *current_cloud);

    int cloud_width = msg->width;
    int cloud_height = msg->height;

    if (cloud_height != current_mask.rows || cloud_width != current_mask.cols) {
      ROS_ERROR_THROTTLE(1.0,
                         "Mask and point cloud dimensions don't match! Mask: "
                         "%dx%d, Cloud: %dx%d",
                         current_mask.rows, current_mask.cols, cloud_height,
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

        GlobalPoint global_point;
        global_point.x = point.x;
        global_point.y = point.y;

        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = 0.0;

        uchar mask_value = current_mask.at<uchar>(row, col);

        if (point.z < 0.1) {

          if (isPointOutOfBounds(global_point)) {
            needs_resize = true;
            break;
          }

          if (mask_value > 127) {
            obstacle_cloud->push_back(pcl_point);
          } else {
            if (i % 5 == 0) {
              free_cloud->push_back(pcl_point);
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
      modify_pub.publish(resize_msg);
      return;
    }

    /* sensor_msgs::PointCloud2 debug_cloud; */
    /* pcl::toROSMsg(*obstacle_cloud, debug_cloud); */
    /* debug_cloud.header.frame_id = "robot/odom"; */
    /* debug_cloud.header.stamp = ros::Time::now(); */
    /* debug_cloud_pub.publish(debug_cloud); */
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    current_pose.global_pose.x = msg->pose.pose.position.x;
    current_pose.global_pose.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(current_pose.roll, current_pose.pitch,
                             current_pose.yaw);
  }

  void updateMap(nav_msgs::OccupancyGrid &map) {
    if (map_resizing)
      return;

    std::unordered_map<int, int> free_count;
    std::unordered_map<int, int> obstacle_count;
    std::unordered_map<int, float> free_accumulated;
    std::unordered_map<int, float> obstacle_accumulated;

    for (const auto &point : free_cloud->points) {
      GlobalPoint global_point{point.x, point.y};
      GridCell cell = globalPointToPixelIndex(global_point, grid);

      if (cell.x >= 0 && cell.x < grid.width && cell.y >= 0 &&
          cell.y < grid.height) {
        int index = cell.y * grid.width + cell.x;
        free_count[index]++;
        free_accumulated[index] += probToLogOdds(prob_miss);
      }
    }

    for (const auto &point : obstacle_cloud->points) {
      GlobalPoint global_point{point.x, point.y};
      GridCell cell = globalPointToPixelIndex(global_point, grid);

      if (cell.x >= 0 && cell.x < grid.width && cell.y >= 0 &&
          cell.y < grid.height) {
        int index = cell.y * grid.width + cell.x;
        obstacle_count[index]++;
        obstacle_accumulated[index] += probToLogOdds(prob_hit);
      }
    }

    for (const auto &pair : free_count) {
      int index = pair.first;
      int count = pair.second;

      float average_update = free_accumulated[index] / count;
      log_odds_map[index] += average_update;

      float prob = logOddsToProb(log_odds_map[index]);
      prob = std::max(min_prob, std::min(max_prob, prob));
      log_odds_map[index] = probToLogOdds(prob);

      map.data[index] = (prob >= obstacle_thresh) ? 100 : 0;
    }

    for (const auto &pair : obstacle_count) {
      int index = pair.first;
      int count = pair.second;
      float average_update = obstacle_accumulated[index] / count;
      log_odds_map[index] += average_update;

      float prob = logOddsToProb(log_odds_map[index]);
      prob = std::max(min_prob, std::min(max_prob, prob));
      log_odds_map[index] = probToLogOdds(prob);

      map.data[index] = (prob >= obstacle_thresh) ? 100 : 0;
    }
  }

  void loadParameters() {
    private_nh.param<float>("prior", prior, 0.5);
    private_nh.param<float>("prob_hit", prob_hit, 0.6);
    private_nh.param<float>("prob_miss", prob_miss, 0.3);
    private_nh.param<float>("min_prob", min_prob, 0.12);
    private_nh.param<float>("max_prob", max_prob, 0.97);
    private_nh.param<float>("obstacle_threshold", obstacle_thresh, 0.6);
    private_nh.param<double>("resolution", resolution, 0.01);
    private_nh.param<int>("width", width, 3500);
    private_nh.param<int>("height", height, 3500);
  }

public:
  PCMapper()
      : obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        free_cloud(new pcl::PointCloud<pcl::PointXYZ>), got_mask(false),
        tf_listener(tf_buffer) {

    loadParameters();

    cloud_sub = nh.subscribe("pointcloud_topic_sub", 1,
                             &PCMapper::pointCloudCallback, this);
    odom_sub = nh.subscribe("odom_topic_sub", 1, &PCMapper::odomCallback, this);
    mask_sub = nh.subscribe("mask_topic_sub", 1, &PCMapper::maskCallback, this);

    modify_sub =
        nh.subscribe("map_modify_sub", 1, &PCMapper::resizeMapCallback, this);

    modify_pub = nh.advertise<std_msgs::String>("map_modify_pub", 1);

    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_pub", 1, true);
    debug_cloud_pub =
        nh.advertise<sensor_msgs::PointCloud2>("debug_cloud_pub", 1, true);

    grid = {resolution, -(width / 2.0) * resolution,
            -(height / 2.0) * resolution, width, height};

    log_odds_map.resize(grid.width * grid.height, probToLogOdds(prior));
  }

  void run() {
    ros::Rate loop_rate(20);
    nav_msgs::OccupancyGrid map;

    map.info.resolution = grid.resolution;
    map.info.width = grid.width;
    map.info.height = grid.height;
    map.info.origin.position.x = grid.origin_x;
    map.info.origin.position.y = grid.origin_y;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(grid.width * grid.height, -1);

    map.header.stamp = ros::Time::now();
    map.header.frame_id = "robot/odom";

    while (ros::ok()) {
      try {
        if (tf_buffer.canTransform("robot/odom", "robot/base_link",
                                   ros::Time(0), ros::Duration(1.0))) {
          if (map_resizing) {
            map.info.origin.position.x = grid.origin_x;
            map.info.origin.position.y = grid.origin_y;
            map.data.clear();
            map.data.resize(grid.width * grid.height, -1);

            map_resizing = false;
          }

          updateMap(map);
          map_pub.publish(map);
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
