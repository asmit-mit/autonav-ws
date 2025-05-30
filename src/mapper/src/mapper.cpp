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

#include <limits>
#include <unordered_map>
#include <vector>

#include "utils.cpp"

class Mapper {
private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  ros::Publisher map_pub;
  ros::Publisher modify_pub;
  ros::Subscriber cloud_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber mask_sub;
  ros::Subscriber modify_sub;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  float prior;
  float prob_hit;
  float prob_miss;
  float min_prob;
  float max_prob;
  float obstacle_thresh;
  double resolution;
  int width;
  int height;
  int preserve_meters;

  bool got_mask;
  bool got_odom;
  bool got_cloud;
  bool got_transform;

  bool is_map_resizing = false;

  std::vector<double> log_odds_map;

  BotPose current_pose;
  cv::Mat current_mask;
  Map output_map;

  std::unordered_map<int, int> free_count;
  std::unordered_map<int, int> obstacle_count;
  std::unordered_map<int, float> free_accumulated;
  std::unordered_map<int, float> obstacle_accumulated;

  float probToLogOdds(float prob) { return log(prob / (1.0 - prob)); }

  float logOddsToProb(float log_odds) { return 1.0 / (1.0 + exp(-log_odds)); }

  bool checkTransformAvailable() {
    try {
      geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
          "odom", "base_link", ros::Time(0), ros::Duration(0.1)
      );
      got_transform = true;
      return true;
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(5.0, "[mapper] Transform not available: %s", ex.what());
      got_transform = false;
      return false;
    }
  }

  bool allDataReady() { return got_mask && got_odom; }

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
    private_nh.param<int>("preserve_meters", preserve_meters, 10);
  }

  void publishMap(Map &map) {
    nav_msgs::OccupancyGrid grid_msg;

    grid_msg.header.stamp    = ros::Time::now();
    grid_msg.header.frame_id = "odom";

    grid_msg.info.resolution = map.resolution;
    grid_msg.info.width      = map.width;
    grid_msg.info.height     = map.height;

    grid_msg.info.origin.position.x    = map.origin.x;
    grid_msg.info.origin.position.y    = map.origin.y;
    grid_msg.info.origin.position.z    = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;

    grid_msg.data = map.grid;

    map_pub.publish(grid_msg);
  }

  void relocate(Map &map) {
    int preserve_cells = static_cast<int>(preserve_meters / map.resolution);

    std::vector<int8_t> old_grid     = map.grid;
    std::vector<double> old_log_odds = log_odds_map;
    Map old_map                      = map;

    MapPose robot_old_pos =
        Utils::getMapPoseFromWorldPose(current_pose.world_pose, old_map);

    log_odds_map.clear();
    log_odds_map.resize(map.height * map.width, probToLogOdds(prior));

    map.origin.x =
        current_pose.world_pose.x - (map.width * map.resolution / 2.0);
    map.origin.y =
        current_pose.world_pose.y - (map.height * map.resolution / 2.0);
    map.grid.clear();
    map.grid.resize(map.height * map.width, -1);

    MapPose robot_new_pos =
        Utils::getMapPoseFromWorldPose(current_pose.world_pose, map);

    for (int dy = -preserve_cells / 2; dy <= preserve_cells / 2; dy++) {
      for (int dx = -preserve_cells / 2; dx <= preserve_cells / 2; dx++) {
        int old_x = robot_old_pos.x + dx;
        int old_y = robot_old_pos.y + dy;

        int new_x = robot_new_pos.x + dx;
        int new_y = robot_new_pos.y + dy;

        if (old_map.isValid(old_x, old_y) && map.isValid(new_x, new_y)) {
          int old_index = old_map.getIndex(old_x, old_y);
          int new_index = map.getIndex(new_x, new_y);

          map.grid[new_index]     = old_grid[old_index];
          log_odds_map[new_index] = old_log_odds[old_index];
        }
      }
    }

    ROS_INFO(
        "[mapper] Relocated map to %f %f, preserved %dx%d area around robot",
        map.origin.x, map.origin.y, preserve_meters, preserve_meters
    );

    publishMap(map);
    is_map_resizing = false;
  }

  void updateMap(Map &map) {
    for (const auto &pair : free_count) {
      int index = pair.first;
      int count = pair.second;

      float average_update = free_accumulated[index] / count;
      log_odds_map[index] += average_update;

      float prob          = logOddsToProb(log_odds_map[index]);
      prob                = std::max(min_prob, std::min(max_prob, prob));
      log_odds_map[index] = probToLogOdds(prob);

      map.grid[index] = (prob >= obstacle_thresh) ? 100 : 0;
    }

    for (const auto &pair : obstacle_count) {
      int index            = pair.first;
      int count            = pair.second;
      float average_update = obstacle_accumulated[index] / count;
      log_odds_map[index] += average_update;

      float prob          = logOddsToProb(log_odds_map[index]);
      prob                = std::max(min_prob, std::min(max_prob, prob));
      log_odds_map[index] = probToLogOdds(prob);

      map.grid[index] = (prob >= obstacle_thresh) ? 100 : 0;
    }

    free_count.clear();
    obstacle_count.clear();
    free_accumulated.clear();
    obstacle_accumulated.clear();
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (is_map_resizing)
      return;

    if (!checkTransformAvailable()) {
      ROS_WARN_THROTTLE(
          2.0, "[mapper] Transform odom->base_link not available, skipping "
               "point cloud processing"
      );
      return;
    }

    if (!allDataReady()) {
      ROS_WARN_THROTTLE(2.0, "[mapper] Data missing");
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>
    );
    pcl::fromROSMsg(*msg, *current_cloud);

    int cloud_width  = msg->width;
    int cloud_height = msg->height;

    if (cloud_height != current_mask.rows || cloud_width != current_mask.cols) {
      if (cloud_height * cloud_width == current_mask.cols * current_mask.rows) {
        current_mask = current_mask.reshape(1, cloud_height * cloud_width);
      } else {
        ROS_ERROR_THROTTLE(
            1.0,
            "[mapper] Mask and point cloud dimensions don't match! Mask total: "
            "%zu, Cloud total: %d",
            current_mask.total(), cloud_width * cloud_height
        );
        return;
      }
    }

    bool needs_resize = false;

    int count = 0;
    for (int i = 0; i < cloud_width * cloud_height; ++i) {
      const auto &point = current_cloud->points[i];

      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }

      WorldPose wp = WorldPose(point.x, point.y);
      MapPose mp   = Utils::getMapPoseFromWorldPose(wp, output_map);

      if (!output_map.isValid(mp.x, mp.y)) {
        needs_resize = true;
        break;
      }

      int map_index = output_map.getIndex(mp.x, mp.y);

      auto mask_value = current_mask.at<uchar>(i);

      if (mask_value > 127) {
        count++;
        obstacle_count[map_index]++;
        obstacle_accumulated[map_index] += probToLogOdds(prob_hit);
      } else {
        count++;
        free_count[map_index]++;
        free_accumulated[map_index] += probToLogOdds(prob_miss);
      }
    }

    ROS_INFO("[mapper] Processed %d points", count);

    if (needs_resize) {
      is_map_resizing = true;
      relocate(output_map);
    }

    updateMap(output_map);
    publishMap(output_map);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    current_pose.world_pose.x = msg->pose.pose.position.x;
    current_pose.world_pose.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3(q).getRPY(
        current_pose.roll, current_pose.pitch, current_pose.yaw
    );

    got_odom = true;
  }

  void maskCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    current_mask = cv_ptr->image;
    got_mask     = true;
  }

  void relocateCallback(const std_msgs::String &msg) {
    if (msg.data == "relocate") {
      is_map_resizing = true;
      relocate(output_map);
    }
  }

public:
  Mapper() : tf_listener(tf_buffer) {
    loadParameters();

    got_mask      = false;
    got_odom      = false;
    got_cloud     = false;
    got_transform = false;

    ROS_INFO("[mapper] Started map with %d x %d", width, height);

    cloud_sub = nh.subscribe(
        "pointcloud_topic_sub", 1, &Mapper::pointCloudCallback, this
    );
    odom_sub = nh.subscribe("odom_topic_sub", 1, &Mapper::odomCallback, this);
    mask_sub = nh.subscribe("mask_topic_sub", 1, &Mapper::maskCallback, this);

    modify_sub =
        nh.subscribe("map_modify_sub", 1, &Mapper::relocateCallback, this);

    modify_pub = nh.advertise<std_msgs::String>("map_modify_pub", 1);
    map_pub    = nh.advertise<nav_msgs::OccupancyGrid>("map_pub", 1, true);

    output_map.width      = width;
    output_map.height     = height;
    output_map.resolution = resolution;
    output_map.origin.x   = 0;
    output_map.origin.y   = 0;
    output_map.grid.resize(width * height, -1);

    log_odds_map.resize(width * height, probToLogOdds(prior));

    ROS_INFO("[mapper] Waiting for transform between odom and base_link...");
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ROS_INFO("[mapper] PointCloud Mapper Node Started.");
  Mapper mapper;
  ros::spin();
  return 0;
}
