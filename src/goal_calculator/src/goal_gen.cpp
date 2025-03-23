#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <future>
#include <iostream>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_set>
#include <vector>

struct GridCell {
  int x, y;
  GridCell(int x = 0, int y = 0) : x(x), y(y) {}
  bool operator==(const GridCell &other) const {
    return x == other.x && y == other.y;
  }
};

struct GridCellHash {
  size_t operator()(const GridCell &cell) const {
    return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
  }
};

struct Point {
  double x, y;
};

struct Map {
  int width, height;
  GridCell origin;
  double resolution;
  std::vector<int> grid;
};

struct BotPose {
  Point world_position;
  GridCell map_position;
  double yaw;
};

class GoalGenerator {
public:
  GoalGenerator(ros::NodeHandle &nh);
  ~GoalGenerator();

private:
  ros::Publisher marker_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher modify_pub_;

  ros::Subscriber map_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber global_goal_sub_;
  ros::Subscriber modify_sub_;
  ros::Subscriber gps_goal_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<std::vector<bool>> ignored_cells_;
  std::unordered_set<GridCell, GridCellHash> valid_lane_cells_;
  GridCell last_goal_cell_;
  BotPose current_pose_;
  Map current_map_;

  bool paused_ = false;
  int gps_counter_;

  std::atomic<bool> odom_received_;
  std::atomic<bool> map_received_;
  std::mutex map_mutex_;
  std::mutex pose_mutex_;
  std::thread goal_thread_;

  void initializePublishers(ros::NodeHandle &nh);
  void initializeSubscribers(ros::NodeHandle &nh);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void gpsGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void gpsCounterCallback(const std_msgs::Int32::ConstPtr &msg);
  void modifyCallback(const std_msgs::String::ConstPtr &msg);

  void processGoal();
  void updateIgnoredCells(const GridCell &current_pos,
                          const GridCell &goal_cell);
  void processGoalCell(GridCell &goal_cell);
  bool isTransformAvailable();

  void publishMarker(double x, double y, int id);
  void publishGoal(double x, double y, double yaw);
  GridCell findClosestLaneParallel(const std::vector<int> &map, int width,
                                   GridCell start);
  GridCell exploreFarthestParallel(
      const std::unordered_set<GridCell, GridCellHash> &cluster,
      const GridCell &start, double eps);

  static inline int gridToIndex(int x, int y, int width) {
    return y * width + x;
  }

  static inline GridCell indexToGrid(int index, int width) {
    return GridCell(index % width, index / width);
  }

  GridCell pointToGrid(const Point &point, const Map &map) {
    GridCell cell;
    cell.x = static_cast<int>(round((point.x - map.origin.x) / map.resolution));
    cell.y = static_cast<int>(round((point.y - map.origin.y) / map.resolution));
    return cell;
  }

  Point gridToPoint(const GridCell &cell, const Map &map) {
    Point point;
    point.x = cell.x * map.resolution + map.origin.x;
    point.y = cell.y * map.resolution + map.origin.y;
    return point;
  }

  static double distanceGrid(const GridCell &a, const GridCell &b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
  }

  static double distancePoint(const Point &a, const Point &b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
  }
};

GridCell GoalGenerator::findClosestLaneParallel(const std::vector<int> &map,
                                                int width, GridCell start) {
  const int num_threads = 8;
  std::vector<std::future<GridCell>> futures;

  auto searchSection = [&](int startY, int endY) -> GridCell {
    for (int y = startY; y < endY; ++y) {
      for (int x = 0; x < width; ++x) {
        if (map[gridToIndex(x, y, width)] == 1) {
          return GridCell(x, y);
        }
      }
    }
    return GridCell(-1, -1);
  };

  int sectionHeight = map.size() / width / num_threads;
  for (int i = 0; i < num_threads; ++i) {
    int startY = i * sectionHeight;
    int endY =
        (i == num_threads - 1) ? map.size() / width : (i + 1) * sectionHeight;
    futures.push_back(
        std::async(std::launch::async, searchSection, startY, endY));
  }

  GridCell closest(-1, -1);
  double minDist = std::numeric_limits<double>::max();
  for (auto &f : futures) {
    GridCell result = f.get();
    if (result.x != -1) {
      double dist = distanceGrid(start, result);
      if (dist < minDist) {
        minDist = dist;
        closest = result;
      }
    }
  }

  return closest;
}

GridCell GoalGenerator::exploreFarthestParallel(
    const std::unordered_set<GridCell, GridCellHash> &cluster,
    const GridCell &start, double eps) {

  const int num_threads = 4;
  std::vector<std::future<std::pair<GridCell, double>>> futures;

  auto exploreSection =
      [&](const std::vector<GridCell> &cells) -> std::pair<GridCell, double> {
    GridCell farthest = start;
    double maxDist = 0;
    for (const auto &cell : cells) {
      double dist = distanceGrid(start, cell);
      if (dist > maxDist) {
        maxDist = dist;
        farthest = cell;
      }
    }
    return {farthest, maxDist};
  };

  std::vector<GridCell> clusterVec(cluster.begin(), cluster.end());
  int sectionSize = clusterVec.size() / num_threads;

  for (int i = 0; i < num_threads; ++i) {
    int startIdx = i * sectionSize;
    int endIdx =
        (i == num_threads - 1) ? clusterVec.size() : (i + 1) * sectionSize;
    std::vector<GridCell> section(clusterVec.begin() + startIdx,
                                  clusterVec.begin() + endIdx);
    futures.push_back(std::async(std::launch::async, exploreSection, section));
  }

  GridCell farthest = start;
  double maxDist = 0;
  for (auto &f : futures) {
    auto [cell, dist] = f.get();
    if (dist > maxDist) {
      maxDist = dist;
      farthest = cell;
    }
  }

  return farthest;
}

void GoalGenerator::updateIgnoredCells(const GridCell &current_pos,
                                       const GridCell &goal_cell) {
  int size_of_crop = static_cast<int>(distanceGrid(current_pos, goal_cell));

  for (int i = std::max(0, current_pos.x - size_of_crop);
       i <= std::min(current_map_.width - 1, current_pos.x + size_of_crop);
       i++) {
    for (int j = std::max(0, current_pos.y - size_of_crop);
         j <= std::min(current_map_.height - 1, current_pos.y + size_of_crop);
         j++) {
      ignored_cells_[j][i] = true;
    }
  }
}

void GoalGenerator::processGoalCell(GridCell &goal_cell) {
  if (goal_cell == GridCell(0, 0)) {
    goal_cell = last_goal_cell_;
  }

  GridCell goal_cell_e;
  goal_cell_e.x = goal_cell.x + 120 * cos(current_pose_.yaw + 0);
  goal_cell_e.y = goal_cell.y + 120 * sin(current_pose_.yaw + 0);

  // ROS_INFO("Goal given at x %d y %d with map size as %d x %d", goal_cell_e.x,
  //          goal_cell_e.y, current_map_.width, current_map_.height);

  if (goal_cell_e.x < 50 || goal_cell_e.y < 50 ||
      goal_cell_e.x > current_map_.width - 50 ||
      goal_cell_e.y > current_map_.height - 50) {

    std_msgs::String resize_msg;
    resize_msg.data = "resize";
    modify_pub_.publish(resize_msg);
    ROS_INFO("Published resize message");
  }

  goal_cell_e.x =
      std::max(51, std::min(current_map_.width - 51, goal_cell_e.x));
  goal_cell_e.y =
      std::max(51, std::min(current_map_.height - 51, goal_cell_e.y));

  Point goal_point = gridToPoint(goal_cell_e, current_map_);
  publishMarker(goal_point.x, goal_point.y, 1);
  publishGoal(goal_point.x, goal_point.y, current_pose_.yaw);

  last_goal_cell_ = goal_cell;
}

GoalGenerator::GoalGenerator(ros::NodeHandle &nh)
    : tf_listener_(tf_buffer_), last_goal_cell_(-1, -1), odom_received_(false),
      map_received_(false) {
  initializePublishers(nh);
  initializeSubscribers(nh);
  goal_thread_ = std::thread(&GoalGenerator::processGoal, this);
}

GoalGenerator::~GoalGenerator() {
  if (goal_thread_.joinable()) {
    goal_thread_.join();
  }
}

bool GoalGenerator::isTransformAvailable() {
  try {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        "robot/odom", "robot/base_link", ros::Time(0), ros::Duration(1.0));
    return true;
  } catch (tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(5.0, "Transform not available: %s", ex.what());
    return false;
  }
}

void GoalGenerator::initializePublishers(ros::NodeHandle &nh) {
  goal_pub_ =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  marker_pub_ =
      nh.advertise<visualization_msgs::Marker>("/nav/local_goal_marker", 10);
  modify_pub_ = nh.advertise<std_msgs::String>("/nav/needs_modify", 10);
}

void GoalGenerator::initializeSubscribers(ros::NodeHandle &nh) {
  map_sub_ =
      nh.subscribe("nav/global_map", 1, &GoalGenerator::mapCallback, this);
  odom_sub_ = nh.subscribe("robot/dlo/odom_node/odom", 1,
                           &GoalGenerator::odomCallback, this);
  modify_sub_ = nh.subscribe("/nav/needs_modify", 1,
                             &GoalGenerator::modifyCallback, this);
  gps_goal_sub_ =
      nh.subscribe("/nav/gps_goal", 1, &GoalGenerator::gpsGoalCallback, this);
}

void GoalGenerator::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(map_mutex_);

  current_map_.height = msg->info.height;
  current_map_.width = msg->info.width;
  current_map_.origin.x = msg->info.origin.position.x;
  current_map_.origin.y = msg->info.origin.position.y;
  current_map_.resolution = msg->info.resolution;

  current_map_.grid.resize(current_map_.width * current_map_.height, 0);
  ignored_cells_.resize(current_map_.height,
                        std::vector<bool>(current_map_.width, false));
  valid_lane_cells_.clear();

  for (int y = 0; y < current_map_.height; ++y) {
    for (int x = 0; x < current_map_.width; ++x) {
      int i = gridToIndex(x, y, current_map_.width);
      if (!ignored_cells_[y][x]) {
        if (msg->data[i] == 100) {
          current_map_.grid[i] = 1;
          valid_lane_cells_.insert(GridCell(x, y));
        } else {
          current_map_.grid[i] = 0;
        }
      } else {
        current_map_.grid[i] = 0;
      }
    }
  }
  map_received_ = true;
}

void GoalGenerator::modifyCallback(const std_msgs::String::ConstPtr &msg) {
  if (msg->data == "resize") {
    std::lock_guard<std::mutex> lock(map_mutex_);
    std::lock_guard<std::mutex> pose_lock(pose_mutex_);

    ROS_INFO("Got resize message...");

    ignored_cells_.clear();
    ignored_cells_.resize(current_map_.height,
                          std::vector<bool>(current_map_.width, false));
  }
}

void GoalGenerator::gpsGoalCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  Point gps_goal;
  gps_goal.x = msg->pose.position.x;
  gps_goal.y = msg->pose.position.y;
  gps_counter_ = msg->pose.position.z;

  if ((gps_counter_ > 1 && gps_counter_ <= 4) ||
      (distancePoint(gps_goal, current_pose_.world_position) < 5 &&
       gps_counter_ == 1)) {
    paused_ = true;
    publishGoal(gps_goal.x, gps_goal.y, current_pose_.yaw);
    publishMarker(gps_goal.x, gps_goal.y, 1);
  } else {
    paused_ = false;
  }
}

void GoalGenerator::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(pose_mutex_);

  current_pose_.world_position.x = msg->pose.pose.position.x;
  current_pose_.world_position.y = msg->pose.pose.position.y;
  current_pose_.map_position =
      pointToGrid(current_pose_.world_position, current_map_);

  double roll, pitch;
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, current_pose_.yaw);

  odom_received_ = true;
}

void GoalGenerator::publishMarker(double x, double y, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "robot/odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_goal_marker";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.b = 1.0;
  marker.color.g = 0.0;

  marker_pub_.publish(marker);
}

void GoalGenerator::publishGoal(double x, double y, double yaw) {
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "robot/odom";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.orientation.x = q.x();
  goal.pose.orientation.y = q.y();
  goal.pose.orientation.z = q.z();
  goal.pose.orientation.w = q.w();

  goal_pub_.publish(goal);
}

void GoalGenerator::processGoal() {
  while (ros::ok()) {
    if (!isTransformAvailable()) {
      ROS_WARN_THROTTLE(
          5.0,
          "Waiting for transform between robot/odom and robot/base_link...");
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    if (odom_received_ && map_received_ && !paused_) {
      GridCell goal_cell;
      GridCell closest_lane_cell;

      {
        std::lock_guard<std::mutex> map_lock(map_mutex_);
        std::lock_guard<std::mutex> pose_lock(pose_mutex_);

        if (last_goal_cell_ == GridCell(-1, -1)) {
          closest_lane_cell =
              findClosestLaneParallel(current_map_.grid, current_map_.width,
                                      current_pose_.map_position);
          goal_cell =
              exploreFarthestParallel(valid_lane_cells_, closest_lane_cell, 5);
        } else {
          if (valid_lane_cells_.empty()) {
            goal_cell = last_goal_cell_;
          } else {
            closest_lane_cell = findClosestLaneParallel(
                current_map_.grid, current_map_.width, last_goal_cell_);
            goal_cell = exploreFarthestParallel(valid_lane_cells_,
                                                closest_lane_cell, 5);
          }
        }

        updateIgnoredCells(current_pose_.map_position, goal_cell);
        processGoalCell(goal_cell);
      }

      map_received_ = false;
      odom_received_ = false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "goal_gen");
  ros::NodeHandle nh;

  ROS_INFO("Goal Gen Started.");

  GoalGenerator goal_generator(nh);
  ros::spin();

  return 0;
}
