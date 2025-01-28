#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

using namespace std;

struct GridCell {
  int x, y;
  GridCell(int x = 0, int y = 0) : x(x), y(y) {}

  bool operator==(const GridCell &other) const {
    return x == other.x && y == other.y;
  }
};

struct GridCellHash {
  size_t operator()(const GridCell &cell) const {
    return hash<int>()(cell.x) ^ (hash<int>()(cell.y) << 1);
  }
};

struct Point {
  double x, y;
};

struct Goal {
  Point pose;
  int counter;
};

struct Map {
  int width, height;
  GridCell origin;
  double resolution;
  vector<int> grid;
};

struct BotPose {
  Point world_position;
  GridCell map_position;
  double yaw;
};

vector<vector<bool>> ignored_cells;
unordered_set<GridCell, GridCellHash> valid_lane_cells;

GridCell last_goal_cell = GridCell(-1, -1);

BotPose current_pose;
Map current_map;

Point global_goal;

atomic<bool> odom_received(false);
atomic<bool> map_received(false);

ros::Publisher marker_pub;
ros::Publisher goal_pub;

mutex map_mutex;
mutex pose_mutex;

bool paused = false;
bool switch_one = false;
bool switch_two = false;

bool final_goal_reached = false;
bool publish_final_goal = false;
bool stop_node = false;

Goal current_goal;

ros::Time start_time;
ros::Time end_time;

inline int gridToIndex(int x, int y, int width) { return y * width + x; }

inline GridCell indexToGrid(int index, int width) {
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

double distanceGrid(const GridCell &a, const GridCell &b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

double distancePoint(const Point &a, const Point &b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

GridCell findClosestLaneParallel(const vector<int> &map, int width,
                                 GridCell start) {
  const int num_threads = 8;
  vector<future<GridCell>> futures;

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
    futures.push_back(async(launch::async, searchSection, startY, endY));
  }

  GridCell closest(-1, -1);
  double minDist = numeric_limits<double>::max();
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

GridCell
exploreFarthestParallel(const unordered_set<GridCell, GridCellHash> &cluster,
                        const GridCell &start, double eps) {
  const int num_threads = 4;
  vector<future<pair<GridCell, double>>> futures;

  auto exploreSection =
      [&](const vector<GridCell> &cells) -> pair<GridCell, double> {
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

  vector<GridCell> clusterVec(cluster.begin(), cluster.end());
  int sectionSize = clusterVec.size() / num_threads;
  for (int i = 0; i < num_threads; ++i) {
    int startIdx = i * sectionSize;
    int endIdx =
        (i == num_threads - 1) ? clusterVec.size() : (i + 1) * sectionSize;
    vector<GridCell> section(clusterVec.begin() + startIdx,
                             clusterVec.begin() + endIdx);
    futures.push_back(async(launch::async, exploreSection, section));
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

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  lock_guard<mutex> lock(map_mutex);

  if (!map_received) {
    current_map.height = msg->info.height;
    current_map.width = msg->info.width;
    current_map.origin.x = msg->info.origin.position.x;
    current_map.origin.y = msg->info.origin.position.y;
    current_map.resolution = msg->info.resolution;

    current_map.grid.resize(current_map.width * current_map.height, 0);
    ignored_cells.resize(current_map.height,
                         vector<bool>(current_map.width, false));
  }

  valid_lane_cells.clear();

  for (int y = 0; y < current_map.height; ++y) {
    for (int x = 0; x < current_map.width; ++x) {
      int i = gridToIndex(x, y, current_map.width);
      if (!ignored_cells[y][x]) {
        if (msg->data[i] == 100) {
          current_map.grid[i] = 1;
          valid_lane_cells.insert(GridCell(x, y));
        } else {
          current_map.grid[i] = 0;
        }
      } else {
        current_map.grid[i] = 0;
      }
    }
  }
  map_received = true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  lock_guard<mutex> lock(pose_mutex);

  current_pose.world_position.x = msg->pose.pose.position.x;
  current_pose.world_position.y = msg->pose.pose.position.y;

  current_pose.map_position =
      pointToGrid(current_pose.world_position, current_map);

  double roll, pitch;
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, current_pose.yaw);

  odom_received = true;
}

void globalGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  current_goal = {{msg->pose.position.x, msg->pose.position.y},
                  static_cast<int>(msg->pose.position.z)};
}

void publishMarker(double x, double y, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "robot/odom";
  marker.header.stamp = ros::Time::now();
  marker.ns = "local_goal";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.b = 1.0;
  marker.color.g = 0.0;

  marker_pub.publish(marker);
}

void publishGoal(double x, double y, double yaw) {
  geometry_msgs::PoseStamped goal;

  goal.header.frame_id = "robot/odom";
  goal.header.stamp = ros::Time::now();

  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;

  // Convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.orientation.x = q.x();
  goal.pose.orientation.y = q.y();
  goal.pose.orientation.z = q.z();
  goal.pose.orientation.w = q.w();

  goal_pub.publish(goal);
}

void processGoal() {
  while (ros::ok()) {
    if (odom_received && map_received) {
      GridCell goal_cell;
      GridCell closest_lane_cell;

      // double global_goal_dist = distancePoint(current_pose.world_position,
      // current_goal.pose); if (current_goal.counter == 1 && global_goal_dist <
      // 15 && switch_one == false) {
      //     paused = true;
      //     switch_one = true;
      //
      //     for (size_t i = 0; i < ignored_cells.size(); i++) {
      //         for (size_t j = 600; j < ignored_cells.at(0).size(); j++) {
      //             ignored_cells.at(i).at(j) = true;
      //         }
      //     }
      //
      //     // cout << "top half is not ignored" << endl;
      // }
      //
      // // Point test = {-4.813397850951263, 10.01533066110384};
      //
      // // GridCell tes = pointToGrid(test, current_map);
      //
      // // cout << tes.x << " " << tes.y << endl;
      //
      // if(switch_one && !switch_two){
      //     // cout << "inside no mans land" << endl;
      //     publishGoal(current_goal.pose.x, current_goal.pose.y, 1);
      //     if(current_goal.counter == 4 && distancePoint(current_goal.pose,
      //     current_pose.world_position) <= 2) {
      //         paused = false;
      //         switch_two = true;
      //
      //         // cout << "exited no mans land" << endl;
      //
      //         last_goal_cell = GridCell(-1, -1);
      //     }
      // }
      //
      // Point final_pt = {-12.3709185, -23.195704};
      // Point end_course = {-0.143247067, -23.5012827};
      //
      // if(switch_two && distancePoint(current_pose.world_position, final_pt) <
      // 4) {
      //     paused = true;
      //     publish_final_goal = true;
      // }
      //
      // if (publish_final_goal) {
      //     publishGoal(end_course.x, end_course.y, -1);
      //
      //     if(distancePoint(current_pose.world_position, end_course) < 2 &&
      //     !stop_node) {
      //         final_goal_reached = true;
      //         end_time = ros::Time::now();
      //
      //         ros::Duration time_taken = end_time - start_time;
      //         ROS_INFO("==========================================");
      //         ROS_INFO("FINAL GOAL REACHED!");
      //         ROS_INFO_STREAM("Time taken to complete the course: " <<
      //         time_taken.toSec() << " seconds");
      //         ROS_INFO("==========================================");
      //
      //         ros::Duration(1.0).sleep();
      //
      //         stop_node = true;
      //         publish_final_goal = false;
      //     }
      // }
      //
      // // cout << current_pose.world_position.x << " " <<
      // current_pose.world_position.y << endl;
      // // cout << current_goal.x << " " << current_goal.y << endl;

      if (true) {
        {
          lock_guard<mutex> map_lock(map_mutex);
          lock_guard<mutex> pose_lock(pose_mutex);

          if (last_goal_cell == GridCell(-1, -1)) {
            closest_lane_cell = findClosestLaneParallel(
                current_map.grid, current_map.width, current_pose.map_position);
            goal_cell =
                exploreFarthestParallel(valid_lane_cells, closest_lane_cell, 5);

            int size_of_crop = static_cast<int>(
                distanceGrid(current_pose.map_position, goal_cell));

            for (int i = max(0, current_pose.map_position.x - size_of_crop);
                 i <= min(current_map.width - 1,
                          current_pose.map_position.x + size_of_crop);
                 i++) {
              for (int j = max(0, current_pose.map_position.y - size_of_crop);
                   j <= min(current_map.height - 1,
                            current_pose.map_position.y + size_of_crop);
                   j++) {
                ignored_cells[j][i] = true;
              }
            }
          } else {
            if (valid_lane_cells.empty()) {
              goal_cell = last_goal_cell;
            } else {
              closest_lane_cell = findClosestLaneParallel(
                  current_map.grid, current_map.width, last_goal_cell);
              goal_cell = exploreFarthestParallel(valid_lane_cells,
                                                  closest_lane_cell, 5);

              int size_of_crop = static_cast<int>(
                  distanceGrid(current_pose.map_position, goal_cell));

              for (int i = max(0, current_pose.map_position.x - size_of_crop);
                   i <= min(current_map.width - 1,
                            current_pose.map_position.x + size_of_crop);
                   i++) {
                for (int j = max(0, current_pose.map_position.y - size_of_crop);
                     j <= min(current_map.height - 1,
                              current_pose.map_position.y + size_of_crop);
                     j++) {
                  ignored_cells[j][i] = true;
                }
              }
            }
          }

          if (goal_cell == GridCell(0, 0)) {
            goal_cell = last_goal_cell;
          }

          int angle = 90;
          if (current_pose.yaw < 0) {
            angle = -1 * angle;
          }

          GridCell goal_cell_e;
          goal_cell_e.x = goal_cell.x + 120 * cos(current_pose.yaw + 0);
          goal_cell_e.y = goal_cell.y + 120 * sin(current_pose.yaw + 0);

          Point goal_point = gridToPoint(goal_cell_e, current_map);
          publishMarker(goal_point.x, goal_point.y, 1);
          publishGoal(goal_point.x, goal_point.y, current_pose.yaw);

          // cout << "Goal: " << goal_cell.x << " " << goal_cell.y << endl;
          // cout << "Bot Pose: " << current_pose.map_position.x << " " <<
          // current_pose.map_position.y << endl; cout << "Closest lane cell: "
          // << closest_lane_cell.x << " " << closest_lane_cell.y << endl; cout
          // << "Ignored Cells: " << count_if(ignored_cells.begin(),
          // ignored_cells.end(), [](const vector<bool>& row) { return
          // count(row.begin(), row.end(), true); }) << endl; cout << "Valid
          // lane cells: " << valid_lane_cells.size() << endl;

          last_goal_cell = goal_cell;
        }
      }

      // cout << endl;

      map_received = false;
      odom_received = false;
    }

    this_thread::sleep_for(chrono::milliseconds(200));
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "goal_gen");
  ros::NodeHandle nh;

  ROS_INFO("Goal Gen Started.");

  start_time = ros::Time::now();

  goal_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  marker_pub = nh.advertise<visualization_msgs::Marker>("nav/local_goal", 10);

  ros::Subscriber map_sub = nh.subscribe("nav/global_map", 1, mapCallback);
  ros::Subscriber odom_sub =
      nh.subscribe("robot/dlo/odom_node/odom", 1, odomCallback);
  // ros::Subscriber global_goal_sub = nh.subscribe("/goal", 1,
  // globalGoalCallback);

  thread goal_thread(processGoal);

  ros::spin();

  goal_thread.join();

  // if (final_goal_reached) {
  //     ros::Duration time_taken = ros::Time::now() - start_time;
  //     ROS_INFO("==========================================");
  //     ROS_INFO("Final time (printed from main):");
  //     ROS_INFO_STREAM("Time taken to complete the course: " <<
  //     time_taken.toSec() << " seconds");
  //     ROS_INFO("==========================================");
  // } else {
  //     ROS_WARN("Goal Gen node shutting down before reaching final goal.");
  //     ROS_INFO_STREAM("Total runtime: " << (ros::Time::now() -
  //     start_time).toSec() << " seconds");
  // }

  return 0;
}
