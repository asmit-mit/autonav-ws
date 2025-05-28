#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/Marker.h"

#include <cmath>
#include <queue>
#include <thread>
#include <unordered_set>
#include <vector>

struct WorldPose {
  double x;
  double y;

  WorldPose(double x_val = 0.0, double y_val = 0.0) : x(x_val), y(y_val) {}
};

struct MapPose {
  int x;
  int y;

  MapPose(int x_val = 0, int y_val = 0) : x(x_val), y(y_val) {}
};

struct Map {
  int width, height;
  double resolution;
  WorldPose origin;
  std::vector<std::vector<int>> grid;
};

struct BotPose {
  WorldPose world_pose;
  MapPose map_pose;
  double roll, pitch, yaw;
};

class Utils {
public:
  Utils() {}

  static double getAngleRadians(const WorldPose &a, const WorldPose &b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::atan2(dy, dx);
  }

  static double mapDistance(const MapPose &a, const MapPose &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  static double worldDistance(const WorldPose &a, const WorldPose &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  static MapPose
  getMapPoseFromWorldPose(const WorldPose &pose, const Map &map) {
    MapPose map_pose;
    map_pose.x = static_cast<int>((pose.x - map.origin.x) / map.resolution);
    map_pose.y = static_cast<int>((pose.y - map.origin.y) / map.resolution);
    return map_pose;
  }

  static WorldPose
  getWorldPoseFromMapPose(const MapPose &pose, const Map &map) {
    WorldPose world_pose;
    world_pose.x = map.origin.x + (pose.x * map.resolution);
    world_pose.y = map.origin.y + (pose.y * map.resolution);
    return world_pose;
  }

  static MapPose findClosestForValue(
      const MapPose &pose, const Map &map, int radius, int value
  ) {
    const int dx[] = {0, 1, 0, -1, 1, -1, 1, -1};
    const int dy[] = {-1, 0, 1, 0, -1, 1, 1, -1};

    std::queue<std::pair<MapPose, int>> q;

    auto hashFunc = [&map](const MapPose &p) { return p.y * map.width + p.x; };

    std::unordered_set<int> visited;

    if (pose.x < 0 || pose.x >= map.width || pose.y < 0 ||
        pose.y >= map.height) {
      return MapPose(-1, -1);
    }

    q.push({pose, 0});
    visited.insert(hashFunc(pose));

    while (!q.empty()) {
      auto pair = q.front();
      q.pop();

      MapPose current = pair.first;
      int distance    = pair.second;

      if (map.grid[current.y][current.x] == value) {
        return current;
      }

      if (distance >= radius) {
        continue;
      }

      for (int i = 0; i < 4; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        MapPose neighbor(nx, ny);
        int hash = hashFunc(neighbor);

        if (nx >= 0 && nx < map.width && ny >= 0 && ny < map.height &&
            visited.find(hash) == visited.end()) {
          visited.insert(hash);
          q.push({neighbor, distance + 1});
        }
      }
    }

    return MapPose(-1, -1);
  }

  static MapPose exploreMiddleLane(const MapPose &start, const Map &map) {
    const int dx[] = {0, 1, 0, -1, 1, -1, 1, -1};
    const int dy[] = {-1, 0, 1, 0, -1, 1, 1, -1};

    std::queue<MapPose> q;
    std::unordered_set<int> visited;

    auto hashFunc = [&map](const MapPose &p) { return p.y * map.width + p.x; };

    q.push(start);
    visited.insert(hashFunc(start));

    MapPose last_cell = start;

    while (!q.empty()) {
      MapPose current = q.front();
      q.pop();

      last_cell = current;

      for (int i = 0; i < 8; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        MapPose neighbor(nx, ny);
        int hash = hashFunc(neighbor);

        if (nx >= 0 && nx < map.width && ny >= 0 && ny < map.height &&
            visited.find(hash) == visited.end() && map.grid[ny][nx] == 100) {
          visited.insert(hash);
          q.push(neighbor);
        }
      }

      const int search_radius = 10;
      for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
          int nx = current.x + dx;
          int ny = current.y + dy;
          MapPose neighbor(nx, ny);
          int hash = hashFunc(neighbor);

          if (nx >= 0 && nx < map.width && ny >= 0 && ny < map.height &&
              visited.find(hash) == visited.end() && map.grid[ny][nx] == 100) {
            visited.insert(hash);
            q.push(neighbor);
          }
        }
      }
    }

    return last_cell;
  }

  static void removeMapBehindBot(
      Map &map, const WorldPose &bot_pose, double angle, int height_to_remove,
      int width_to_remove
  ) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;

    auto process_rows = [&](int start_y, int end_y) {
      for (int y = start_y; y < end_y; y++) {
        for (int x = 0; x < map.width; x++) {
          MapPose map_pose(x, y);
          WorldPose world_pose = Utils::getWorldPoseFromMapPose(map_pose, map);

          double angle_to_pixel = Utils::getAngleRadians(bot_pose, world_pose);

          double angle_diff = angle_to_pixel - angle;

          while (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;
          while (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;

          double dx = world_pose.x - bot_pose.x;
          double dy = world_pose.y - bot_pose.y;

          if (std::abs(angle_diff) > M_PI / 2 &&
              std::abs(dx) <= width_to_remove &&
              std::abs(dy) <= height_to_remove) {
            map.grid[y][x] = -1;
          }
        }
      }
    };

    process_rows(0, map.height);
  }
};

class GoalGenner {
private:
  ros::Timer timer;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};

  ros::Subscriber map_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber gps_sub;

  ros::Publisher marker_pub;
  ros::Publisher map_pub;
  ros::Publisher goal_pub;
  ros::Publisher modify_pub;

  bool have_map     = false;
  bool have_odom    = false;
  bool finished_gps = false;

  int explore_distance{50};
  int pose_log_offset{3};
  int pose_goal_offset{10};
  int total_gps_goals{3};
  int gps_capture_distance{5};
  std::string mode = "automated";

  Map current_map;
  BotPose prev_pose;
  BotPose current_pose;
  BotPose first_pose;

  bool paused = false;

  ros::Time last_map_time;
  ros::Time last_odom_time;
  double topic_timeout{5.0};

  void createSphereMarker(double x, double y, int id) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "odom";
    marker.header.stamp    = ros::Time::now();

    marker.ns = "sphere_markers";
    marker.id = id;

    marker.type = visualization_msgs::Marker::SPHERE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker_pub.publish(marker);
  }

  void
  createArrowMarker(double x1, double y1, double x2, double y2, int id = 0) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "odom";
    marker.header.stamp    = ros::Time::now();

    marker.ns = "arrow_markers";
    marker.id = id;

    marker.type   = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start, end;
    start.x = x1;
    start.y = y1;
    start.z = 0;
    end.x   = x2;
    end.y   = y2;
    end.z   = 0;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.1;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker_pub.publish(marker);
  }

  void publishMap(const Map &map) {
    nav_msgs::OccupancyGrid grid_msg;

    grid_msg.header.stamp    = ros::Time::now();
    grid_msg.header.frame_id = "odom";

    grid_msg.info.resolution = map.resolution;
    grid_msg.info.width      = map.width;
    grid_msg.info.height     = map.height;

    grid_msg.info.origin.position.x = map.origin.x;
    grid_msg.info.origin.position.y = map.origin.y;
    grid_msg.info.origin.position.z = 0.0;

    grid_msg.info.origin.orientation.x = 0.0;
    grid_msg.info.origin.orientation.y = 0.0;
    grid_msg.info.origin.orientation.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;

    grid_msg.data.resize(map.width * map.height);

    for (int y = 0; y < map.height; y++) {
      for (int x = 0; x < map.width; x++) {
        int index            = y * map.width + x;
        grid_msg.data[index] = map.grid[y][x];
      }
    }

    map_pub.publish(grid_msg);
  }

  void publishGoal(const WorldPose &target, double yaw) {
    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "odom";
    goal.header.stamp    = ros::Time::now();

    goal.pose.position.x = target.x;
    goal.pose.position.y = target.y;
    goal.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    goal_pub.publish(goal);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    last_map_time = ros::Time::now();

    current_map.width      = msg->info.width;
    current_map.height     = msg->info.height;
    current_map.resolution = msg->info.resolution;
    current_map.origin.x   = msg->info.origin.position.x;
    current_map.origin.y   = msg->info.origin.position.y;
    current_map.grid.resize(
        current_map.height, std::vector<int>(current_map.width, -1)
    );

    for (int y = 0; y < current_map.height; y++) {
      for (int x = 0; x < current_map.width; x++) {
        int index              = y * current_map.width + x;
        current_map.grid[y][x] = msg->data[index];
      }
    }

    have_map = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    last_odom_time = ros::Time::now();

    current_pose.world_pose.x = msg->pose.pose.position.x;
    current_pose.world_pose.y = msg->pose.pose.position.y;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_pose.roll  = roll;
    current_pose.pitch = pitch;
    current_pose.yaw   = yaw;

    if (have_map) {
      current_pose.map_pose =
          Utils::getMapPoseFromWorldPose(current_pose.world_pose, current_map);
    }

    if (!have_odom ||
        Utils::worldDistance(prev_pose.world_pose, current_pose.world_pose) >
            pose_log_offset) {
      prev_pose = current_pose;
    }

    if (!have_odom) {
      first_pose = current_pose;
    }

    have_odom = true;
  }

  void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr msg) {
    if (mode == "lane_follow")
      return;

    WorldPose gps_wp;
    gps_wp.x = msg->pose.position.x;
    gps_wp.y = msg->pose.position.y;

    int gps_counter = msg->pose.position.z;

    if (mode == "gps") {
      publishGoal(gps_wp, current_pose.yaw);
      if (gps_counter == total_gps_goals + 1) {
        publishGoal(current_pose.world_pose, current_pose.yaw);
        ros::shutdown();
      }
      return;
    }

    if (gps_counter > 1 && gps_counter <= total_gps_goals ||
        ((Utils::worldDistance(current_pose.world_pose, gps_wp) <
          gps_capture_distance) &&
         gps_counter == 1)) {
      paused = true;
      createSphereMarker(gps_wp.x, gps_wp.y, 2);
      publishGoal(gps_wp, current_pose.yaw);
    }

    if (gps_counter > total_gps_goals) {
      finished_gps = true;
      paused       = false;
    }
  }

  void timerCallback(const ros::TimerEvent &) {
    ros::Time current_time = ros::Time::now();
    bool map_active  = (current_time - last_map_time).toSec() < topic_timeout;
    bool odom_active = (current_time - last_odom_time).toSec() < topic_timeout;

    if (!map_active) {
      ROS_WARN_THROTTLE(
          3.0, "No recent map data received, halting goal generation"
      );
      have_map = false;
    }

    if (!odom_active) {
      ROS_WARN_THROTTLE(
          3.0, "No recent odom data received, halting goal generation"
      );
      have_odom = false;
    }

    if (paused) {
      ROS_INFO("Paused");
      return;
    } else {
      ROS_INFO("Goal Gen Active");
      if (map_active && odom_active) {
        findGoal();
      }
    }
  }

public:
  GoalGenner() {
    private_nh.param<int>("explore_distance", explore_distance, 50);
    private_nh.param<int>("pose_log_offset", pose_log_offset, 3);
    private_nh.param<int>("pose_goal_offset", pose_goal_offset, 10);
    private_nh.param<int>("total_gps_goals", total_gps_goals, 3);
    private_nh.param<int>("gps_capture_distance", gps_capture_distance, 5);
    private_nh.param<std::string>("mode", mode, "automated");

    ROS_INFO("[goal_gen] Taking total gps goals as %d", total_gps_goals);
    ROS_INFO("[goal_gen] Taking mode as %s", mode.c_str());

    last_map_time  = ros::Time::now();
    last_odom_time = ros::Time::now();

    map_sub  = nh.subscribe("/map", 1, &GoalGenner::mapCallback, this);
    odom_sub = nh.subscribe("/odom", 1, &GoalGenner::odomCallback, this);
    gps_sub  = nh.subscribe("/gps_goal", 1, &GoalGenner::gpsCallback, this);

    timer =
        nh.createTimer(ros::Duration(0.2), &GoalGenner::timerCallback, this);

    marker_pub = nh.advertise<visualization_msgs::Marker>("goal_marker", 10);
    modify_pub = nh.advertise<std_msgs::String>("modify_pub", 10);

    /* map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/modified_map", 10); */
    goal_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  }

  void findGoal() {
    if (!(have_map && have_odom)) {
      return;
    }

    if (mode == "gps") {
      ROS_INFO("[goal_gen] Gps Mode Active");
      return;
    }

    if (finished_gps &&
        Utils::worldDistance(current_pose.world_pose, first_pose.world_pose) <
            5) {
      publishGoal(first_pose.world_pose, first_pose.yaw);
      ROS_INFO("[goal_gen] Course Completed");
      ros::shutdown();
    }

    double theta;
    if (Utils::worldDistance(current_pose.world_pose, prev_pose.world_pose) <
        0.1) {
      theta = current_pose.yaw;
    } else {
      theta =
          Utils::getAngleRadians(prev_pose.world_pose, current_pose.world_pose);
    }

    Utils::removeMapBehindBot(
        current_map, current_pose.world_pose, theta, 10, 10
    );

    MapPose nearest_lane = Utils::findClosestForValue(
        current_pose.map_pose, current_map, explore_distance, 100
    );

    MapPose farthest_lane = Utils::exploreMiddleLane(nearest_lane, current_map);
    WorldPose wp = Utils::getWorldPoseFromMapPose(farthest_lane, current_map);

    wp.x       = wp.x + pose_goal_offset * cos(theta);
    wp.y       = wp.y + pose_goal_offset * sin(theta);
    MapPose mp = Utils::getMapPoseFromWorldPose(wp, current_map);

    if (mp.x >= current_map.height - 50 || mp.x < 50 ||
        mp.y >= current_map.width - 50 || mp.y < 50) {
      std_msgs::String resize_msg;
      resize_msg.data = "relocate";
      modify_pub.publish(resize_msg);
    }

    mp.x = std::max(0, std::min(mp.x, current_map.height));
    mp.y = std::max(0, std::min(mp.y, current_map.width));

    if (current_map.grid[mp.y][mp.x] == 100) {
      mp = Utils::findClosestForValue(mp, current_map, explore_distance, 0);
    }

    if (mode == "lane_follow" &&
        Utils::worldDistance(wp, current_pose.world_pose) < 7) {
      ROS_INFO("[mapper] Reached the end of the lane");
      publishGoal(current_pose.world_pose, current_pose.yaw);
      ros::shutdown();
    }

    WorldPose wp_final = Utils::getWorldPoseFromMapPose(mp, current_map);

    createSphereMarker(wp_final.x, wp_final.y, 1);
    publishGoal(wp_final, theta);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "goal_gen");

  ROS_INFO("[goal_gen] Goal Gen Node Started.");

  GoalGenner goal_gen;

  ros::spin();

  return 0;
}
