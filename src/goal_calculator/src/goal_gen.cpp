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

#include "utils.cpp"

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
    current_map.grid       = std::move(msg->data);

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

    if (nearest_lane.x == -1 && nearest_lane.y == -1) {
      return;
    }

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

    if (current_map.getValue(mp.x, mp.y) == 100) {
      mp = Utils::findClosestForValue(mp, current_map, explore_distance, 0);
    }

    if (mode == "lane_follow" &&
        Utils::worldDistance(wp, current_pose.world_pose) < 7) {
      ROS_INFO("[goal_gen] Reached the end of the lane");
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
