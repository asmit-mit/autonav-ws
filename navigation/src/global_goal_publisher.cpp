#include <ros/ros.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>
#include <cmath>

using namespace std;

ros::Publisher pose_pub;
ros::Subscriber odom_sub;
double ref_lat, ref_lon, ref_alt;
double ref_northing, ref_easting;
string ref_zone;
vector<tuple<double, double, double>> goals;  // lat, lon, alt
size_t current_goal_index = 0;
double publish_rate;
const double GOAL_REACHED_THRESHOLD = 3; // 5 meters
geometry_msgs::Pose current_pose;
bool pose_received = false;
ros::Time start_time;
bool timer_started = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose = msg->pose.pose;
    pose_received = true;
}

bool isGoalReached(const geometry_msgs::Pose& current, const geometry_msgs::Pose& goal)
{
    double dx = current.position.x - goal.position.x;
    double dy = current.position.y - goal.position.y;
    double dz = current.position.z - goal.position.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    return distance < GOAL_REACHED_THRESHOLD;
}

void publishCurrentGoal()
{
    if (current_goal_index >= goals.size()) {
        ROS_INFO("All goals reached. Stopping.");
        ros::shutdown();
        return;
    }

    double latitude = get<0>(goals[current_goal_index]);
    double longitude = get<1>(goals[current_goal_index]);
    double altitude = get<2>(goals[current_goal_index]);
    double northing, easting;
    string zone;

    RobotLocalization::NavsatConversions::LLtoUTM(latitude, longitude, northing, easting, zone);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = ref_northing - northing;
    pose_msg.pose.position.y = ref_easting - easting;
    pose_msg.pose.position.z = current_goal_index  + 1;
    pose_msg.pose.orientation.w = 1.0;  // No rotation

    // ROS_INFO("Publishing Goal %zu: x=%.2f, y=%.2f, z=%.2f", 
    //          current_goal_index + 1, pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
    pose_pub.publish(pose_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_goal_publisher");
    ros::NodeHandle nh("~");

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 10);
    odom_sub = nh.subscribe("/odom", 1, odomCallback);

    nh.param<double>("ref_lat", ref_lat, 49.899999);
    nh.param<double>("ref_lon", ref_lon, 8.900001);
    nh.param<double>("ref_alt", ref_alt, 0.194685);

    nh.param<double>("publish_rate", publish_rate, 10.0);  // Default to 1 Hz

    // Define the 5 goals (latitude, longitude, altitude tuples)
    goals = {
        {49.899889574744286, 8.899697616835839, 0.19473543564318233},
        {49.89993349214121, 8.899853275524414, 0.19471782080005984}, // nomans land exit
        {49.90006261801672, 8.899855871989393, 0.19437419270357567}, // ramp entry
        {49.9001499523225, 8.899686049564702, 0.19474662264463902} // no mans land entry
    };

    RobotLocalization::NavsatConversions::LLtoUTM(ref_lat, ref_lon, ref_northing, ref_easting, ref_zone);

    ros::Rate rate(publish_rate);

    while (ros::ok())
    {
        if (pose_received)
        {
            double latitude = get<0>(goals[current_goal_index]);
            double longitude = get<1>(goals[current_goal_index]);
            double altitude = get<2>(goals[current_goal_index]);
            double northing, easting;
            string zone;

            RobotLocalization::NavsatConversions::LLtoUTM(latitude, longitude, northing, easting, zone);

            geometry_msgs::Pose goal_pose;
            goal_pose.position.x = ref_northing - northing;
            goal_pose.position.y = ref_easting - easting;
            goal_pose.position.z = ref_alt - altitude;

            if (isGoalReached(current_pose, goal_pose))
            {
                ROS_INFO("Goal %zu reached!", current_goal_index + 1);
                
                if (!timer_started) {
                    start_time = ros::Time::now();
                    timer_started = true;
                    ROS_INFO("Timer started!");
                }
                
                current_goal_index++;
                if (current_goal_index >= goals.size())
                {
                    ros::Duration elapsed_time = ros::Time::now() - start_time;
                    ROS_INFO("All goals reached. Total time: %.2f seconds", elapsed_time.toSec());
                    break;
                }
            }

            publishCurrentGoal();
        }
        else
        {
            ROS_WARN_THROTTLE(5, "Waiting for odometry data...");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}