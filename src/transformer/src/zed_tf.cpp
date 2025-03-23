#include <deque>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class ImuTransformer {
public:
  ImuTransformer() : received_imu(false), running_sum(0.0), window_size(30) {
    ros::NodeHandle nh;
    imu_sub = nh.subscribe("/zed_node/imu/data", 10,
                           &ImuTransformer::imuCallback, this);
    tf_broadcaster = new tf2_ros::TransformBroadcaster();
    monitor_timer = nh.createTimer(ros::Duration(0.5),
                                   &ImuTransformer::monitorCallback, this);
    last_imu_time = ros::Time::now();
    ROS_INFO("IMU Transformer initialized. Waiting for data...");
  }

  ~ImuTransformer() { delete tf_broadcaster; }

private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    received_imu = true;
    last_imu_time = ros::Time::now();

    last_imu_msg = *msg;

    processAndPublishTransform(msg);
  }

  void processAndPublishTransform(const sensor_msgs::Imu::ConstPtr &msg) {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "robot/base_link";
    transform_stamped.child_frame_id = "base_link";

    tf2::Quaternion imu_orientation(msg->orientation.x, msg->orientation.y,
                                    msg->orientation.z, msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(imu_orientation).getRPY(roll, pitch, yaw);

    if (pitch_window.size() >= window_size) {
      running_sum -= pitch_window.front();
      pitch_window.pop_front();
    }

    pitch_window.push_back(pitch);
    running_sum += pitch;
    double avg_pitch = running_sum / pitch_window.size();

    tf2::Quaternion corrected_orientation;
    corrected_orientation.setRPY(0.0, avg_pitch, 0.0);

    transform_stamped.transform.rotation.x = corrected_orientation.x();
    transform_stamped.transform.rotation.y = corrected_orientation.y();
    transform_stamped.transform.rotation.z = corrected_orientation.z();
    transform_stamped.transform.rotation.w = corrected_orientation.w();

    transform_stamped.transform.translation.x = -0.114;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 1.0;

    tf_broadcaster->sendTransform(transform_stamped);
  }

  void publishLastImu() {
    if (received_imu) {
      sensor_msgs::Imu::ConstPtr msg_ptr(new sensor_msgs::Imu(last_imu_msg));

      ROS_INFO_THROTTLE(5.0, "Using last known IMU data due to timeout");

      processAndPublishTransform(msg_ptr);
    }
  }

  void monitorCallback(const ros::TimerEvent &) {
    ros::Time current_time = ros::Time::now();

    if (!received_imu) {
      ROS_WARN_THROTTLE(
          5.0, "No IMU data has been received yet from /zed_node/imu/data");
    } else {
      double time_since_last_imu = (current_time - last_imu_time).toSec();
      if (time_since_last_imu > 1.0) {
        ROS_WARN_THROTTLE(5.0, "No IMU data received for %.1f seconds",
                          time_since_last_imu);

        publishLastImu();
      }
    }

    if (received_imu && pitch_window.size() < window_size) {
      ROS_WARN_THROTTLE(5.0,
                        "Pitch averaging window not yet full (%zu/%zu samples)",
                        pitch_window.size(), window_size);
    }
  }

  ros::Subscriber imu_sub;
  tf2_ros::TransformBroadcaster *tf_broadcaster;
  ros::Timer monitor_timer;
  ros::Time last_imu_time;
  bool received_imu;
  std::deque<double> pitch_window;
  double running_sum;
  const size_t window_size;

  sensor_msgs::Imu last_imu_msg;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "zed_tf");
  ImuTransformer transformer;
  ros::spin();
  return 0;
}
