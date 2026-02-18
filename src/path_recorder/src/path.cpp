#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include "path_recorder.hpp"

namespace {

path_recorder::PathRecorder g_recorder;
geometry_msgs::PoseStamped g_last_pose;
bool g_has_last_pose = false;
double g_record_distance_threshold = 2.0;

/**
 * @brief Records a new waypoint when the vehicle moved far enough from the previous one.
 */
void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
  if (!g_has_last_pose) {
    g_last_pose = *pose_msg;
    g_has_last_pose = true;

    // Always persist the first sample as the initial reference waypoint.
    g_recorder.RecordPose(*pose_msg);
    return;
  }

  const double moved = path_recorder::PathRecorder::PlanarDistance(
      pose_msg->pose.position, g_last_pose.pose.position);

  if (moved < g_record_distance_threshold) {
    return;
  }

  if (g_recorder.RecordPose(*pose_msg)) {
    g_last_pose = *pose_msg;
    ROS_INFO("Recorded waypoint at x=%.3f y=%.3f (delta=%.3f)",
             pose_msg->pose.position.x,
             pose_msg->pose.position.y,
             moved);
  }
}

/**
 * @brief Stores the latest twist so each waypoint row can include a speed estimate.
 */
void TwistCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg) {
  g_recorder.UpdateTwist(*twist_msg);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_publisher_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string pose_topic = "/Unity_ROS_message_Rx/OurCar/CoM/pose";
  std::string twist_topic = "/Unity_ROS_message_Rx/OurCar/CoM/twist";
  std::string csv_path = "src/path_recorder/recorded_path.csv";

  pnh.param<std::string>("pose_topic", pose_topic, pose_topic);
  pnh.param<std::string>("twist_topic", twist_topic, twist_topic);
  pnh.param<std::string>("csv_path", csv_path, csv_path);
  pnh.param<double>("record_distance_threshold", g_record_distance_threshold,
                    g_record_distance_threshold);

  if (!g_recorder.Start(csv_path)) {
    return 1;
  }

  ros::Subscriber pose_sub = nh.subscribe(pose_topic, 50, PoseCallback);
  ros::Subscriber twist_sub = nh.subscribe(twist_topic, 50, TwistCallback);

  ROS_INFO("path_publisher_node started. pose_topic=%s twist_topic=%s threshold=%.2f",
           pose_topic.c_str(), twist_topic.c_str(), g_record_distance_threshold);

  ros::spin();
  g_recorder.Stop();
  return 0;
}
