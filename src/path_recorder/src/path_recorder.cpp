#include "path_recorder.hpp"

#include <cmath>

#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace path_recorder {

PathRecorder::PathRecorder() : next_index_(0) {}

PathRecorder::~PathRecorder() {
  Stop();
}

bool PathRecorder::Start(const std::string& csv_path) {
  csv_file_.open(csv_path.c_str(), std::ios::out | std::ios::trunc);
  if (!csv_file_.is_open()) {
    ROS_ERROR("Failed to open CSV output: %s", csv_path.c_str());
    return false;
  }

  // Keep one canonical CSV schema for every path_recorder tool node.
  csv_file_ << "index,timestamp,x,y,z,yaw,velocity\n";
  ROS_INFO("Recording waypoints to: %s", csv_path.c_str());
  return true;
}

void PathRecorder::Stop() {
  if (csv_file_.is_open()) {
    csv_file_.close();
  }
}

void PathRecorder::UpdateTwist(const geometry_msgs::TwistStamped& twist_msg) {
  last_twist_ = twist_msg;
}

double PathRecorder::PlanarDistance(const geometry_msgs::Point& a,
                                    const geometry_msgs::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool PathRecorder::RecordPose(const geometry_msgs::PoseStamped& pose_msg) {
  if (!csv_file_.is_open()) {
    ROS_ERROR_THROTTLE(2.0, "CSV file is not open; dropping waypoint sample");
    return false;
  }

  // Convert quaternion to yaw for downstream goal/visualization tools.
  tf2::Quaternion q;
  tf2::fromMsg(pose_msg.pose.orientation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  const auto& p = pose_msg.pose.position;
  const auto& v = last_twist_.twist.linear;
  const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

  csv_file_ << next_index_ << ','
            << pose_msg.header.stamp.toSec() << ','
            << p.x << ',' << p.y << ',' << p.z << ','
            << yaw << ',' << speed << '\n';

  ++next_index_;
  return true;
}

}  // namespace path_recorder
