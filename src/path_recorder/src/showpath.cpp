#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {

/**
 * @brief Loads poses from the canonical path CSV file.
 */
std::vector<geometry_msgs::PoseStamped> LoadPoses(const std::string& csv_path,
                                                  const std::string& frame_id) {
  std::ifstream file(csv_path.c_str());
  if (!file.is_open()) {
    ROS_ERROR("Unable to open CSV file: %s", csv_path.c_str());
    return {};
  }

  std::string line;
  std::getline(file, line);  // Header.

  std::vector<geometry_msgs::PoseStamped> poses;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> cols;
    while (std::getline(ss, token, ',')) {
      cols.push_back(token);
    }

    if (cols.size() < 6) {
      continue;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = std::stod(cols[2]);
    pose.pose.position.y = std::stod(cols[3]);
    pose.pose.position.z = std::stod(cols[4]);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::stod(cols[5]));
    pose.pose.orientation = tf2::toMsg(q);

    poses.push_back(pose);
  }

  return poses;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "show_path_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string csv_path = "src/path_recorder/recorded_path.csv";
  std::string frame_id = "map";
  std::string topic = "/recorded_path";
  double publish_hz = 1.0;

  pnh.param<std::string>("csv_path", csv_path, csv_path);
  pnh.param<std::string>("frame_id", frame_id, frame_id);
  pnh.param<std::string>("topic", topic, topic);
  pnh.param<double>("publish_hz", publish_hz, publish_hz);

  std::vector<geometry_msgs::PoseStamped> poses = LoadPoses(csv_path, frame_id);
  if (poses.empty()) {
    ROS_ERROR("No valid poses loaded from %s", csv_path.c_str());
    return 1;
  }

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>(topic, 1, true);

  nav_msgs::Path path_msg;
  path_msg.header.frame_id = frame_id;
  path_msg.poses = poses;

  ROS_INFO("Loaded %zu poses for visualization on %s", poses.size(), topic.c_str());

  ros::Rate rate(publish_hz);
  while (ros::ok()) {
    path_msg.header.stamp = ros::Time::now();
    for (auto& pose : path_msg.poses) {
      pose.header.stamp = path_msg.header.stamp;
    }

    path_pub.publish(path_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
