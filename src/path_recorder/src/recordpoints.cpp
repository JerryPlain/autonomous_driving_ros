#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace {

/**
 * @brief Loads waypoints from CSV and converts each one into a static TF frame.
 */
std::vector<geometry_msgs::TransformStamped> LoadWaypointTransforms(
    const std::string& csv_path,
    const std::string& parent_frame,
    const std::string& child_prefix) {
  std::ifstream file(csv_path.c_str());
  if (!file.is_open()) {
    ROS_ERROR("Unable to open CSV file: %s", csv_path.c_str());
    return {};
  }

  std::vector<geometry_msgs::TransformStamped> transforms;
  std::string line;
  std::getline(file, line);  // Header.

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

    if (cols.size() < 5) {
      continue;
    }

    const std::string index = cols[0];
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = parent_frame;
    tf.child_frame_id = child_prefix + index;
    tf.transform.translation.x = std::stod(cols[2]);
    tf.transform.translation.y = std::stod(cols[3]);
    tf.transform.translation.z = std::stod(cols[4]);
    tf.transform.rotation.w = 1.0;

    transforms.push_back(tf);
  }

  return transforms;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "csv_tf_publisher");
  ros::NodeHandle pnh("~");

  std::string csv_path = "src/path_recorder/recorded_path.csv";
  std::string parent_frame = "map";
  std::string child_prefix = "recorded_wp_";

  pnh.param<std::string>("csv_path", csv_path, csv_path);
  pnh.param<std::string>("parent_frame", parent_frame, parent_frame);
  pnh.param<std::string>("child_prefix", child_prefix, child_prefix);

  std::vector<geometry_msgs::TransformStamped> transforms =
      LoadWaypointTransforms(csv_path, parent_frame, child_prefix);
  if (transforms.empty()) {
    ROS_ERROR("No transforms generated from CSV: %s", csv_path.c_str());
    return 1;
  }

  const ros::Time now = ros::Time::now();
  for (auto& tf : transforms) {
    tf.header.stamp = now;
  }

  tf2_ros::StaticTransformBroadcaster broadcaster;
  broadcaster.sendTransform(transforms);

  ROS_INFO("Published %zu static waypoint transforms", transforms.size());
  ros::spin();
  return 0;
}
