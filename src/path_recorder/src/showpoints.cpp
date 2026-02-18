#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace {

/**
 * @brief Loads xyz points from recorder CSV and returns marker points.
 */
std::vector<geometry_msgs::Point> LoadPoints(const std::string& csv_path) {
  std::ifstream file(csv_path.c_str());
  if (!file.is_open()) {
    ROS_ERROR("Unable to open CSV file: %s", csv_path.c_str());
    return {};
  }

  std::string line;
  std::getline(file, line);  // Header.

  std::vector<geometry_msgs::Point> points;
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

    geometry_msgs::Point p;
    p.x = std::stod(cols[2]);
    p.y = std::stod(cols[3]);
    p.z = std::stod(cols[4]);
    points.push_back(p);
  }

  return points;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "showpoints");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string csv_path = "src/path_recorder/recorded_path.csv";
  std::string frame_id = "map";
  std::string topic = "/recorded_points";
  double publish_hz = 1.0;
  double marker_scale = 0.2;

  pnh.param<std::string>("csv_path", csv_path, csv_path);
  pnh.param<std::string>("frame_id", frame_id, frame_id);
  pnh.param<std::string>("topic", topic, topic);
  pnh.param<double>("publish_hz", publish_hz, publish_hz);
  pnh.param<double>("marker_scale", marker_scale, marker_scale);

  const std::vector<geometry_msgs::Point> points = LoadPoints(csv_path);
  if (points.empty()) {
    ROS_ERROR("No points found in CSV: %s", csv_path.c_str());
    return 1;
  }

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>(topic, 1, true);

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "recorded_points";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = marker_scale;
  marker.scale.y = marker_scale;
  marker.scale.z = marker_scale;
  marker.color.r = 1.0;
  marker.color.g = 0.2;
  marker.color.b = 0.2;
  marker.color.a = 1.0;
  marker.points = points;

  ROS_INFO("Loaded %zu points for marker topic %s", points.size(), topic.c_str());

  ros::Rate rate(publish_hz);
  while (ros::ok()) {
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
