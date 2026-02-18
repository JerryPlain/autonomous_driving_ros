#include <cstddef>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {

struct Waypoint {
  std::size_t index = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

std::vector<Waypoint> g_waypoints;
std::size_t g_next_goal = 0;
bool g_goal_pending = false;

/**
 * @brief Parses recorder CSV rows with schema: index,timestamp,x,y,z,yaw,velocity.
 */
std::vector<Waypoint> LoadWaypoints(const std::string& csv_path) {
  std::ifstream file(csv_path.c_str());
  if (!file.is_open()) {
    ROS_ERROR("Failed to open waypoint CSV: %s", csv_path.c_str());
    return {};
  }

  std::string line;
  std::getline(file, line);  // Skip header.

  std::vector<Waypoint> result;
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
      ROS_WARN("Skipping malformed waypoint row: %s", line.c_str());
      continue;
    }

    Waypoint wp;
    try {
      wp.index = static_cast<std::size_t>(std::stoll(cols[0]));
      wp.x = std::stod(cols[2]);
      wp.y = std::stod(cols[3]);
      wp.z = std::stod(cols[4]);
      wp.yaw = std::stod(cols[5]);
      result.push_back(wp);
    } catch (const std::exception& e) {
      ROS_WARN("Skipping waypoint row due to parse error: %s", e.what());
    }
  }

  ROS_INFO("Loaded %zu waypoints from %s", result.size(), csv_path.c_str());
  return result;
}

/**
 * @brief Unblocks next goal publication once controller reports the reached index.
 */
void ReachedCallback(const std_msgs::Int32::ConstPtr& msg) {
  if (g_next_goal == 0 || g_next_goal > g_waypoints.size()) {
    return;
  }

  const std::size_t expected_index = g_waypoints[g_next_goal - 1].index;
  if (static_cast<std::size_t>(msg->data) != expected_index) {
    ROS_WARN("Received reached_goal=%d but expected=%zu", msg->data, expected_index);
    return;
  }

  g_goal_pending = false;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "csv_goal_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string csv_path = "src/path_recorder/recorded_path.csv";
  std::string goal_topic = "/move_base_simple/goal";
  std::string reached_topic = "/reached_goal";
  std::string frame_id = "map";
  double loop_hz = 10.0;

  pnh.param<std::string>("csv_path", csv_path, csv_path);
  pnh.param<std::string>("goal_topic", goal_topic, goal_topic);
  pnh.param<std::string>("reached_topic", reached_topic, reached_topic);
  pnh.param<std::string>("frame_id", frame_id, frame_id);
  pnh.param<double>("loop_hz", loop_hz, loop_hz);

  g_waypoints = LoadWaypoints(csv_path);
  if (g_waypoints.empty()) {
    return 1;
  }

  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
  ros::Subscriber reached_sub = nh.subscribe(reached_topic, 10, ReachedCallback);

  ros::Rate rate(loop_hz);
  while (ros::ok() && goal_pub.getNumSubscribers() == 0) {
    ROS_INFO_THROTTLE(1.0, "Waiting for subscribers on %s", goal_topic.c_str());
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok() && g_next_goal < g_waypoints.size()) {
    ros::spinOnce();

    if (!g_goal_pending) {
      const Waypoint& wp = g_waypoints[g_next_goal];

      geometry_msgs::PoseStamped goal;
      goal.header.stamp = ros::Time::now();
      goal.header.frame_id = frame_id;
      goal.header.seq = static_cast<uint32_t>(wp.index);
      goal.pose.position.x = wp.x;
      goal.pose.position.y = wp.y;
      goal.pose.position.z = wp.z;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, wp.yaw);
      goal.pose.orientation = tf2::toMsg(q);

      goal_pub.publish(goal);
      g_goal_pending = true;
      ++g_next_goal;

      ROS_INFO("Published goal idx=%u x=%.3f y=%.3f yaw=%.3f",
               goal.header.seq,
               goal.pose.position.x,
               goal.pose.position.y,
               wp.yaw);
    }

    rate.sleep();
  }

  ROS_INFO("csv_goal_publisher finished. All waypoints sent.");
  return 0;
}
