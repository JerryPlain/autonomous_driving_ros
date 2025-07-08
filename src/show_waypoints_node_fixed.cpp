#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sstream>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "show_waypoints_node");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_markers", 1, true);

    std::ifstream file("/home/user/桌面/Projectros/introtoros_2025-main-project/project/src/waypoint_logger/waypoints.csv");
    if (!file.is_open()) {
        ROS_ERROR("无法打开 waypoints.csv 文件！");
        return 1;
    }

    visualization_msgs::MarkerArray marker_array;
    std::string line;
    int id = 0;

    // 跳过表头
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> tokens;

        while (std::getline(ss, item, ',')) {
            tokens.push_back(item);
        }

        if (tokens.size() < 7) {
            ROS_WARN("跳过格式错误的行");
            continue;
        }

        try {
            float x = std::stof(tokens[0]);
            float y = std::stof(tokens[1]);
            float z = 0.0;  // 修改为 Z=0.0
            std::string turn_type = tokens[6];  // LEFT_TURN / RIGHT_TURN

            visualization_msgs::Marker marker;
            marker.header.frame_id = "OurCar/INS";  // 确保与你 RViz 的 Fixed Frame 一致
            marker.header.stamp = ros::Time::now();
            marker.ns = "waypoints";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;  // 修改后的 Z 值
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            if (turn_type == "LEFT_TURN") {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            } else if (turn_type == "RIGHT_TURN") {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
            }

            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);

        } catch (const std::exception& e) {
            ROS_WARN("跳过无法解析的一行: %s", e.what());
            continue;
        }
    }

    file.close();

    ROS_INFO("Loaded %lu 个 waypoints marker", marker_array.markers.size());

    marker_pub.publish(marker_array);

    ros::spin();
    return 0;
}

