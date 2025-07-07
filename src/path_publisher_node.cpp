#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sstream>
#include <vector>

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_publisher_node");
    ros::NodeHandle nh;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/planned_path", 10, true);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/turn_markers", 10, true);

    std::ifstream file("/home/user/桌面/Projectros/introtoros_2025-main-project/project/src/waypoint_logger/waypoints.csv");
    if (!file.is_open()) {
        ROS_ERROR("无法打开waypoints.csv文件！");
        return 1;
    }

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "OurCar/INS";  // 或 map，视你 RViz 配置

    visualization_msgs::MarkerArray marker_array;

    std::string line;
    std::getline(file, line);  // 跳过表头

    geometry_msgs::Point prev_point;
    bool first = true;
    int id = 0;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> tokens;

        while (std::getline(ss, item, ',')) {
            tokens.push_back(item);
        }

        if (tokens.size() < 3) continue;

        float x = std::stof(tokens[0]);
        float y = std::stof(tokens[1]);
        float z = std::stof(tokens[2]);

        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0;  // 简单处理

        path_msg.poses.push_back(pose);

        if (!first) {
            float dx = x - prev_point.x;
            float dy = y - prev_point.y;
            float heading = atan2(dy, dx);

            visualization_msgs::Marker arrow;
            arrow.header = path_msg.header;
            arrow.ns = "turn_marker";
            arrow.id = id++;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;
            arrow.pose.position.x = x;
            arrow.pose.position.y = y;
            arrow.pose.position.z = z;

            arrow.pose.orientation.z = sin(heading / 2.0);
            arrow.pose.orientation.w = cos(heading / 2.0);

            arrow.scale.x = 1.0;
            arrow.scale.y = 0.2;
            arrow.scale.z = 0.2;

            float angle_diff = atan2(dy, dx) - atan2(prev_point.y, prev_point.x);
            std_msgs::ColorRGBA color;
            color.a = 1.0;

            if (angle_diff > 0.2) {
                color.r = 1.0; color.g = 0.0; color.b = 0.0;  // 红色右转
            } else if (angle_diff < -0.2) {
                color.r = 0.0; color.g = 1.0; color.b = 0.0;  // 绿色左转
            } else {
                color.r = 0.0; color.g = 0.0; color.b = 1.0;  // 蓝色直行
            }

            arrow.color = color;
            marker_array.markers.push_back(arrow);
        }

        prev_point.x = x;
        prev_point.y = y;
        prev_point.z = z;
        first = false;
    }

    // 发布一次
    ros::Rate rate(1);
    while (ros::ok()) {
        path_pub.publish(path_msg);
        marker_pub.publish(marker_array);
        rate.sleep();
    }

    return 0;
}

