#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "show_path_node");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_markers", 1, true);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("waypoint_path", 1, true);

    std::ifstream file("/home/user/桌面/Projectros/introtoros_2025-main-project/project/src/path_recorder/waypoints.csv");
    if (!file.is_open()) {
        ROS_ERROR("无法打开 waypoints.csv 文件！");
        return 1;
    }

    visualization_msgs::MarkerArray marker_array;
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "OurCar/INS";
    path_msg.header.stamp = ros::Time::now();

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
            float z = 0.0;

            // 创建 Marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = "OurCar/INS";
            marker.header.stamp = ros::Time::now();
            marker.ns = "waypoints";
            marker.id = id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);

            // 添加到 Path
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);

            id++;

        } catch (...) {
            ROS_WARN("跳过无法解析的行");
            continue;
        }
    }

    file.close();

    ros::Duration(1.0).sleep();  // 等待订阅器连接
    marker_pub.publish(marker_array);
    path_pub.publish(path_msg);
    ROS_INFO("已发布 %lu 个 Marker 和 %lu 个 Path 点", marker_array.markers.size(), path_msg.poses.size());

    ros::spin();  // 保持运行
    return 0;
}

