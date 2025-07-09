#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <cmath>
#include <string>

// 定义起点和上一个路径点
geometry_msgs::Point start_point;  // 记录小车静止时的起点位置
geometry_msgs::Point last_point;
geometry_msgs::TwistStamped last_twist;

std::ofstream logfile;
ros::Publisher path_pub;
nav_msgs::Path path_msg;

// 距离阈值（每10米记录一次）
const double DISTANCE_THRESHOLD = 10.0;  // 每 10 米记录一个路径点

// 计算两点之间的欧几里得距离
double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

// 记录路径点并发布
void recordTurn(const std::string& turn_type, const geometry_msgs::PoseStamped& msg) {
    const auto& current = msg.pose.position;
    const auto& vel = last_twist.twist.linear;

    // 计算相对小车起点的坐标
    geometry_msgs::Point relative_position;
    relative_position.x = current.x - start_point.x;
    relative_position.y = current.y - start_point.y;
    relative_position.z = 0.0;  // 固定为 0.0，确保在地面上

    // 写入 CSV 文件
    logfile << relative_position.x << "," << relative_position.y << "," << relative_position.z << ",";
    logfile << vel.x << "," << vel.y << "," << vel.z << ",";
    logfile << turn_type << "\n";

    // 打印到终端
    ROS_INFO("Recorded %s at (%.2f, %.2f) with speed (%.2f, %.2f)",
             turn_type.c_str(), relative_position.x, relative_position.y, vel.x, vel.y);

    // 发布路径点到 RViz
    geometry_msgs::PoseStamped stamped;
    stamped.header.stamp = ros::Time::now();
    stamped.header.frame_id = "OurCar/INS";  // 使用小车坐标系
    stamped.pose = msg.pose;

    // 发布相对小车起点的坐标
    stamped.pose.position.x = relative_position.x;
    stamped.pose.position.y = relative_position.y;
    stamped.pose.position.z = 0.0;  // 确保路径点的 Z 坐标是 0.0

    path_msg.header = stamped.header;
    path_msg.poses.push_back(stamped);
    path_pub.publish(path_msg);
}

// 接收速度信息
void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    last_twist = *msg;
}

// 接收位置并判断是否需要记录
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::Point current = msg->pose.position;

    // 第一次接收到小车位置时，记录起点
    if (last_point.x == 0.0 && last_point.y == 0.0) {
        start_point = current;  // 记录小车的起点
        last_point = current;
        return;
    }

    // 计算当前位置与上一个记录点的距离
    double dist = calculateDistance(current, last_point);

    // 如果移动了 10 米以上，记录当前路径点
    if (dist >= DISTANCE_THRESHOLD) {
        std::string turn_type = "STRAIGHT";
        recordTurn(turn_type, *msg);  // 如果距离大于 10 米，记录
        last_point = current;  // 更新上一个记录点
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_logger_node");
    ros::NodeHandle nh;

    // ⚠️ 替换为你真实用户名路径
    logfile.open("/home/user/桌面/Projectros/introtoros_2025-main-project/project/src/path_recorder/waypoints.csv");

    ros::Subscriber pose_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10, poseCallback);
    ros::Subscriber twist_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 10, twistCallback);
    path_pub = nh.advertise<nav_msgs::Path>("logged_path", 10, true);

    ros::spin();
    logfile.close();
    return 0;
}

