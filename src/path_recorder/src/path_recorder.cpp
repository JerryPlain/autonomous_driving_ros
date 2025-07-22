#include "path_recorder.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <fstream>
#include <cmath>

// 构造函数
PathRecorder::PathRecorder() {
    last_point.x = 0.0;
    last_point.y = 0.0;
    last_point.z = 0.0;
}

// 析构函数
PathRecorder::~PathRecorder() {
    if (logfile.is_open()) {
        logfile.close();
    }
}

// 设置起点位置（如保留也不影响）
void PathRecorder::setStartPoint(const geometry_msgs::Point& point) {
    start_point = point;
}

// 设置当前速度（供记录使用）
void PathRecorder::setTwist(const geometry_msgs::TwistStamped& twist) {
    last_twist = twist;
}

// 启动写入 CSV 文件
void PathRecorder::startRecording(const std::string& filename) {
    logfile.open(filename);
    if (logfile.is_open()) {
        ROS_INFO("Started recording to file: %s", filename.c_str());
        logfile << "x,y,z,vx,vy,vz,turn_type\n";  // 表头
    } else {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
    }
}

// 停止写入 CSV 文件
void PathRecorder::stopRecording() {
    if (logfile.is_open()) {
        logfile.close();
        ROS_INFO("Stopped recording.");
    }
}

// 欧几里得距离
double PathRecorder::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

// 记录路径点（世界坐标）到 CSV 并发布路径
void PathRecorder::recordTurn(const std::string& turn_type, const geometry_msgs::PoseStamped& msg) {
    const auto& current = msg.pose.position;
    const auto& vel = last_twist.twist.linear;

    // ✅ 使用世界坐标（不再减去起点）
    if (logfile.is_open()) {
        logfile << current.x << "," << current.y << "," << current.z << ",";
        logfile << vel.x << "," << vel.y << "," << vel.z << ",";
        logfile << turn_type << "\n";
    }

    // 发布路径点
    geometry_msgs::PoseStamped stamped;
    stamped.header.stamp = ros::Time::now();
    stamped.header.frame_id = "map";  // 统一为世界坐标系，如需改用 "odom" 或 "OurCar/INS" 请自行替换
    stamped.pose = msg.pose;
    stamped.pose.position = current;  // ✅ 直接使用原始位置（世界坐标）

    path_msg.header = stamped.header;
    path_msg.poses.push_back(stamped);

    ROS_INFO("Recorded WORLD point: %.2f, %.2f", current.x, current.y);
}
