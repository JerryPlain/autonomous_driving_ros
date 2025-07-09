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

// 设置起点位置
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

// 记录路径点（相对起点）到 CSV 并发布路径
void PathRecorder::recordTurn(const std::string& turn_type, const geometry_msgs::PoseStamped& msg) {
    const auto& current = msg.pose.position;
    const auto& vel = last_twist.twist.linear;

    // 相对起点坐标
    geometry_msgs::Point relative_position;
    relative_position.x = current.x - start_point.x;
    relative_position.y = current.y - start_point.y;
    relative_position.z = 0.0;

    if (logfile.is_open()) {
        logfile << relative_position.x << "," << relative_position.y << "," << relative_position.z << ",";
        logfile << vel.x << "," << vel.y << "," << vel.z << ",";
        logfile << turn_type << "\n";
    }

    // 发布路径点
    geometry_msgs::PoseStamped stamped;
    stamped.header.stamp = ros::Time::now();
    stamped.header.frame_id = "OurCar/INS";
    stamped.pose = msg.pose;
    stamped.pose.position = relative_position;  // 改为相对坐标
    stamped.pose.position.z = 0.0;

    path_msg.header = stamped.header;
    path_msg.poses.push_back(stamped);

    ROS_INFO("Recorded: %.2f, %.2f", relative_position.x, relative_position.y);
}

