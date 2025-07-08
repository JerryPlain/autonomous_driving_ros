#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <cmath>

geometry_msgs::Point last_point;
double last_heading = 0.0;
bool first_point = true;
geometry_msgs::TwistStamped last_twist;

std::ofstream logfile;
ros::Publisher path_pub;
nav_msgs::Path path_msg;

// 冷却机制
ros::Time last_turn_time;
const double TURN_COOLDOWN = 2.0;  // 单位：秒

// 将角度标准化到 [-π, π]
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// 保存拐弯点并发布到路径
void recordTurn(const std::string& turn_type, const geometry_msgs::PoseStamped& msg) {
    const auto& current = msg.pose.position;
    const auto& vel = last_twist.twist.linear;

    // 写入 CSV
    logfile << current.x << "," << current.y << "," << current.z << ",";
    logfile << vel.x << "," << vel.y << "," << vel.z << ",";
    logfile << turn_type << "\n";

    // 打印到终端
    ROS_INFO("Recorded %s at (%.2f, %.2f) with speed (%.2f, %.2f)",
             turn_type.c_str(), current.x, current.y, vel.x, vel.y);

    // 发布路径点到 RViz
    geometry_msgs::PoseStamped stamped;
    stamped.header.stamp = ros::Time::now();
    stamped.header.frame_id = "map";  // 与 RViz 中 Fixed Frame 一致
    stamped.pose = msg.pose;

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

    if (first_point) {
        last_point = current;
        last_heading = 0.0;
        first_point = false;
        logfile << "x,y,z,vx,vy,vz,turn_type\n";
        last_turn_time = ros::Time::now() - ros::Duration(TURN_COOLDOWN);  // 允许立即记录第一次
        return;
    }

    double dx = current.x - last_point.x;
    double dy = current.y - last_point.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 0.1) return;  // 忽略极小移动

    double current_heading = atan2(dy, dx);
    double delta = normalize_angle(current_heading - last_heading);
    ros::Time now = ros::Time::now();

    if (std::fabs(delta) > 0.35 && (now - last_turn_time).toSec() > TURN_COOLDOWN) {
        std::string turn_type = (delta > 0) ? "LEFT_TURN" : "RIGHT_TURN";
        recordTurn(turn_type, *msg);
        last_turn_time = now;
        last_heading = current_heading;
        last_point = current;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_logger_node");
    ros::NodeHandle nh;

    // ⚠️ 替换为你真实用户名路径
    logfile.open("/home/user/桌面/Projectros/introtoros_2025-main-project/project/src/waypoint_logger/waypoints.csv");

    ros::Subscriber pose_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10, poseCallback);
    ros::Subscriber twist_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 10, twistCallback);
    path_pub = nh.advertise<nav_msgs::Path>("logged_path", 10, true);

    ros::spin();
    logfile.close();
    return 0;
}

