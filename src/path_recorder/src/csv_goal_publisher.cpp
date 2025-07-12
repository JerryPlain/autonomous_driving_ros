#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <vector>

// 定义轨迹点结构体
struct Waypoint {
    double x;
    double y;
    double theta;
};

// 从 CSV 文件中读取所有轨迹点
std::vector<Waypoint> readWaypointsFromCSV(const std::string& filepath)
{
    std::vector<Waypoint> waypoints;
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open CSV file: %s", filepath.c_str());
        return waypoints;
    }

    std::string line;
    getline(file, line);  // 跳过表头

    while (getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;
        Waypoint wp;

        // time
        getline(ss, token, ',');

        // x
        getline(ss, token, ',');
        wp.x = std::stod(token);

        // y
        getline(ss, token, ',');
        wp.y = std::stod(token);

        // v
        getline(ss, token, ',');

        // 简单写死 theta，如果需要可自行计算方向
        wp.theta = 0.0;

        waypoints.push_back(wp);
    }
    file.close();
    ROS_INFO("Read %lu waypoints from CSV.", waypoints.size());
    return waypoints;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_goal_publisher");
    ros::NodeHandle nh;

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    // 修改成你自己的 CSV 文件完整路径
    std::string csv_path = "/home/user/桌面/Projectros/introtoros_2025-main-project/project/src/path_recorder/recorded_path.csv";
    std::vector<Waypoint> waypoints = readWaypointsFromCSV(csv_path);

    ros::Duration(2.0).sleep();  // 等待 publisher 初始化

    for (const auto& wp : waypoints)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = wp.x;
        goal.pose.position.y = wp.y;
        goal.pose.orientation.w = 1.0;  // 简化写法，默认正向

        ROS_INFO("Publishing goal: x = %.2f, y = %.2f", wp.x, wp.y);
        goal_pub.publish(goal);

        // 等待一段时间后再发下一个目标，可根据需要修改（比如监听是否到达）
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    ROS_INFO("All goals published. Task completed.");

    return 0;
}

