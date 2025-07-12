#include <ros/ros.h>
#include "a_star.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "a_star_path_planner");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("planned_path", 1);

    // 简单测试地图
    std::vector<std::vector<int>> grid(20, std::vector<int>(40, 0));
    for (int y = 8; y < 12; ++y)
        grid[y][20] = 1;

    // A* 路径规划
    AStarPlanner planner(grid);
    auto path = planner.plan(2, 8, 33, 15);  // 起点→终点

    // 转换为 ROS Path 消息
    nav_msgs::Path ros_path;
    ros_path.header.frame_id = "map";
    for (auto& [x, y] : path) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0;
        ros_path.poses.push_back(pose);
    }

    // 发布路径
    ros::Rate rate(1.0);
    while (ros::ok()) {
        ros_path.header.stamp = ros::Time::now();
        path_pub.publish(ros_path);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

