#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <vector>

struct Waypoint {
    int index;
    double x;
    double y;
    double theta;
};

std::vector<Waypoint> readWaypointsFromCSV(const std::string& filepath)
{
    std::vector<Waypoint> waypoints;
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        ROS_ERROR("无法打开 CSV 文件: %s", filepath.c_str());
        return waypoints;
    }

    std::string line;
    getline(file, line);  // 跳过表头

    int generated_index = 0;

    while (getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;
        Waypoint wp;

        // 跳过时间列
        getline(ss, token, ',');

        // x
        getline(ss, token, ',');
        wp.x = std::stod(token);

        // y
        getline(ss, token, ',');
        wp.y = std::stod(token);

        // v
        getline(ss, token, ',');

        wp.theta = 0.0;

        // 自动生成 index
        wp.index = generated_index;
        generated_index++;

        waypoints.push_back(wp);
    }
    file.close();
    ROS_INFO("已从 CSV 文件读取 %lu 个目标点", waypoints.size());
    return waypoints;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_goal_publisher");
    ros::NodeHandle nh;

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    // 修改为你的实际 CSV 文件路径
    std::string csv_path = "/home/user/桌面/Projectros/introtoros_2025-main-project/project/src/path_recorder/recorded_path.csv";
    std::vector<Waypoint> waypoints = readWaypointsFromCSV(csv_path);

    ros::Duration(2.0).sleep();  // 等待 publisher 初始化

    for (const auto& wp : waypoints)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.header.seq = wp.index;  // ✅ 把 index 放到 header.seq 中
        goal.pose.position.x = wp.x;
        goal.pose.position.y = wp.y;
        goal.pose.orientation.w = 1.0;  // 默认无旋转

        ROS_INFO("Publishing goal: index = %d, x = %.2f, y = %.2f", wp.index, wp.x, wp.y);
        goal_pub.publish(goal);

        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    ROS_INFO("已发布所有目标点，任务完成。");

    return 0;
}
