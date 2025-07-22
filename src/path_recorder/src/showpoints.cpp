#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <sstream>
#include <vector>

// CSV 文件路径（根据你自己的路径修改）
std::string csv_file_path = "/home/user/桌面/Projectros/introtoros_2025-main-project/project/src/path_recorder/recorded_path.csv";

// 从 CSV 文件中读取点
std::vector<geometry_msgs::Point> readPointsFromCSV(const std::string& filepath)
{
    std::vector<geometry_msgs::Point> points;
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", filepath.c_str());
        return points;
    }

    std::string line;
    getline(file, line);  // 跳过表头

    while (getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;
        geometry_msgs::Point p;

        // time
        getline(ss, token, ',');

        // x
        getline(ss, token, ',');
        p.x = std::stod(token);

        // y
        getline(ss, token, ',');
        p.y = std::stod(token);

        // v
        getline(ss, token, ',');

        p.z = 0.0;

        points.push_back(p);
    }
    file.close();
    ROS_INFO("Read %ld points from CSV file.", points.size());
    return points;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "showpoints");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    std::vector<geometry_msgs::Point> points = readPointsFromCSV(csv_file_path);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "recorded_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.points = points;

    ros::Rate rate(1);
    while (ros::ok())
    {
        marker.header.stamp = ros::Time::now();
        marker_pub.publish(marker);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

