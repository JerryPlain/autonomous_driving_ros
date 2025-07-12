#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_path_node");
    ros::NodeHandle nh;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/recorded_path", 1, true);

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map"; // 需要和你的仿真 frame 对齐

    std::ifstream csv_file("recorded_path.csv");
    if (!csv_file.is_open())
    {
        ROS_ERROR("无法打开 CSV 文件: recorded_path.csv");
        return -1;
    }

    std::string line;
    std::getline(csv_file, line); // 跳过表头

    while (std::getline(csv_file, line))
    {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ','))
        {
            tokens.push_back(token);
        }

        if (tokens.size() < 6)
            continue;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x = std::stod(tokens[1]);
        pose.pose.position.y = std::stod(tokens[2]);
        pose.pose.position.z = std::stod(tokens[3]);

        tf::Quaternion q;
        q.setRPY(0, 0, std::stod(tokens[4]));
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        path_msg.poses.push_back(pose);
    }

    csv_file.close();
    ROS_INFO("加载完成，共 %lu 个路径点", path_msg.poses.size());

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        path_msg.header.stamp = ros::Time::now();
        path_pub.publish(path_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
