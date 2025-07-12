#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_goals_sender");

    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up...");
    }

    // 读取 waypoints.yaml 文件
    std::string yaml_file;
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("waypoints_file", yaml_file, "/home/user/maps/waypoints.yaml");

    YAML::Node yaml_node = YAML::LoadFile(yaml_file);

    if (!yaml_node.IsSequence())
    {
        ROS_ERROR("Invalid YAML file format. Expected a sequence of waypoints.");
        return 1;
    }

    for (const auto &point : yaml_node)
    {
        if (!point["x"] || !point["y"] || !point["w"])
        {
            ROS_ERROR("Invalid waypoint format. Each point needs 'x', 'y', 'w'.");
            continue;
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = point["x"].as<double>();
        goal.target_pose.pose.position.y = point["y"].as<double>();
        goal.target_pose.pose.orientation.w = point["w"].as<double>();

        ROS_INFO("Sending goal: x = %.2f, y = %.2f, w = %.2f",
                 point["x"].as<double>(),
                 point["y"].as<double>(),
                 point["w"].as<double>());

        ac.sendGoal(goal);
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Arrived at goal!");
        else
            ROS_WARN("Failed to reach goal.");
    }

    ROS_INFO("All waypoints completed!");

    return 0;
}

