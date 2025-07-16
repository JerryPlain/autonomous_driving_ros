#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "path_recorder.hpp"

PathRecorder recorder;

geometry_msgs::PoseStamped last_pose;
bool has_last_pose = false;

// 每次 pose 回调
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!has_last_pose) {
        last_pose = *msg;
        recorder.setStartPoint(msg->pose.position);
        has_last_pose = true;
        return;
    }

    double dist = recorder.calculateDistance(msg->pose.position, last_pose.pose.position);

    if (dist >= 10.0) {
        recorder.recordTurn("FORWARD", *msg);
        ROS_INFO("Recorded point at (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);
        last_pose = *msg;
    }
}

// 速度信息更新（保存最新速度供记录用）
void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    recorder.setTwist(*msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_publisher_node");
    ros::NodeHandle nh;

    recorder.startRecording("/home/user/waypoints.csv");

    ros::Subscriber pose_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 10, poseCallback);
    ros::Subscriber twist_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 10, twistCallback);

    ros::spin();

    recorder.stopRecording();
    return 0;
}

