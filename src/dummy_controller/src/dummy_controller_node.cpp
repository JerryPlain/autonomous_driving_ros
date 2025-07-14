#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <simulation/VehicleControl.h>
#include <tf/tf.h>
#include <cmath>

// 当前目标点
geometry_msgs::PoseStamped current_goal;
bool has_goal = false;

// 当前车辆位置
geometry_msgs::Pose current_pose;
bool has_pose = false;

// 当前车辆速度
double current_speed = 0.0;
bool has_speed = false;

// 目标点回调
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_goal = *msg;
    has_goal = true;
    ROS_INFO("Received goal: x=%.2f, y=%.2f", msg->pose.position.x, msg->pose.position.y);
}

// 当前位姿回调
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = msg->pose;
    has_pose = true;
}

// 速度回调
void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_speed = msg->twist.linear.x;  // 假设 x 方向是车辆前向
    has_speed = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_controller_node");
    ros::NodeHandle nh;

    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalCallback);
    ros::Subscriber pose_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1, poseCallback);
    ros::Subscriber speed_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 1, speedCallback);
    ros::Publisher pub = nh.advertise<simulation::VehicleControl>("car_command", 1);

    constexpr float loop_interval = 0.05;
    ros::Rate loop_rate(1 / loop_interval);

    // 距离 PID 参数
    double kp_dist = 1.0;
    double ki_dist = 0.0;
    double kd_dist = 0.1;
    double prev_error_dist = 0.0;
    double integral_dist = 0.0;

    // Yaw PID 参数
    double kp_yaw = 5.0;
    double ki_yaw = 0.0;
    double kd_yaw = 0.1;
    double prev_error_yaw = 0.0;
    double integral_yaw = 0.0;

    while (ros::ok())
    {
        ros::spinOnce();

        simulation::VehicleControl msg;

        if (has_goal && has_pose)
        {
            double current_x = current_pose.position.x;
            double current_y = current_pose.position.y;

            double error_x = current_goal.pose.position.x - current_x;
            double error_y = current_goal.pose.position.y - current_y;
            double distance_error = sqrt(error_x * error_x + error_y * error_y);

            double target_yaw = atan2(error_y, error_x);

            // 提取当前朝向 yaw
            tf::Quaternion q(
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            double yaw_error = target_yaw - yaw;

            // 保证 yaw 误差在 [-pi, pi]
            while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

            // PID 控制计算油门
            integral_dist += distance_error * loop_interval;
            double derivative_dist = (distance_error - prev_error_dist) / loop_interval;
            double throttle_cmd = kp_dist * distance_error + ki_dist * integral_dist + kd_dist * derivative_dist;
            prev_error_dist = distance_error;
            
            

            // 限制油门范围
            if (throttle_cmd > 1.0)
                throttle_cmd = 1.0;
            if (throttle_cmd < 0.0)
                throttle_cmd = 0.0;

            // PID 控制计算 steering
            integral_yaw += yaw_error * loop_interval;
            double derivative_yaw = (yaw_error - prev_error_yaw) / loop_interval;
            double steering_cmd = -(kp_yaw * yaw_error + ki_yaw * integral_yaw + kd_yaw * derivative_yaw);//注意此处负号是与yawerror的正负相反
            prev_error_yaw = yaw_error;

            // 限制转角范围
            if (steering_cmd > 1.0) steering_cmd = 1.0;
            if (steering_cmd < -1.0) steering_cmd = -1.0;

            msg.Throttle = throttle_cmd;
            msg.Steering = steering_cmd;

            if (distance_error < 0.5)
            {
                msg.Throttle = 0.0;
                msg.Brake = 0.0;
                ROS_INFO("\033[31mReached goal!\033[0m");
            }
            else
            {
                msg.Brake = 0.0;
            }

            if (has_speed)
            {
                ROS_INFO("x:%.2f ,y:%.2f,Speed: %.2f m/s, Throttle: %.2f, Steering: %.2f, Yaw: %.2f, targetyaw:%.2f",
                    current_x, current_y, current_speed, msg.Throttle, msg.Steering, yaw, target_yaw); 
            }
        }
        else
        {
            msg.Throttle = 0.0;
            msg.Steering = 0.0;
            msg.Brake = 1.0;
        }

        pub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}