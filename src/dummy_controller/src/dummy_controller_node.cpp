#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <simulation/VehicleControl.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <cmath>

class PIDControllerNode
{
public:
    PIDControllerNode() : 
        nh_(), 
        loop_interval_(0.05), 
        has_goal_(false), 
        has_pose_(false), 
        has_speed_(false), 
        has_decision_(false),
        current_speed_(0.0), 
        current_goal_index_(-1),
        goal_already_reported_(false),
        final_goal_reached_(false),
        max_goal_index_(1042),
        kp_dist_(0.25), ki_dist_(0.0), kd_dist_(0.1),
        kp_yaw_(7.5), ki_yaw_(0.0), kd_yaw_(0.1),
        prev_error_dist_(0.0), integral_dist_(0.0),
        prev_error_yaw_(0.0), integral_yaw_(0.0)
    {
        // Subscribers
        goal_sub_     = nh_.subscribe("/move_base_simple/goal", 1, &PIDControllerNode::goalCallback, this);
        pose_sub_     = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1, &PIDControllerNode::poseCallback, this);
        speed_sub_    = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 1, &PIDControllerNode::speedCallback, this);
        decision_sub_ = nh_.subscribe("traffic_decision", 1, &PIDControllerNode::trafficDecisionCallback, this);

        // Publishers
        control_pub_ = nh_.advertise<simulation::VehicleControl>("car_command", 1);
        reached_pub_ = nh_.advertise<std_msgs::Int32>("/reached_goal", 1);
    }

    void spin()
    {
        ros::Rate rate(1 / loop_interval_);
        while (ros::ok())
        {
            ros::spinOnce();
            processControl();
            rate.sleep();
        }
    }

private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_goal_ = *msg;
        current_goal_index_ = msg->header.seq;
        goal_already_reported_ = false;
        has_goal_ = true;

        ROS_INFO("Received goal: index = %d, x = %.2f, y = %.2f",
                 current_goal_index_, msg->pose.position.x, msg->pose.position.y);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pose_ = msg->pose;
        has_pose_ = true;
    }

    void speedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
        has_speed_ = true;
    }

    void trafficDecisionCallback(const std_msgs::String::ConstPtr& msg)
    {
        traffic_decision_ = msg->data;
        has_decision_ = true;
        ROS_INFO("Received traffic decision: %s", traffic_decision_.c_str());
    }

    void processControl()
    {
        simulation::VehicleControl msg;

        if (final_goal_reached_)
        {
            // Stop vehicle after final goal
            msg.Throttle = 0.0;
            msg.Steering = 0.0;
            msg.Brake = 1.0;
            ROS_INFO_THROTTLE(1.0, "\033[41mFinal goal reached. Car is stopped.\033[0m");
        }
        else if (has_goal_ && has_pose_)
        {
            double cx = current_pose_.position.x;
            double cy = current_pose_.position.y;
            double gx = current_goal_.pose.position.x;
            double gy = current_goal_.pose.position.y;

            double dx = gx - cx;
            double dy = gy - cy;
            double dist_err = std::sqrt(dx * dx + dy * dy);

            double target_yaw = std::atan2(dy, dx);

            tf::Quaternion q(current_pose_.orientation.x,
                             current_pose_.orientation.y,
                             current_pose_.orientation.z,
                             current_pose_.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            double yaw_err = target_yaw - yaw;
            while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

            // PID throttle
            integral_dist_ += dist_err * loop_interval_;
            double d_dist = (dist_err - prev_error_dist_) / loop_interval_;
            double throttle = kp_dist_ * dist_err + ki_dist_ * integral_dist_ + kd_dist_ * d_dist;
            prev_error_dist_ = dist_err;

            if (throttle > 1.0) throttle = 1.0;
            if (throttle < 0.0) throttle = 0.0;

            // PID steering
            integral_yaw_ += yaw_err * loop_interval_;
            double d_yaw = (yaw_err - prev_error_yaw_) / loop_interval_;
            double steering = -(kp_yaw_ * yaw_err + ki_yaw_ * integral_yaw_ + kd_yaw_ * d_yaw);
            prev_error_yaw_ = yaw_err;

            if (steering > 1.0) steering = 1.0;
            if (steering < -1.0) steering = -1.0;

            msg.Throttle = throttle;
            msg.Steering = steering;
            msg.Brake = 0.0;

            // Override for red light
            if (traffic_decision_ == "BRAKE")
            {
                msg.Throttle = 0.0;
                msg.Brake = 1.0;
                msg.Steering = 0.0;
                ROS_WARN("Red light detected: BRAKE!");
            }

            // Check goal reached
            if (dist_err < 0.5)
            {
                msg.Throttle = 0.0;
                msg.Brake = 0.0;
                ROS_INFO("\033[31mReached goal!\033[0m");

                if (!goal_already_reported_ && current_goal_index_ >= 0)
                {
                    std_msgs::Int32 done_msg;
                    done_msg.data = current_goal_index_;
                    reached_pub_.publish(done_msg);
                    goal_already_reported_ = true;

                    if (current_goal_index_ >= max_goal_index_)
                    {
                        final_goal_reached_ = true;
                        ROS_WARN("\033[41mFinal goal reached! Car will brake permanently.\033[0m");
                    }
                }
            }

            if (has_speed_)
            {
                ROS_INFO("x:%.2f, y:%.2f, Speed: %.2f, Throttle: %.2f, Steering: %.2f, Yaw: %.2f, Target_yaw: %.2f",
                         cx, cy, current_speed_, msg.Throttle, msg.Steering, yaw, target_yaw);
            }
        }
        else
        {
            // No valid input: full brake
            msg.Throttle = 0.0;
            msg.Steering = 0.0;
            msg.Brake = 1.0;
        }

        control_pub_.publish(msg);
    }

private:
    ros::NodeHandle nh_;

    // Subscribers and Publishers
    ros::Subscriber goal_sub_, pose_sub_, speed_sub_, decision_sub_;
    ros::Publisher control_pub_, reached_pub_;

    // Control flags and data
    geometry_msgs::PoseStamped current_goal_;
    geometry_msgs::Pose current_pose_;
    std::string traffic_decision_;

    double current_speed_;
    int current_goal_index_;
    bool has_goal_, has_pose_, has_speed_, has_decision_;
    bool goal_already_reported_, final_goal_reached_;

    // Parameters
    int max_goal_index_;
    const float loop_interval_;

    // PID variables
    double kp_dist_, ki_dist_, kd_dist_;
    double kp_yaw_, ki_yaw_, kd_yaw_;
    double prev_error_dist_, integral_dist_;
    double prev_error_yaw_, integral_yaw_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_controller_node");
    PIDControllerNode controller;
    controller.spin();
    return 0;
}
