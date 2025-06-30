#include <ros/ros.h>
#include <simulation/VehicleControl.h>
#include <std_msgs/Bool.h>
#include <cmath>

class VehicleController
{
public:
    VehicleController()
        : nh_(), loop_interval_(0.05), elapsed_time_(0.0), is_red_light_(false)
    {
        pub_ = nh_.advertise<simulation::VehicleControl>("car_command", 1);
        sub_ = nh_.subscribe("/traffic_light", 1, &VehicleController::trafficLightCallback, this);

        timer_ = nh_.createTimer(ros::Duration(loop_interval_), &VehicleController::controlLoop, this);
    }

    void trafficLightCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        is_red_light_ = msg->data;
    }

    void controlLoop(const ros::TimerEvent &)
    {
        simulation::VehicleControl msg;

        if (is_red_light_)
        {
            msg.Throttle = 0.0f;//油门值，范围从 -1 到 1，这是施加到电机上的扭矩
            msg.Brake = 1.0f;// 刹车值，范围从 0 到 1，这将施加刹车扭矩使汽车停止
            ROS_INFO("Red light! Stop: Throttle = %.2f, Brake = %.2f", msg.Throttle, msg.Brake);
        }
        else
        {
            msg.Throttle = 0.5f;
            msg.Brake = 0.0f;
            ROS_INFO("Green light, Arrow: Throttle = %.2f, Brake = %.2f", msg.Throttle, msg.Brake);
        }

        msg.Steering = std::sin(6.28 * elapsed_time_) * 0.5f;
        msg.Reserved = 0.0f;

        pub_.publish(msg);
        elapsed_time_ += loop_interval_;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;
    const float loop_interval_;
    float elapsed_time_;
    bool is_red_light_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_controller_node");

    VehicleController controller;
    ros::spin();

    return 0;
}
