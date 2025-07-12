#include <ros/ros.h>
#include <simulation/VehicleControl.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>

class VehicleController
{
public:
    VehicleController()
        : nh_(), loop_interval_(0.05), elapsed_time_(0.0), traffic_light_state_("UNKNOWN")
    {
        pub_ = nh_.advertise<simulation::VehicleControl>("car_command", 1);
        sub_ = nh_.subscribe("traffic_light_color", 1, &VehicleController::trafficLightCallback, this);
        timer_ = nh_.createTimer(ros::Duration(loop_interval_), &VehicleController::controlLoop, this);
    }

    void trafficLightCallback(const std_msgs::String::ConstPtr &msg)
    {
        // 格式为 "id:state:confidence"
        std::istringstream ss(msg->data);
        std::string id, state, confidence;
        if (std::getline(ss, id, ':') && std::getline(ss, state, ':') && std::getline(ss, confidence)) {
            traffic_light_state_ = state;
        }
    }

    void controlLoop(const ros::TimerEvent &)
    {
        simulation::VehicleControl msg;

        if (traffic_light_state_ == "RED")
        {
            msg.Throttle = 0.0f;
            msg.Brake = 1.0f;
            ROS_INFO("Red light detected! Stop: Throttle = %.2f, Brake = %.2f", msg.Throttle, msg.Brake);
        }
        else
        {
            msg.Throttle = 0.5f;
            msg.Brake = 0.0f;
            ROS_INFO("Green/Yellow/Unknown light: Throttle = %.2f, Brake = %.2f", msg.Throttle, msg.Brake);
        }

        // 添加简单的周期性转向动作
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
    std::string traffic_light_state_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_controller_node");

    VehicleController controller;
    ros::spin();

    return 0;
}