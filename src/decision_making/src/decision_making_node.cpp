#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>
#include <decision_making/TrafficLightColor.h>  // 引入你自定义的消息类型

class TrafficDecisionPublisher
{
public:
    TrafficDecisionPublisher()
        : nh_(), loop_interval_(0.05), traffic_light_state_("UNKNOWN"), distance_(999.0)
    {
        pub_ = nh_.advertise<std_msgs::String>("traffic_decision", 1);
        sub_ = nh_.subscribe("traffic_light_color", 1, &TrafficDecisionPublisher::trafficLightCallback, this);
        timer_ = nh_.createTimer(ros::Duration(loop_interval_), &TrafficDecisionPublisher::decisionLoop, this);
    }

    void trafficLightCallback(const decision_making::TrafficLightColor::ConstPtr &msg)
    {
        traffic_light_state_ = msg->color;
        distance_ = msg->distance;
    }

    void decisionLoop(const ros::TimerEvent &)
    {
        std_msgs::String decision_msg;

        if (distance_ > 20.0) {
            ROS_INFO("Wait traffic_light, current distance: %.2fm...", distance_);
            return;
        }

        if (traffic_light_state_ == "red")
        {
            decision_msg.data = "BRAKE";
            ROS_INFO("Red Light（%.2fm）：BRAKE", distance_);
        }
        else if (traffic_light_state_ == "yellow")
        {
            decision_msg.data = "ACCELERATE";
            ROS_INFO("Yellow Light（%.2fm）：ACCELERATE", distance_);
        }
        else if (traffic_light_state_ == "green")
        {
            decision_msg.data = "DRIVE";
            ROS_INFO("Green Light（%.2fm）：DRIVE", distance_);
        }
        else
        {
            decision_msg.data = "none";
            ROS_WARN("信号未知（%.2fm）：UNKNOWN", distance_);
        }

        pub_.publish(decision_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;
    const float loop_interval_;

    std::string traffic_light_state_;
    float distance_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traffic_decision_node");

    TrafficDecisionPublisher node;
    ros::spin();

    return 0;
}