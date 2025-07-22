#include <ros/ros.h>
#include <std_msgs/String.h>
#include <traffic_light_detector/TrafficLightColor.h>

class TrafficDecisionPublisher
{
public:
    TrafficDecisionPublisher()
        : nh_(), loop_interval_(0.05), traffic_light_state_("UNKNOWN")
    {
        pub_ = nh_.advertise<std_msgs::String>("traffic_decision", 1);
        sub_ = nh_.subscribe("traffic_light_color", 1, &TrafficDecisionPublisher::trafficLightCallback, this);
        timer_ = nh_.createTimer(ros::Duration(loop_interval_), &TrafficDecisionPublisher::decisionLoop, this);

        last_msg_time_ = ros::Time::now();
        timeout_threshold_ = ros::Duration(1.0);  
    }

    void trafficLightCallback(const traffic_light_detector::TrafficLightColor::ConstPtr &msg)
    {
        traffic_light_state_ = msg->color;
        last_msg_time_ = ros::Time::now();  
        ROS_INFO("Received traffic light color: %s", traffic_light_state_.c_str());
    }

    void decisionLoop(const ros::TimerEvent &)
    {
        std_msgs::String decision_msg;

        if (ros::Time::now() - last_msg_time_ > timeout_threshold_)
        {
            ROS_WARN("No traffic light color received recently. Defaulting to DRIVE.");
            decision_msg.data = "DRIVE";
        }
        else
        {
            if (traffic_light_state_ == "red")
                decision_msg.data = "BRAKE";
            else if (traffic_light_state_ == "yellow")
                decision_msg.data = "ACCELERATE";
            else if (traffic_light_state_ == "green")
                decision_msg.data = "DRIVE";
            else
                decision_msg.data = "UNKNOWN";
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

    ros::Time last_msg_time_;
    ros::Duration timeout_threshold_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_making_node");
    TrafficDecisionPublisher node;
    ros::spin();
    return 0;
}