#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>

class TrafficDecisionPublisher
{
public:
    TrafficDecisionPublisher()
        : nh_(), loop_interval_(0.05), traffic_light_state_("UNKNOWN")
    {
        pub_ = nh_.advertise<std_msgs::String>("traffic_decision", 1);  // 新话题
        sub_ = nh_.subscribe("traffic_light_color", 1, &TrafficDecisionPublisher::trafficLightCallback, this);
        timer_ = nh_.createTimer(ros::Duration(loop_interval_), &TrafficDecisionPublisher::decisionLoop, this);
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

    void decisionLoop(const ros::TimerEvent &)
    {
        std_msgs::String decision_msg;

        if (traffic_light_state_ == "RED")//检测到红灯，发布需要刹车的决策
        {
            decision_msg.data = "BRAKE";
            ROS_INFO("Red Light：  BRAKE");
        }
        else if (traffic_light_state_ == "YELLOW")//检测到红灯，发布加速的决策
        {
            decision_msg.data = "ACCELERATE";
            ROS_INFO("Yellow Light：  ACCELERATE");
        }
        else if (traffic_light_state_ == "GREEN")//检测到绿灯，发布正常行驶的决策
        {
            decision_msg.data = "DRIVE";
            ROS_INFO("Green Light： DRIVE");
        }
        else
        {
            decision_msg.data = "UNKNOWN";//未知，发布未知信号
            ROS_WARN("UNKNOWN， UNKNOWN");
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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traffic_decision_node");

    TrafficDecisionPublisher node;
    ros::spin();

    return 0;
}
