#include <algorithm>
#include <cctype>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <traffic_light_detector/TrafficLightColor.h>

namespace {

/**
 * @brief Normalizes color labels to lowercase to keep message mapping robust.
 */
std::string NormalizeColor(std::string color) {
  std::transform(color.begin(), color.end(), color.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return color;
}

/**
 * @brief Converts traffic light state into a control-friendly decision label.
 */
std::string DecisionFromColor(const std::string& color) {
  if (color == "red") {
    return "BRAKE";
  }
  if (color == "yellow") {
    return "ACCELERATE";
  }
  if (color == "green") {
    return "DRIVE";
  }
  if (color == "none") {
    return "UNKNOWN";
  }
  return "UNKNOWN";
}

class TrafficDecisionNode {
public:
  TrafficDecisionNode()
      : nh_(),
        pnh_("~"),
        decision_timeout_sec_(1.0),
        publish_interval_sec_(0.05),
        latest_color_("none") {
    std::string input_topic = "traffic_light_color";
    std::string output_topic = "traffic_decision";

    pnh_.param<std::string>("input_topic", input_topic, input_topic);
    pnh_.param<std::string>("output_topic", output_topic, output_topic);
    pnh_.param<double>("decision_timeout_sec", decision_timeout_sec_, decision_timeout_sec_);
    pnh_.param<double>("publish_interval_sec", publish_interval_sec_, publish_interval_sec_);

    decision_pub_ = nh_.advertise<std_msgs::String>(output_topic, 10);
    color_sub_ = nh_.subscribe(input_topic, 10, &TrafficDecisionNode::ColorCallback, this);
    timer_ = nh_.createTimer(ros::Duration(publish_interval_sec_),
                             &TrafficDecisionNode::PublishDecision, this);

    last_color_update_ = ros::Time(0);
    ROS_INFO("decision_making_node started. input=%s output=%s timeout=%.2f",
             input_topic.c_str(), output_topic.c_str(), decision_timeout_sec_);
  }

private:
  /**
   * @brief Stores latest color label and update timestamp from detector output.
   */
  void ColorCallback(const traffic_light_detector::TrafficLightColor::ConstPtr& msg) {
    latest_color_ = NormalizeColor(msg->color);
    last_color_update_ = ros::Time::now();
  }

  /**
   * @brief Publishes a decision periodically; falls back to DRIVE after timeout.
   */
  void PublishDecision(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();

    std_msgs::String decision_msg;
    if (last_color_update_.isZero() ||
        (now - last_color_update_).toSec() > decision_timeout_sec_) {
      decision_msg.data = "DRIVE";
    } else {
      decision_msg.data = DecisionFromColor(latest_color_);
    }

    decision_pub_.publish(decision_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber color_sub_;
  ros::Publisher decision_pub_;
  ros::Timer timer_;

  double decision_timeout_sec_;
  double publish_interval_sec_;
  std::string latest_color_;
  ros::Time last_color_update_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "decision_making_node");
  TrafficDecisionNode node;
  ros::spin();
  return 0;
}
