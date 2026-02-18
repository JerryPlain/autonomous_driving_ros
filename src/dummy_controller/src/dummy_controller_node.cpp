#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <simulation/VehicleControl.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {

double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/**
 * @brief PID tracking controller with traffic decision override.
 *
 * Inputs:
 * - /move_base_simple/goal
 * - /Unity_ROS_message_Rx/OurCar/CoM/pose
 * - /Unity_ROS_message_Rx/OurCar/CoM/twist
 * - traffic_decision
 *
 * Outputs:
 * - car_command (simulation/VehicleControl)
 * - /reached_goal (std_msgs/Int32)
 */
class PIDControllerNode {
public:
  PIDControllerNode()
      : nh_(),
        pnh_("~"),
        has_goal_(false),
        has_pose_(false),
        has_speed_(false),
        current_speed_(0.0),
        current_goal_index_(-1),
        goal_already_reported_(false),
        final_goal_reached_(false),
        loop_interval_sec_(0.05),
        max_goal_index_(1042),
        goal_tolerance_m_(0.5),
        kp_dist_(0.25),
        ki_dist_(0.0),
        kd_dist_(0.1),
        kp_yaw_(7.5),
        ki_yaw_(0.0),
        kd_yaw_(0.1),
        prev_error_dist_(0.0),
        integral_dist_(0.0),
        prev_error_yaw_(0.0),
        integral_yaw_(0.0),
        traffic_decision_("UNKNOWN") {
    std::string goal_topic = "/move_base_simple/goal";
    std::string pose_topic = "/Unity_ROS_message_Rx/OurCar/CoM/pose";
    std::string speed_topic = "/Unity_ROS_message_Rx/OurCar/CoM/twist";
    std::string decision_topic = "traffic_decision";
    std::string command_topic = "car_command";
    std::string reached_topic = "/reached_goal";

    pnh_.param<std::string>("goal_topic", goal_topic, goal_topic);
    pnh_.param<std::string>("pose_topic", pose_topic, pose_topic);
    pnh_.param<std::string>("speed_topic", speed_topic, speed_topic);
    pnh_.param<std::string>("decision_topic", decision_topic, decision_topic);
    pnh_.param<std::string>("command_topic", command_topic, command_topic);
    pnh_.param<std::string>("reached_topic", reached_topic, reached_topic);

    pnh_.param<double>("loop_interval_sec", loop_interval_sec_, loop_interval_sec_);
    pnh_.param<int>("max_goal_index", max_goal_index_, max_goal_index_);
    pnh_.param<double>("goal_tolerance_m", goal_tolerance_m_, goal_tolerance_m_);

    pnh_.param<double>("kp_dist", kp_dist_, kp_dist_);
    pnh_.param<double>("ki_dist", ki_dist_, ki_dist_);
    pnh_.param<double>("kd_dist", kd_dist_, kd_dist_);
    pnh_.param<double>("kp_yaw", kp_yaw_, kp_yaw_);
    pnh_.param<double>("ki_yaw", ki_yaw_, ki_yaw_);
    pnh_.param<double>("kd_yaw", kd_yaw_, kd_yaw_);

    goal_sub_ = nh_.subscribe(goal_topic, 10, &PIDControllerNode::GoalCallback, this);
    pose_sub_ = nh_.subscribe(pose_topic, 20, &PIDControllerNode::PoseCallback, this);
    speed_sub_ = nh_.subscribe(speed_topic, 20, &PIDControllerNode::SpeedCallback, this);
    decision_sub_ = nh_.subscribe(decision_topic, 10, &PIDControllerNode::DecisionCallback, this);

    command_pub_ = nh_.advertise<simulation::VehicleControl>(command_topic, 10);
    reached_pub_ = nh_.advertise<std_msgs::Int32>(reached_topic, 10);

    ROS_INFO("dummy_controller_node started. command_topic=%s", command_topic.c_str());
  }

  void Spin() {
    ros::Rate rate(1.0 / loop_interval_sec_);
    while (ros::ok()) {
      ros::spinOnce();
      PublishControl();
      rate.sleep();
    }
  }

private:
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_goal_ = *msg;
    current_goal_index_ = static_cast<int>(msg->header.seq);
    goal_already_reported_ = false;
    has_goal_ = true;

    // Reset distance/yaw PID integrators on each new goal for stable transitions.
    prev_error_dist_ = 0.0;
    integral_dist_ = 0.0;
    prev_error_yaw_ = 0.0;
    integral_yaw_ = 0.0;
  }

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = msg->pose;
    has_pose_ = true;
  }

  void SpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    current_speed_ = msg->twist.linear.x;
    has_speed_ = true;
  }

  void DecisionCallback(const std_msgs::String::ConstPtr& msg) {
    traffic_decision_ = msg->data;
  }

  /**
   * @brief Builds a safe stop command used as fallback and final state.
   */
  static simulation::VehicleControl StopCommand() {
    simulation::VehicleControl cmd;
    cmd.Throttle = 0.0;
    cmd.Steering = 0.0;
    cmd.Brake = 1.0;
    cmd.Reserved = 0.0;
    return cmd;
  }

  void PublishControl() {
    simulation::VehicleControl cmd = StopCommand();

    if (final_goal_reached_) {
      command_pub_.publish(cmd);
      return;
    }

    if (!(has_goal_ && has_pose_)) {
      command_pub_.publish(cmd);
      return;
    }

    const double cx = current_pose_.position.x;
    const double cy = current_pose_.position.y;
    const double gx = current_goal_.pose.position.x;
    const double gy = current_goal_.pose.position.y;

    const double dx = gx - cx;
    const double dy = gy - cy;
    const double dist_err = std::sqrt(dx * dx + dy * dy);

    const double target_yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    tf2::fromMsg(current_pose_.orientation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double current_yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

    const double yaw_err = NormalizeAngle(target_yaw - current_yaw);

    // Longitudinal PID for throttle command.
    integral_dist_ += dist_err * loop_interval_sec_;
    const double derivative_dist = (dist_err - prev_error_dist_) / loop_interval_sec_;
    double throttle = kp_dist_ * dist_err + ki_dist_ * integral_dist_ + kd_dist_ * derivative_dist;
    prev_error_dist_ = dist_err;

    // Lateral PID for steering command.
    integral_yaw_ += yaw_err * loop_interval_sec_;
    const double derivative_yaw = (yaw_err - prev_error_yaw_) / loop_interval_sec_;
    double steering = -(kp_yaw_ * yaw_err + ki_yaw_ * integral_yaw_ + kd_yaw_ * derivative_yaw);
    prev_error_yaw_ = yaw_err;

    throttle = std::max(0.0, std::min(1.0, throttle));
    steering = std::max(-1.0, std::min(1.0, steering));

    cmd.Throttle = throttle;
    cmd.Steering = steering;
    cmd.Brake = 0.0;

    // Hard safety override for red light decisions.
    if (traffic_decision_ == "BRAKE") {
      cmd = StopCommand();
    }

    if (dist_err < goal_tolerance_m_) {
      cmd.Throttle = 0.0;
      cmd.Brake = 0.0;

      if (!goal_already_reported_ && current_goal_index_ >= 0) {
        std_msgs::Int32 reached_msg;
        reached_msg.data = current_goal_index_;
        reached_pub_.publish(reached_msg);
        goal_already_reported_ = true;

        if (current_goal_index_ >= max_goal_index_) {
          final_goal_reached_ = true;
          cmd = StopCommand();
        }
      }
    }

    if (has_speed_) {
      ROS_INFO_THROTTLE(0.5,
                        "pos=(%.2f, %.2f) speed=%.2f throttle=%.2f steering=%.2f dist=%.2f",
                        cx,
                        cy,
                        current_speed_,
                        cmd.Throttle,
                        cmd.Steering,
                        dist_err);
    }

    command_pub_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber goal_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber decision_sub_;

  ros::Publisher command_pub_;
  ros::Publisher reached_pub_;

  geometry_msgs::PoseStamped current_goal_;
  geometry_msgs::Pose current_pose_;

  bool has_goal_;
  bool has_pose_;
  bool has_speed_;

  double current_speed_;
  int current_goal_index_;
  bool goal_already_reported_;
  bool final_goal_reached_;

  double loop_interval_sec_;
  int max_goal_index_;
  double goal_tolerance_m_;

  double kp_dist_;
  double ki_dist_;
  double kd_dist_;
  double kp_yaw_;
  double ki_yaw_;
  double kd_yaw_;

  double prev_error_dist_;
  double integral_dist_;
  double prev_error_yaw_;
  double integral_yaw_;

  std::string traffic_decision_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "pid_controller_node");
  PIDControllerNode node;
  node.Spin();
  return 0;
}
