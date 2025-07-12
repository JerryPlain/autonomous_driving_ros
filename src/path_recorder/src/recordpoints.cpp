#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <tf/tf.h>

class Recorder
{
public:
    Recorder() : nh_("~"), received_pose_(false), received_twist_(false)
    {
        // 订阅位置
        pose_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/pose", 1, &Recorder::poseCallback, this);

        // 订阅速度
        twist_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/CoM/twist", 1, &Recorder::twistCallback, this);

        // 定时器，0.5 秒打一个点
        record_timer_ = nh_.createTimer(ros::Duration(0.5), &Recorder::recordCallback, this);

        std::string filename = "recorded_path.csv";
        csv_file_.open(filename, std::ios::out | std::ios::trunc);

        if (!csv_file_.is_open())
        {
            ROS_ERROR("无法打开文件: %s", filename.c_str());
        }
        else
        {
            ROS_INFO("CSV 文件已打开: %s", filename.c_str());
            csv_file_ << "timestamp,x,y,z,yaw,velocity\n";
            csv_file_.flush();
        }
    }

    ~Recorder()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;
    ros::Timer record_timer_;

    geometry_msgs::Pose current_pose_;
    geometry_msgs::Twist current_twist_;
    bool received_pose_;
    bool received_twist_;

    std::ofstream csv_file_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pose_ = msg->pose;
        received_pose_ = true;

        ROS_INFO_ONCE("已收到 PoseStamped 消息，开始记录逻辑已准备！");
    }

    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        current_twist_ = msg->twist;
        received_twist_ = true;

        ROS_INFO_ONCE("已收到 TwistStamped 消息！");
    }

    void recordCallback(const ros::TimerEvent&)
    {
        if (!received_pose_ || !received_twist_)
        {
            ROS_WARN_THROTTLE(5, "还未收到位姿或速度消息，无法记录");
            return;
        }

        double x = current_pose_.position.x;
        double y = current_pose_.position.y;
        double z = current_pose_.position.z;

        tf::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        );
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 计算线速度大小
        double velocity = std::sqrt(
            std::pow(current_twist_.linear.x, 2) +
            std::pow(current_twist_.linear.y, 2) +
            std::pow(current_twist_.linear.z, 2)
        );

        // 判断速度阈值
        if (velocity > 0.01)
        {
            double timestamp = ros::Time::now().toSec();

            csv_file_ << timestamp << "," << x << "," << y << "," << z << ","
                      << yaw << "," << velocity << "\n";
            csv_file_.flush();

            ROS_INFO("记录点: t=%.2f x=%.2f y=%.2f z=%.2f yaw=%.2f vel=%.2f",
                     timestamp, x, y, z, yaw, velocity);
        }
        else
        {
            ROS_INFO_THROTTLE(5, "速度为 0，暂停记录");
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "recordpoints_node");
    Recorder recorder;
    ros::spin();
    return 0;
}
