#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void publishStaticTF(const std::string& parent, const std::string& child,
                     double x, double y, double z,
                     double qx, double qy, double qz, double qw)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped tf_msg;

    tf_msg.header.stamp = ros::Time::now();  // 当前时间戳
    tf_msg.header.frame_id = parent;
    tf_msg.child_frame_id = child;

    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;

    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;

    static_broadcaster.sendTransform(tf_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_tf_broadcaster_node");
    ros::NodeHandle nh;

    // 等待时间系统有效（无论是 sim_time 或 wall time）
    ros::Time::waitForValid();

    publishStaticTF("OurCar/INS", "OurCar/Sensors/DepthCamera",     0.3, 0.0, 1.2, 0, 0, 0, 1);
    publishStaticTF("OurCar/INS", "OurCar/Sensors/RGBCameraLeft",   0.3, 0.1, 1.2, 0, 0, 0, 1);
    publishStaticTF("OurCar/INS", "OurCar/Sensors/RGBCameraRight",  0.3, -0.1, 1.2, 0, 0, 0, 1);
    publishStaticTF("OurCar/INS", "OurCar/Sensors/SemanticCamera",  0.3, 0.0, 1.3, 0, 0, 0, 1);

    ROS_INFO(" Static TFs published with timestamps (C++)");

    ros::spin();  // 必须 spin 保持 TF 广播生效
    return 0;
}
