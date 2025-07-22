// this file is a ROS node that publishes static transforms for sensors on a car
// It uses the tf2_ros library to broadcast static transforms, which are used to define the spatial relationships between different frames in a robotic system.
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

/// Function to publish a static transform
void publishStaticTF(const std::string& parent, const std::string& child,
                     double x, double y, double z,
                     double qx, double qy, double qz, double qw)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster; // Create a static broadcaster instance
    geometry_msgs::TransformStamped tf_msg; // Create a TransformStamped message

    tf_msg.header.stamp = ros::Time::now();  // current timestamp
    tf_msg.header.frame_id = parent; // parent frame
    tf_msg.child_frame_id = child; // child frame

    // Set the translation and rotation
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;

    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;

    static_broadcaster.sendTransform(tf_msg); // send the transform
}

// Main function to initialize the ROS node and publish static transforms
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "static_tf_broadcaster_node");
    ros::NodeHandle nh;

    // wait for the ROS time to be valid
    ros::Time::waitForValid();

    // Publish static transforms for the sensors on the car
    // The quaternion values are set to represent a 90-degree rotation around the Z-axis
    const double qx = -0.5, qy = 0.5, qz = -0.5, qw = 0.5;

    publishStaticTF("OurCar/INS", "OurCar/Sensors/DepthCamera",    0.3,  0.0, 1.2, qx, qy, qz, qw);
    publishStaticTF("OurCar/INS", "OurCar/Sensors/RGBCameraLeft",  0.3,  0.1, 1.2, qx, qy, qz, qw);
    publishStaticTF("OurCar/INS", "OurCar/Sensors/RGBCameraRight", 0.3, -0.1, 1.2, qx, qy, qz, qw);
    publishStaticTF("OurCar/INS", "OurCar/Sensors/SemanticCamera", 0.3,  0.0, 1.3, qx, qy, qz, qw);


    ROS_INFO(" Static TFs published with timestamps (C++)"); // Log the publication of static transforms

    ros::spin(); // Keep the node running to maintain the static transforms
    return 0; // Keep the node running
}
