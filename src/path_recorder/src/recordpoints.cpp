#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <sstream>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "csv_tf_publisher");
    ros::NodeHandle nh;

    tf2_ros::StaticTransformBroadcaster static_broadcaster;

    std::vector<geometry_msgs::TransformStamped> transforms;

    // Change to your CSV file path!
    std::string csv_file = "/home/user/recorded_path.csv";
    std::ifstream file(csv_file);

    if (!file.is_open())
    {
        ROS_ERROR("Failed to open CSV file: %s", csv_file.c_str());
        return 1;
    }

    std::string line;
    int line_count = 0;

    // Skip header
    std::getline(file, line);

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;

        double x = 0, y = 0, z = 0;

        // Example CSV columns: index,x,y,z
        std::getline(ss, value, ','); // index (we won't use this directly, we'll assign our own index)
        std::getline(ss, value, ','); x = std::stod(value);
        std::getline(ss, value, ','); y = std::stod(value);
        std::getline(ss, value, ','); z = std::stod(value);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "WorldFrame";
        transformStamped.child_frame_id = "frame" + std::to_string(line_count); // start at frame0

        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = z;

        // Default identity rotation
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        transforms.push_back(transformStamped);

        ROS_INFO("Prepared static TF for frame%d at (%.2f, %.2f, %.2f)", line_count, x, y, z);
        line_count++;
    }

    file.close();

    // Publish all static TFs
    static_broadcaster.sendTransform(transforms);

    ROS_INFO("Published %lu static transforms from CSV", transforms.size());

    ros::spin();
    return 0;
}
