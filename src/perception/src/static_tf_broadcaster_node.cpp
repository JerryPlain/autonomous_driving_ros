#include <array>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace {

struct SensorTF {
  std::string parent;
  std::string child;
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
};

/**
 * @brief Publishes static sensor transforms for the ego vehicle.
 */
int Run() {
  ros::NodeHandle nh;

  // Quaternion corresponds to the camera mounting orientation used in simulation.
  constexpr double qx = -0.5;
  constexpr double qy = 0.5;
  constexpr double qz = -0.5;
  constexpr double qw = 0.5;

  const std::array<SensorTF, 4> sensors = {{{"OurCar/INS", "OurCar/Sensors/DepthCamera", 0.3, 0.0, 1.2, qx, qy, qz, qw},
                                            {"OurCar/INS", "OurCar/Sensors/RGBCameraLeft", 0.3, 0.1, 1.2, qx, qy, qz, qw},
                                            {"OurCar/INS", "OurCar/Sensors/RGBCameraRight", 0.3, -0.1, 1.2, qx, qy, qz, qw},
                                            {"OurCar/INS", "OurCar/Sensors/SemanticCamera", 0.3, 0.0, 1.3, qx, qy, qz, qw}}};

  std::vector<geometry_msgs::TransformStamped> transforms;
  transforms.reserve(sensors.size());

  const ros::Time stamp = ros::Time::now();
  for (const auto& sensor : sensors) {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = sensor.parent;
    tf.child_frame_id = sensor.child;
    tf.transform.translation.x = sensor.x;
    tf.transform.translation.y = sensor.y;
    tf.transform.translation.z = sensor.z;
    tf.transform.rotation.x = sensor.qx;
    tf.transform.rotation.y = sensor.qy;
    tf.transform.rotation.z = sensor.qz;
    tf.transform.rotation.w = sensor.qw;
    transforms.push_back(tf);
  }

  tf2_ros::StaticTransformBroadcaster broadcaster;
  broadcaster.sendTransform(transforms);
  ROS_INFO("Published %zu static transforms for perception sensors", transforms.size());

  ros::spin();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "static_tf_broadcaster_node");
  return Run();
}
