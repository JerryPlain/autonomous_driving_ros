#include <ros/ros.h>

/**
 * @brief Legacy entry point kept only for backward compatibility.
 *
 * This node intentionally does nothing and exits with success because the
 * functionality has been consolidated into `recordpoints_node`.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "recordpointsfromsimu_legacy_stub");
  ROS_WARN("recordpointsfromsimu is deprecated. Use recordpoints_node instead.");
  return 0;
}
