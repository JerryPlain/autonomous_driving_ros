#include <string>

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

namespace {

std::string g_output_path;

/**
 * @brief Converts an Octomap message and writes it to a .bt file.
 */
void OctomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
  std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::fullMsgToMap(*msg));
  if (!tree) {
    ROS_ERROR("Failed to convert Octomap message to an octree object");
    return;
  }

  octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree.get());
  if (octree == nullptr) {
    ROS_ERROR("Converted octree is not of type octomap::OcTree");
    return;
  }

  if (!octree->writeBinary(g_output_path)) {
    ROS_ERROR("Failed to write octomap to %s", g_output_path.c_str());
    return;
  }

  ROS_INFO("Saved octomap to %s", g_output_path.c_str());
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "save_octomap_subscriber");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string topic = "/octomap_binary";
  g_output_path = "/tmp/map.bt";

  pnh.param<std::string>("topic", topic, topic);
  pnh.param<std::string>("output_path", g_output_path, g_output_path);

  ros::Subscriber sub = nh.subscribe(topic, 1, OctomapCallback);
  ROS_INFO("Listening for octomap messages on %s", topic.c_str());

  ros::spin();
  return 0;
}
