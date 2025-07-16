// this file is a ROS node that subscribes to octomap messages and saves the received octomap to a file in binary format.
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

// Callback function to handle incoming octomap messages
void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) 
{
    ROS_INFO("Received octomap message, trying to convert...");


    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
    if (tree)
    {

        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
        if (octree)
        {
            std::string filename = "/home/user/map.bt"; 
            if (octree->writeBinary(filename))
            {
                ROS_INFO("Octomap saved to %s", filename.c_str());
            }
            else
            {
                ROS_ERROR("Failed to save octomap to file!");
            }
        }
        else
        {
            ROS_ERROR("Failed to cast AbstractOcTree to OcTree!");
        }
        delete tree;
    }
    else
    {
        ROS_ERROR("Failed to convert Octomap message to tree!");
    }
}

// Main function to initialize the ROS node and subscribe to octomap messages
int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_octomap_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/octomap_binary", 1, octomapCallback);

    ROS_INFO("Waiting for octomap_binary messages...");
    ros::spin();

    return 0;
}