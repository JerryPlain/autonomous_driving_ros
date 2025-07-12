#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
    ROS_INFO("Received octomap message, trying to convert...");

    // 转换为 AbstractOcTree
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
    if (tree)
    {
        // 尝试转为 OcTree 类型（最常用）
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
        if (octree)
        {
            std::string filename = "/home/user/map.bt";  // 修改保存路径
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
        delete tree;  // 别忘了释放内存
    }
    else
    {
        ROS_ERROR("Failed to convert Octomap message to tree!");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_octomap_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/octomap_binary", 1, octomapCallback);

    ROS_INFO("Waiting for octomap_binary messages...");
    ros::spin();

    return 0;
}

