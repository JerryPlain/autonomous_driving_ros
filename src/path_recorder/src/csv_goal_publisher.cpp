#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <fstream>
#include <sstream>
#include <vector>

// 定义轨迹点结构体
struct Waypoint {
    double x;
    double y;
    double theta;
};

std::vector<Waypoint> readWaypointsFromCSV(const std::string& filepath)
{
    std::vector<Waypoint> waypoints;
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open CSV file: %s", filepath.c_str());
        return waypoints;
    }

    std::string line;
    getline(file, line);  // 跳过表头

    while (getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;
        Waypoint wp;
        // index
        getline(ss, token, ',');

        // time
        getline(ss, token, ',');

        // x
        getline(ss, token, ',');
        wp.x = std::stod(token);

        // y
        getline(ss, token, ',');
        wp.y = std::stod(token);

        // v
        getline(ss, token, ',');

        // theta（可以自行加角度，这里默认 0）
        wp.theta = 0.0;

        waypoints.push_back(wp);
    }
    file.close();
    ROS_INFO("Read %lu waypoints from CSV.", waypoints.size());
    return waypoints;
}

// 当前索引
int current_index = 0;
bool goal_reached = true;

void reachedCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Received reached signal for index: %d", msg->data);
    goal_reached = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_goal_publisher");
    ros::NodeHandle nh;

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Subscriber reached_sub = nh.subscribe("/reached_goal", 1, reachedCallback);

    std::string csv_path = "/home/kenway/桌面/tum/i2ros/main/src/path_recorder/recorded_path.csv";
    std::vector<Waypoint> waypoints = readWaypointsFromCSV(csv_path);

    // ⭐ 加入等待
while (goal_pub.getNumSubscribers() == 0)
{
    ROS_WARN("Waiting for subscriber to connect to /move_base_simple/goal ...");
    ros::Duration(0.5).sleep();
}

    ros::Rate rate(10);

    while (ros::ok() && current_index < waypoints.size())
    {
        ros::spinOnce();

        if (goal_reached)
        {
            Waypoint wp = waypoints[current_index];

            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.header.stamp = ros::Time::now();
            goal.pose.position.x = wp.x;
            goal.pose.position.y = wp.y;
            goal.pose.orientation.w = 1.0;  // 默认朝向

            // 把索引写进 header.seq
            goal.header.seq = current_index;

            goal_pub.publish(goal);

            ROS_INFO("Publishing goal index: %d, x = %.2f, y = %.2f", current_index, wp.x, wp.y);
        

            goal_reached = false;
            current_index++;
        }

        rate.sleep();
    }

    ROS_INFO("All goals published. Task completed.");

    return 0;
}
