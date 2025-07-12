#ifndef PATH_RECORDER_HPP
#define PATH_RECORDER_HPP

#include <string>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>

class PathRecorder {
public:
    PathRecorder();
    ~PathRecorder();

    void startRecording(const std::string& filename);
    void stopRecording();
    void recordTurn(const std::string& turn_type, const geometry_msgs::PoseStamped& msg);

    void setStartPoint(const geometry_msgs::Point& point);
    void setTwist(const geometry_msgs::TwistStamped& twist);
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);

private:
    std::ofstream logfile;
    geometry_msgs::Point start_point;            // 小车静止时作为原点
    geometry_msgs::Point last_point;             // 上一个记录点
    geometry_msgs::TwistStamped last_twist;      // 上一次速度
    nav_msgs::Path path_msg;                     // 可用于可视化的路径消息
};

#endif  // PATH_RECORDER_HPP

