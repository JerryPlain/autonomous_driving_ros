#ifndef PATH_RECORDER_HPP
#define PATH_RECORDER_HPP

#include <fstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace path_recorder {

/**
 * @brief Records sparse waypoints from pose/twist streams into a normalized CSV format.
 *
 * CSV schema:
 * index,timestamp,x,y,z,yaw,velocity
 */
class PathRecorder {
public:
  PathRecorder();
  ~PathRecorder();

  /**
   * @brief Opens CSV output and writes header.
   */
  bool Start(const std::string& csv_path);

  /**
   * @brief Stops recording and closes CSV file.
   */
  void Stop();

  /**
   * @brief Updates latest velocity sample used when writing a waypoint row.
   */
  void UpdateTwist(const geometry_msgs::TwistStamped& twist_msg);

  /**
   * @brief Returns planar distance between two points.
   */
  static double PlanarDistance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

  /**
   * @brief Writes one waypoint row from a pose sample.
   */
  bool RecordPose(const geometry_msgs::PoseStamped& pose_msg);

private:
  std::ofstream csv_file_;
  geometry_msgs::TwistStamped last_twist_;
  std::size_t next_index_;
};

}  // namespace path_recorder

#endif  // PATH_RECORDER_HPP
