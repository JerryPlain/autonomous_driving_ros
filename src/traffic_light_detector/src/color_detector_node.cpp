// src/color_detector_node.cpp

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <traffic_light_detector/TrafficLightBBoxArray.h>
#include <traffic_light_detector/TrafficLightColor.h> 

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class ColorDetectorNode
{
public:
  ColorDetectorNode(ros::NodeHandle& nh)
    : nh_(nh),
      sub_boxes_(nh_, "traffic_lights_bbox_rgb", 1),
      sub_img_(nh_, "/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", 1)
  {
    // synchronize bounding‐boxes and camera images
    typedef message_filters::sync_policies::ApproximateTime<
      traffic_light_detector::TrafficLightBBoxArray,
      sensor_msgs::Image
    > SyncPolicy;
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
      SyncPolicy(10), sub_boxes_, sub_img_));
    sync_->registerCallback(
      boost::bind(&ColorDetectorNode::callback, this, _1, _2));

    // publisher for traffic‐light color messages
    color_pub_ = nh_.advertise<traffic_light_detector::TrafficLightColor>(
      "traffic_light_color", 1);

    // HSV thresholds for red, yellow, green
    lo_r_ = cv::Scalar(0,   69, 246);
    hi_r_ = cv::Scalar(11, 224, 255);
    lo_y_ = cv::Scalar(30, 144, 168);
    hi_y_ = cv::Scalar(37, 232, 255);
    lo_g_ = cv::Scalar(51, 122, 240);
    hi_g_ = cv::Scalar(67, 238, 255);

    ROS_INFO("ColorDetectorNode initialized (max distance 25m)");
  }

  ~ColorDetectorNode()
  {
    cv::destroyAllWindows();
  }

private:
  void callback(
    const traffic_light_detector::TrafficLightBBoxArray::ConstPtr& boxes_msg,
    const sensor_msgs::ImageConstPtr& img_msg)
  {
    // convert ROS image to OpenCV BGR
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(
        img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge error: %s", e.what());
      return;
    }
    const cv::Mat& img = cv_ptr->image;
    int W = img.cols, H = img.rows;

    // define ROI: upper half, middle third
    cv::Rect roi(W/3, 0, W/3, H/2);
    roi &= cv::Rect(0, 0, W, H);
    if (roi.empty()) return;
    cv::Mat view = img(roi).clone();

    // for each detected box, filter by distance and determine color
    for (const auto& box : boxes_msg->boxes)
    {
      // skip if farther than 25 meters
      if (box.distance > 25.0)
        continue;

      // compute local patch rectangle
      cv::Rect r(box.x - roi.x,
                 box.y - roi.y,
                 box.width,
                 box.height);
      r &= cv::Rect(0, 0, view.cols, view.rows);
      if (r.empty())
        continue;

      // extract HSV patch
      cv::Mat hsv_patch;
      cv::cvtColor(view(r), hsv_patch, cv::COLOR_BGR2HSV);

      // create masks and count pixels
      cv::Mat mr, my, mg;
      cv::inRange(hsv_patch, lo_r_, hi_r_, mr);
      cv::inRange(hsv_patch, lo_y_, hi_y_, my);
      cv::inRange(hsv_patch, lo_g_, hi_g_, mg);
      int cnt_r = cv::countNonZero(mr);
      int cnt_y = cv::countNonZero(my);
      int cnt_g = cv::countNonZero(mg);

      // select color with largest pixel count
      std::string color = "none";
      if      (cnt_r > cnt_y && cnt_r > cnt_g) color = "red";
      else if (cnt_y > cnt_r && cnt_y > cnt_g) color = "yellow";
      else if (cnt_g > cnt_r && cnt_g > cnt_y) color = "green";

      // only publish if a color was detected
      if (color != "none")
      {
        traffic_light_detector::TrafficLightColor msg;
        msg.id    = box.id;
        msg.color = color;
        color_pub_.publish(msg);
      }
    }
  }

  ros::NodeHandle nh_;
  message_filters::Subscriber<traffic_light_detector::TrafficLightBBoxArray> sub_boxes_;
  message_filters::Subscriber<sensor_msgs::Image>                             sub_img_;
  boost::shared_ptr<
    message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        traffic_light_detector::TrafficLightBBoxArray,
        sensor_msgs::Image
      >
    >
  > sync_;
  ros::Publisher color_pub_;

  // HSV threshold ranges
  cv::Scalar lo_r_, hi_r_, lo_y_, hi_y_, lo_g_, hi_g_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_detector_node");
  ros::NodeHandle nh;
  ColorDetectorNode node(nh);
  ros::spin();
  return 0;
}
