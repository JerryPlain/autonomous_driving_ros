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

class ColorDetectorNode
{
public:
  ColorDetectorNode(ros::NodeHandle& nh)
    : sub_boxes_(nh, "traffic_lights_bbox", 1),
      sub_img_(nh, "/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", 1)
  {
    typedef message_filters::sync_policies::ApproximateTime<
      traffic_light_detector::TrafficLightBBoxArray,
      sensor_msgs::Image
    > SyncPolicy;

    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), sub_boxes_, sub_img_));
    sync_->registerCallback(boost::bind(&ColorDetectorNode::callback, this, _1, _2));

    color_pub_ = nh.advertise<traffic_light_detector::TrafficLightColor>("traffic_light_color", 10);
    debug_img_pub_ = nh.advertise<sensor_msgs::Image>("traffic_light_color_debug", 1);

    // HSV-Farbbereiche
    lo_r_ = cv::Scalar(0,   69, 246);
    hi_r_ = cv::Scalar(11, 224, 255);
    lo_y_ = cv::Scalar(30, 144, 168);
    hi_y_ = cv::Scalar(37, 232, 255);
    lo_g_ = cv::Scalar(51, 122, 240);
    hi_g_ = cv::Scalar(67, 238, 255);

    ROS_INFO("ColorDetectorNode initialized (max distance 7m)");
  }

private:
  void callback(
    const traffic_light_detector::TrafficLightBBoxArray::ConstPtr& boxes_msg,
    const sensor_msgs::ImageConstPtr& img_msg)
  {
    const float max_distance = 7.0f;

    // Prüfen, ob irgendwo im Bild eine Ampel in Reichweite ist
    bool any_close = false;
    for (const auto& box : boxes_msg->boxes) {
      if (box.distance >= 0.0 && box.distance <= max_distance) {
        any_close = true;
        break;
      }
    }
    if (!any_close)
      return;

    // Kamera-Frame holen
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge error: %s", e.what());
      return;
    }
    cv::Mat img = cv_ptr->image;
    int W = img.cols, H = img.rows;

    // ROI: oberes Drittel in der Mitte
    cv::Rect roi(W / 3, 0, W / 3, H / 2);
    roi &= cv::Rect(0, 0, W, H);
    if (roi.empty()) return;

    cv::Mat view = img(roi).clone();
    bool any_color_published = false;

    for (const auto& box : boxes_msg->boxes)
    {
      // Nur Boxen im ROI analysieren
      cv::Rect r(box.x - roi.x,
                 box.y - roi.y,
                 box.width,
                 box.height);
      r &= cv::Rect(0, 0, view.cols, view.rows);
      if (r.empty())
        continue;

      // HSV-Analyse
      cv::Mat hsv_patch;
      cv::cvtColor(view(r), hsv_patch, cv::COLOR_BGR2HSV);

      cv::Mat mr, my, mg;
      cv::inRange(hsv_patch, lo_r_, hi_r_, mr);
      cv::inRange(hsv_patch, lo_y_, hi_y_, my);
      cv::inRange(hsv_patch, lo_g_, hi_g_, mg);
      int cnt_r = cv::countNonZero(mr);
      int cnt_y = cv::countNonZero(my);
      int cnt_g = cv::countNonZero(mg);

      std::string color = "none";
      if      (cnt_r > cnt_y && cnt_r > cnt_g) color = "red";
      else if (cnt_y > cnt_r && cnt_y > cnt_g) color = "yellow";
      else if (cnt_g > cnt_r && cnt_g > cnt_y) color = "green";

      // Publish – immer
      traffic_light_detector::TrafficLightColor msg;
      msg.id = box.id;
      msg.color = color;
      msg.distance = box.distance;
      color_pub_.publish(msg);
      any_color_published = true;

      // Debugfarbe
      cv::Scalar dbg_color;
      if (color == "red")        dbg_color = cv::Scalar(0, 0, 255);
      else if (color == "yellow") dbg_color = cv::Scalar(0, 255, 255);
      else if (color == "green")  dbg_color = cv::Scalar(0, 255, 0);
      else                        dbg_color = cv::Scalar(128, 128, 128);

      cv::Rect full_box(box.x, box.y, box.width, box.height);
      cv::rectangle(img, full_box, dbg_color, 2);
      cv::putText(img, color, cv::Point(full_box.x, full_box.y - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, dbg_color, 1);
    }

    // Wenn keine Box im ROI verarbeitet wurde, trotzdem "none" publishen
    if (!any_color_published) {
      traffic_light_detector::TrafficLightColor msg;
      msg.id = -1;
      msg.color = "none";
      msg.distance = -1.0;
      color_pub_.publish(msg);
    }

    // ROI einzeichnen
    cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 2);
    cv::putText(img, "ROI", cv::Point(roi.x + 5, roi.y + 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

    // Debugbild publishen
    cv_bridge::CvImage debug_msg;
    debug_msg.header = img_msg->header;
    debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
    debug_msg.image = img;
    debug_img_pub_.publish(debug_msg.toImageMsg());
  }

  message_filters::Subscriber<traffic_light_detector::TrafficLightBBoxArray> sub_boxes_;
  message_filters::Subscriber<sensor_msgs::Image> sub_img_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      traffic_light_detector::TrafficLightBBoxArray,
      sensor_msgs::Image>>> sync_;

  ros::Publisher color_pub_;
  ros::Publisher debug_img_pub_;
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

