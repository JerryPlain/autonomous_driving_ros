#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <traffic_light_hsv_detector/BoundingBoxes.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class ColorDetector {
public:
  ColorDetector(ros::NodeHandle& nh)
    : it_(nh)
  {
    // 1) Publisher für Ampelfarbe
    pub_ = nh.advertise<std_msgs::String>("traffic_light/color", 1);

    // 2) Subscriber für Bounding-Boxes
    bbox_sub_ = nh.subscribe(
      "traffic_light/bounding_boxes", 1,
      &ColorDetector::bboxCb, this
    );

    // 3) Subscriber für RGB-Bild
    img_sub_ = it_.subscribe(
      "/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw",
      1,
      &ColorDetector::imageCb,
      this
    );

    // HSV-Ranges initialisieren
    min_pixel_count_ = 1;
    lo_red1_   = cv::Scalar(  0, 120, 200);
    hi_red1_   = cv::Scalar( 10, 255, 255);
    lo_red2_   = cv::Scalar(170, 120, 200);
    hi_red2_   = cv::Scalar(179, 255, 255);
    lo_yellow_ = cv::Scalar( 25, 140, 200);
    hi_yellow_ = cv::Scalar( 35, 225, 255);
    lo_green_  = cv::Scalar( 40, 130, 200);
    hi_green_  = cv::Scalar( 80, 245, 255);

    ROS_INFO("ColorDetector gestartet, warte auf Bounding-Boxes …");
  }

private:
void bboxCb(const traffic_light_hsv_detector::BoundingBoxes::ConstPtr& msg)
{
  bboxes_.clear();
  ROS_INFO("Empfange %zu BoundingBoxes", msg->boxes.size());

  for (const auto& bb : msg->boxes)
  {
    // bb ist hier gültig – und wir benutzen printf-Style Platzhalter
    ROS_INFO("  Box: x=%d, y=%d, width=%d, height=%d",
             bb.x, bb.y, bb.width, bb.height);

    bboxes_.emplace_back(bb.x, bb.y, bb.width, bb.height);
  }
}


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if (bboxes_.empty()) return;

    // BGR → HSV
    cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // Für jede Box: Farbe detektieren
    for (size_t i = 0; i < bboxes_.size(); ++i)
    {
      cv::Rect roi = bboxes_[i] & cv::Rect(0,0,hsv.cols,hsv.rows);
      if (roi.area() < 1) continue;

      cv::Mat patch = hsv(roi);
      // Rot-Maske
      cv::Mat m1, m2, mask_r;
      cv::inRange(patch, lo_red1_, hi_red1_, m1);
      cv::inRange(patch, lo_red2_, hi_red2_, m2);
      cv::bitwise_or(m1, m2, mask_r);
      // Gelb & Grün
      cv::Mat mask_y, mask_g;
      cv::inRange(patch, lo_yellow_, hi_yellow_, mask_y);
      cv::inRange(patch, lo_green_,  hi_green_,  mask_g);

      int cnt_r = cv::countNonZero(mask_r);
      int cnt_y = cv::countNonZero(mask_y);
      int cnt_g = cv::countNonZero(mask_g);

      std::string color = "UNKNOWN";
      if      (cnt_r > cnt_y && cnt_r > cnt_g) color = "RED";
      else if (cnt_y > cnt_r && cnt_y > cnt_g) color = "YELLOW";
      else if (cnt_g > cnt_r && cnt_g > cnt_y) color = "GREEN";

      std_msgs::String out;
      out.data = "TL#" + std::to_string(i) + ":" + color;
      pub_.publish(out);

      cv::rectangle(bgr, roi, cv::Scalar(255,0,0), 2);
    }

    cv::imshow("detected_lights", bgr);
    cv::waitKey(1);
  }

  image_transport::ImageTransport it_;
  image_transport::Subscriber  img_sub_;
  ros::Subscriber              bbox_sub_;
  ros::Publisher               pub_;
  std::vector<cv::Rect>        bboxes_;
  int                          min_pixel_count_;
  cv::Scalar                   lo_red1_, hi_red1_, lo_red2_, hi_red2_;
  cv::Scalar                   lo_yellow_, hi_yellow_, lo_green_, hi_green_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_detector_node");
  ros::NodeHandle nh;
  ColorDetector cd(nh);
  ros::spin();
  cv::destroyAllWindows();
  return 0;
}
