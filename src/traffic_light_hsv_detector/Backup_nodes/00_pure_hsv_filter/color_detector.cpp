#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class ColorDetector {
public:
  ColorDetector(ros::NodeHandle& nh)
    : it_(nh),
      pub_(nh.advertise<std_msgs::String>("traffic_light/color", 1))
  {
    // minimale Blob-Größe (in Pixeln)
    min_pixel_count_ = 1;

    // HSV-Ranges (OpenCV Hue 0–179) für Rot, Gelb, Grün
    lo_red1_   = cv::Scalar(  0, 120, 200);
    hi_red1_   = cv::Scalar( 10, 255, 255);
    lo_red2_   = cv::Scalar(170, 120, 200);
    hi_red2_   = cv::Scalar(179, 255, 255);
    // gemessen: h30, s156-209, v255
    lo_yellow_ = cv::Scalar( 25, 140, 200);
    hi_yellow_ = cv::Scalar( 35, 225, 255);
    // gemessen: h54-62, s156-219, v255
    lo_green_  = cv::Scalar( 40, 130, 200);
    hi_green_  = cv::Scalar( 80, 245, 255);

    // Subscriber — Topic kann per <remap> im Launchfile angepasst werden
    sub_ = it_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", 1, &ColorDetector::imageCb, this);

    // Debug-Fenster
//    cv::namedWindow("mask_red",    cv::WINDOW_NORMAL);
//    cv::namedWindow("mask_yellow", cv::WINDOW_NORMAL);
//    cv::namedWindow("mask_green",  cv::WINDOW_NORMAL);
//    cv::namedWindow("chosen_blob", cv::WINDOW_NORMAL);

    ROS_INFO("ColorDetector gestartet, warte auf Bilder …");
  }
  struct Blob {
    float area;
    cv::Rect bbox;
  };
void imageCb(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO_ONCE("▶ imageCb() bekommt Frames!");

  // 1) BGR → HSV
  cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  // 2) Masken erzeugen
  cv::Mat m_red1, m_red2, mask_red, mask_yellow, mask_green;
  cv::inRange(hsv, lo_red1_, hi_red1_, m_red1);
  cv::inRange(hsv, lo_red2_, hi_red2_, m_red2);
  cv::bitwise_or(m_red1, m_red2, mask_red);

  cv::inRange(hsv, lo_yellow_, hi_yellow_, mask_yellow);
  cv::inRange(hsv, lo_green_,  hi_green_,  mask_green);

  // 3) Rauschen entfernen
  cv::Mat elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2));
  cv::morphologyEx(mask_red,    mask_red,    cv::MORPH_OPEN, elem);
  cv::morphologyEx(mask_yellow, mask_yellow, cv::MORPH_OPEN, elem);
  cv::morphologyEx(mask_green,  mask_green,  cv::MORPH_OPEN, elem);

  // 4) Debug: Alle Masken anzeigen
  cv::imshow("mask_red",    mask_red);
  cv::imshow("mask_yellow", mask_yellow);
  cv::imshow("mask_green",  mask_green);
  cv::waitKey(1);

  // 5) Blobs sammeln
  struct Blob { float area; int color; std::vector<cv::Point> cnt; };
  std::vector<Blob> blobs;
  auto collect = [&](const cv::Mat& mask, int color){
    std::vector<std::vector<cv::Point>> cnts;
    cv::findContours(mask, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto& c : cnts) {
      float a = cv::contourArea(c);
      if (a >= min_pixel_count_) {
        blobs.push_back({a, color, c});
      }
    }
  };
  collect(mask_red,    0);
  collect(mask_yellow, 1);
  collect(mask_green,  2);

  if (blobs.empty()) {
    ROS_INFO("Keine Ampel-Blobs gefunden");
    return;
  }

  // 6) Kleinstes Blob finden (am weitesten entfernt)
  auto best = std::min_element(blobs.begin(), blobs.end(),
    [](const Blob& A, const Blob& B){ return A.area < B.area; });

  static const char* names[] = {"RED", "YELLOW", "GREEN"};
  ROS_INFO("→ Ausgewählte Ampel: %s (Fläche=%.1f px)",
           names[best->color], best->area);

  // 7) Debug: gewähltes Blob einzeichnen
  cv::Mat dbg = bgr.clone();
  cv::Rect r = cv::boundingRect(best->cnt);
  cv::rectangle(dbg, r, cv::Scalar(255,0,0), 2);
  cv::imshow("chosen_blob", dbg);
  cv::waitKey(1);

  // 8) Publish
  std_msgs::String out;
  out.data = std::string(names[best->color]) + " traffic light";
  pub_.publish(out);
}



private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber    sub_;
  ros::Publisher                 pub_;
  cv::Scalar lo_red1_, hi_red1_, lo_red2_, hi_red2_;
  cv::Scalar lo_yellow_, hi_yellow_, lo_green_, hi_green_;
  int        min_pixel_count_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "color_detector_cpp");
  ros::NodeHandle nh;
  ColorDetector cd(nh);
  ros::spin();
  cv::destroyAllWindows();
  return 0;
}

