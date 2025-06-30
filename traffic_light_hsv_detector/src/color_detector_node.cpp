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
    lo_red1_   = cv::Scalar(  0, 163, 227);
    hi_red1_   = cv::Scalar( 5, 209, 255);
    lo_red2_   = cv::Scalar(178, 163, 227);
    hi_red2_   = cv::Scalar(180, 209, 255);
    // // gemessen: h30, s156-209, v255
    // lo_yellow_ = cv::Scalar( 25, 140, 200);
    // hi_yellow_ = cv::Scalar( 35, 225, 255);
    // // gemessen: h54-62, s156-219, v255
    // lo_green_  = cv::Scalar( 40, 130, 200);
    // hi_green_  = cv::Scalar( 80, 245, 255);
    lo_yellow_ = cv::Scalar( 30, 153, 255);
    hi_yellow_ = cv::Scalar( 30, 222, 255);
    lo_green_  = cv::Scalar( 54, 156, 255);
    hi_green_  = cv::Scalar( 61, 204, 255);

    // Subscriber — Topic kann per <remap> im Launchfile angepasst werden
    sub_ = it_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", 1, &ColorDetector::imageCb, this);
  }
  struct Blob {
    float area;
    cv::Rect bbox;
  };
void imageCb(const sensor_msgs::ImageConstPtr& msg) {
  // 1) BGR → HSV
  cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  // ── NEU: ROI definieren ───────────────────────────────────────────
  int H = hsv.rows; 
  int W = hsv.cols;
  // Obere Hälfte:
  int y0 = 0, h = H/2;
  // Mittleres Drittel horizontal:
  int x0 = W/3, w = W/3;
  cv::Rect roi_rect(x0, y0, w, h);
  cv::Mat hsv_roi = hsv(roi_rect);
  // 2) Maske auf hsv_roi
  cv::Mat m1, m2, mask_red;
  cv::inRange(hsv_roi, lo_red1_, hi_red1_, m1);
  cv::inRange(hsv_roi, lo_red2_, hi_red2_, m2);
  cv::bitwise_or(m1, m2, mask_red);

  // 3) Rauschen
  cv::Mat elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, {2,2});
  cv::morphologyEx(mask_red, mask_red, cv::MORPH_OPEN, elem);

  // 5) Blobs sammeln (Area & Contour)
  struct Blob {
    float area;
    int   color;  // 0=RED
    std::vector<cv::Point> cnt;
  };
  std::vector<Blob> blobs;
  {
    std::vector<std::vector<cv::Point>> cnts;
    cv::findContours(mask_red, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto& c : cnts) {
      float a = cv::contourArea(c);
      if (a < min_pixel_count_) continue;
      blobs.push_back({a, 0, c});
    }
  }

  if (blobs.empty()) {
    // nichts gefunden
    return;
  }

  // 6) Auswahl: genau 1 → dieser, sonst größter Bereich
  Blob* best = nullptr;
  if (blobs.size() == 1) {
    best = &blobs[0];
  } else {
    best = & *std::max_element(
      blobs.begin(), blobs.end(),
      [](const Blob& A, const Blob& B){ return A.area < B.area; }
    );
  }

  // 7) Bounding-Box ins Originalbild zurückprojizieren
  cv::Rect rel = cv::boundingRect(best->cnt);
  cv::Rect abs = {
    rel.x + roi_rect.x,
    rel.y + roi_rect.y,
    rel.width,
    rel.height
  };

  // 8) Debug: Nur chosen_blob anzeigen
  cv::Mat dbg = bgr.clone();
  cv::rectangle(dbg, abs, cv::Scalar(255,0,0), 2);
  cv::imshow("chosen_blob", dbg);
  cv::waitKey(1);

  // 9) Publish
  std_msgs::String out;
  out.data = "RED traffic light";  // wenn nur rot aktiv ist
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
  ros::init(argc, argv, "color_detector_node");
  ros::NodeHandle nh;
  // Fenster fürs Debug-Output
  cv::namedWindow("chosen_blob", cv::WINDOW_NORMAL);
  cv::startWindowThread();

  ColorDetector cd(nh);
  ros::spin();
  cv::destroyAllWindows();
  return 0;
}
