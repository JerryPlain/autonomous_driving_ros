// src/semantic_depth_median_node.cpp

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <traffic_light_hsv_detector/TrafficLight.h>
#include <traffic_light_hsv_detector/TrafficLightArray.h>
#include <geometry_msgs/Point.h>
#include <algorithm>
#include <vector>

class SemanticDepthMedianNode
{
public:
  SemanticDepthMedianNode(ros::NodeHandle& nh)
    : nh_(nh)
  {
    // Subscriber für Semantic- und Depth-Topic
    sub_sem_.subscribe(nh_,
      "/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1);
    sub_depth_.subscribe(nh_,
      "/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", 1);

    // Synchronizer ApproximateTime
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image
    > SyncPolicy;
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
      SyncPolicy(10), sub_sem_, sub_depth_));
    sync_->registerCallback(
      boost::bind(&SemanticDepthMedianNode::callback, this, _1, _2)
    );

    // Publisher für TrafficLightArray
    pub_ = nh_.advertise<traffic_light_hsv_detector::TrafficLightArray>(
      "traffic_lights", 1);

    ROS_INFO("semantic_depth_median_node initialized");
  }

private:
  void callback(
    const sensor_msgs::ImageConstPtr& sem_msg,
    const sensor_msgs::ImageConstPtr& depth_msg)
  {
    // A) Semantic → MONO8
    cv_bridge::CvImageConstPtr sem_cv;
    try {
      sem_cv = cv_bridge::toCvShare(
        sem_msg, sensor_msgs::image_encodings::MONO8
      );
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge semantic exception: %s", e.what());
      return;
    }
    const cv::Mat& sem_img = sem_cv->image;  // z.B. 320×240

    // B) Maske für Klasse 215 (Ampel)
    cv::Mat mask_sem = (sem_img == 215);

    // Connected Components im Semantic-Bild
    cv::Mat labels;
    int nLabels = cv::connectedComponents(
      mask_sem, labels, 8, CV_32S);

    // C) Depth → TYPE_16UC1
    cv_bridge::CvImagePtr depth_cv;
    try {
      depth_cv = cv_bridge::toCvCopy(
        depth_msg, sensor_msgs::image_encodings::TYPE_16UC1
      );
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge depth exception: %s", e.what());
      return;
    }
    cv::Mat depth_16u = depth_cv->image; // z.B. 640×480, Werte in mm

    // D) Ausgabe-Message vorbereiten
    traffic_light_hsv_detector::TrafficLightArray out_msg;
    out_msg.header = sem_msg->header;

    // E) Pro Connected Component
    for (int lbl = 1; lbl < nLabels; ++lbl)
    {
      // 1) Maske der aktuellen Komponente
      cv::Mat mask_label = (labels == lbl);

      // 2) Pixel-Centroid im Semantic-Bild
      cv::Moments m = cv::moments(mask_label, true);
      if (m.m00 == 0) continue;
      float cx = static_cast<float>(m.m10 / m.m00);
      float cy = static_cast<float>(m.m01 / m.m00);

      // 3) Maske auf Depth-Auflösung skalieren
      cv::Mat mask_depth;
      cv::resize(
        mask_label, mask_depth,
        depth_16u.size(),
        0, 0, cv::INTER_NEAREST
      );

      // 4) Tiefenbild in Meter umwandeln
      cv::Mat depth_m;
      depth_16u.convertTo(depth_m, CV_32F, 1.0f/1000.0f);

      // 5) Tiefenwerte sammeln und Median berechnen
      std::vector<float> depths;
      for (int y = 0; y < depth_m.rows; ++y) {
        const float* dptr = depth_m.ptr<float>(y);
        const uchar* mptr = mask_depth.ptr<uchar>(y);
        for (int x = 0; x < depth_m.cols; ++x) {
          if (mptr[x]) {
            float d = dptr[x];
            if (std::isfinite(d) && d > 0.0f)
              depths.push_back(d);
          }
        }
      }
      if (depths.empty()) continue;
      std::nth_element(
        depths.begin(),
        depths.begin() + depths.size()/2,
        depths.end()
      );
      float depth_med = depths[depths.size()/2];

      // 6) TrafficLight-Message füllen
      traffic_light_hsv_detector::TrafficLight tl;
      tl.header = sem_msg->header;
      tl.id     = lbl;
      tl.centroid_pixel.x = cx;
      tl.centroid_pixel.y = cy;
      tl.centroid_pixel.z = 0.0;
      tl.distance = depth_med;

      out_msg.lights.push_back(tl);
    }

    // F) Publizieren, wenn mindestens eine Ampel gefunden
    if (!out_msg.lights.empty()) {
      pub_.publish(out_msg);
    }
  }

  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> sub_sem_, sub_depth_;
  boost::shared_ptr<
    message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image
      >
    >
  > sync_;
  ros::Publisher pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "semantic_depth_median_node");
  ros::NodeHandle nh;
  SemanticDepthMedianNode node(nh);
  ros::spin();
  return 0;
}
