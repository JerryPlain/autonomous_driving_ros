// src/semantic_depth_node.cpp

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class SemanticDepthProcessor
{
public:
  SemanticDepthProcessor(ros::NodeHandle& nh)
    : nh_(nh)
  {
    // 1) Subscriber für Semantic- und Depth-Topic
    sub_sem_.subscribe(nh_, "/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1);
    sub_depth_.subscribe(nh_, "/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw",    1);

    // 2) Synchronizer mit ApproximateTime für 2 Topics
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      sensor_msgs::Image
    > SyncPolicy2;
    sync_.reset(
      new message_filters::Synchronizer<SyncPolicy2>(
        SyncPolicy2(10), sub_sem_, sub_depth_
      )
    );
    sync_->registerCallback(
      boost::bind(&SemanticDepthProcessor::callback, this, _1, _2)
    );

    // 3) Publisher fürs Debug-Depth (mit Bounding-Boxes)
    pub_debug_ = nh_.advertise<sensor_msgs::Image>("traffic_light/debug_depth", 1);

    ROS_INFO("semantic_depth_node initialized");
  }

private:
  void callback(
    const sensor_msgs::ImageConstPtr& sem_msg,
    const sensor_msgs::ImageConstPtr& depth_msg)
  {
    // --- A) Semantic → MONO8
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

    // --- B) Maske für Ampel-Klasse 215
    cv::Mat mask_sem = (sem_img == 215);

    // --- C) Tiefenbild als 16UC1 einlesen
    cv_bridge::CvImagePtr depth_raw;
    try {
      depth_raw = cv_bridge::toCvCopy(
        depth_msg, sensor_msgs::image_encodings::TYPE_16UC1
      );
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge depth exception: %s", e.what());
      return;
    }
    cv::Mat depth_16u = depth_raw->image;    // z.B. 640×480, Werte in mm

    // --- D) Maske auf Depth-Auflösung skalieren
    cv::Mat mask_depth;
    cv::resize(
      mask_sem, mask_depth,
      depth_16u.size(),
      0, 0, cv::INTER_NEAREST
    );

    // --- E) Tiefenbild → 32F in Meter
    cv::Mat depth_m;
    depth_16u.convertTo(depth_m, CV_32F, 1.0f/1000.0f);

    // --- F) Median-Berechnung im maskierten Bereich
    std::vector<float> depths;
    depths.reserve(depth_m.rows*depth_m.cols/10);
    for (int y = 0; y < depth_m.rows; ++y) {
      const float*  dptr = depth_m.ptr<float>(y);
      const uchar* mptr = mask_depth.ptr<uchar>(y);
      for (int x = 0; x < depth_m.cols; ++x) {
        if (mptr[x]) {
          float d = dptr[x];
          if (std::isfinite(d) && d > 0.0f)
            depths.push_back(d);
        }
      }
    }
    if (depths.empty()) {
      ROS_WARN_THROTTLE(1.0, "Keine gültigen Depth-Pixel in Ampel-Maske");
    } else {
      std::nth_element(
        depths.begin(),
        depths.begin() + depths.size()/2,
        depths.end()
      );
      float depth_med = depths[depths.size()/2];
      ROS_INFO("Ampel-Median-Depth = %.2f m", depth_med);
    }

    // --- G) (Optional) Kontrolle: Bounding-Boxes zeichnen
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(
      mask_sem, labels, stats, centroids, 8, CV_32S
    );
    // Kästchen im Depth-Bild einzeichnen
    for (int L = 1; L < nLabels; ++L) {
      int x = stats.at<int>(L, cv::CC_STAT_LEFT);
      int y = stats.at<int>(L, cv::CC_STAT_TOP);
      int w = stats.at<int>(L, cv::CC_STAT_WIDTH);
      int h = stats.at<int>(L, cv::CC_STAT_HEIGHT);

      // auf Depth-Skala umrechnen
      float scale_x = float(depth_m.cols) / float(sem_img.cols);
      float scale_y = float(depth_m.rows) / float(sem_img.rows);
      cv::Rect r;
      r.x      = int(x * scale_x);
      r.y      = int(y * scale_y);
      r.width  = int(w * scale_x);
      r.height = int(h * scale_y);

      cv::rectangle(depth_raw->image, r, cv::Scalar(65535), 2);
    }

    // --- H) Publish Debug-Depth
    pub_debug_.publish(depth_raw->toImageMsg());
  }

  ros::NodeHandle                                            nh_;
  message_filters::Subscriber<sensor_msgs::Image>            sub_sem_, sub_depth_;
  boost::shared_ptr<
    message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image
      >
    >
  > sync_;
  ros::Publisher                                             pub_debug_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_fusion");
  ros::NodeHandle nh;
  SemanticDepthProcessor proc(nh);
  ros::spin();
  return 0;
}
