#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <traffic_light_hsv_detector/BoundingBoxes.h>

class SegmentationProcessor
{
public:
  SegmentationProcessor(ros::NodeHandle& nh)
  {
    sub_sem_ = nh.subscribe(
      "/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw",
      1,
      &SegmentationProcessor::imageCallback,
      this
    );
    pub_boxes_  = nh.advertise<traffic_light_hsv_detector::BoundingBoxes>(
      "traffic_light/bounding_boxes", 1
    );
    pub_mask_   = nh.advertise<sensor_msgs::Image>(
      "traffic_light/mask", 1
    );
    pub_debug_  = nh.advertise<sensor_msgs::Image>(
      "traffic_light/debug_image", 1
    );
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& sem_msg)
  {
    // 1) Semantic image in MONO8
    cv_bridge::CvImageConstPtr cv_sem;
    try {
      cv_sem = cv_bridge::toCvShare(
        sem_msg, sensor_msgs::image_encodings::MONO8
      );
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    const cv::Mat& sem = cv_sem->image;

    // 2) Maske für Ampel-Klasse 215
    const int TL_CLASS = 215;
    cv::Mat mask = (sem == TL_CLASS);

    // Publish reine Maske
    {
      cv_bridge::CvImage m;
      m.header   = sem_msg->header;
      m.encoding = sensor_msgs::image_encodings::MONO8;
      m.image    = mask;
      pub_mask_.publish(m.toImageMsg());
    }

    // 3) Connected Components für einzelne Objekte
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(
      mask, labels, stats, centroids, 8, CV_32S
    );

    traffic_light_hsv_detector::BoundingBoxes boxes_msg;
    boxes_msg.header = sem_msg->header;

    // 4) Debug-Image: wir wandeln das Graustufen-Semantikbild in BGR um
    cv::Mat debug_bgr;
    cv::cvtColor(sem, debug_bgr, cv::COLOR_GRAY2BGR);

    for (int lab = 1; lab < nLabels; ++lab)
    {
      int x = stats.at<int>(lab, cv::CC_STAT_LEFT);
      int y = stats.at<int>(lab, cv::CC_STAT_TOP);
      int w = stats.at<int>(lab, cv::CC_STAT_WIDTH);
      int h = stats.at<int>(lab, cv::CC_STAT_HEIGHT);

      // BoundingBox in Message
      traffic_light_hsv_detector::BoundingBox bb;
      bb.x      = x;
      bb.y      = y;
      bb.width  = w;
      bb.height = h;
      boxes_msg.boxes.push_back(bb);

      // Rectangle auf Debug-Bild
      cv::rectangle(
        debug_bgr,
        cv::Point(x, y),
        cv::Point(x + w - 1, y + h - 1),
        cv::Scalar(0, 255, 0), 2
      );
    }

    // 5) Publish BoundingBoxes und Debug-Image
    pub_boxes_.publish(boxes_msg);

    cv_bridge::CvImage dbg;
    dbg.header   = sem_msg->header;
    dbg.encoding = sensor_msgs::image_encodings::BGR8;
    dbg.image    = debug_bgr;
    pub_debug_.publish(dbg.toImageMsg());
  }

  ros::Subscriber             sub_sem_;
  ros::Publisher              pub_boxes_, pub_mask_, pub_debug_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "segmentation_processor_node");
  ros::NodeHandle nh;
  SegmentationProcessor proc(nh);
  ros::spin();
  return 0;
}
