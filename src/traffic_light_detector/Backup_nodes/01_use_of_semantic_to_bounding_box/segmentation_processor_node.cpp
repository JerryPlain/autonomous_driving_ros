#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <traffic_light_hsv_detector/BoundingBoxes.h>

class SegmentationProcessor
{
public:
  SegmentationProcessor(ros::NodeHandle& nh)
  {
    sub_ = nh.subscribe(
      "/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw",
      1,
      &SegmentationProcessor::imageCallback,
      this
    );
    pub_ = nh.advertise<traffic_light_hsv_detector::BoundingBoxes>(
      "traffic_light/bounding_boxes", 1
    );
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    // 1) in MONO8 umwandeln
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(
        msg, sensor_msgs::image_encodings::MONO8
      );
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    const cv::Mat& img = cv_ptr->image;
    //ROS_INFO("Image size: %d x %d, channels: %d",
    //         img.cols, img.rows, img.channels());

    // 2) Ziel-Klassen (z.B. Auto=128, Ampel-Gelb=215)
    std::vector<int> targetClasses = {215};

    // 3) Message vorbereiten
    traffic_light_hsv_detector::BoundingBoxes out;
    out.header = msg->header;

    // 4) Pro Klasse Connected Components
    for (int cls : targetClasses)
    {
      cv::Mat mask = (img == cls);
      cv::Mat labels, stats, centroids;
      int nLabels = cv::connectedComponentsWithStats(
        mask, labels, stats, centroids, 8, CV_32S
      );

      ROS_INFO("Klasse %d: %d Objekte gefunden", cls, nLabels-1);

      for (int label = 1; label < nLabels; ++label)
      {
        int x = stats.at<int>(label, cv::CC_STAT_LEFT);
        int y = stats.at<int>(label, cv::CC_STAT_TOP);
        int w = stats.at<int>(label, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(label, cv::CC_STAT_HEIGHT);

        ROS_INFO("  [Klasse %d | Obj %d] TL=(%d,%d) BR=(%d,%d)  Größe:%dx%d",
                 cls, label, x, y, x+w-1, y+h-1, w, h);

        traffic_light_hsv_detector::BoundingBox bb;
        bb.x      = x;
        bb.y      = y;
        bb.width  = w;
        bb.height = h;
        out.boxes.push_back(bb);
      }
    }

    // 5) publish
    pub_.publish(out);
  }

  ros::Subscriber sub_;
  ros::Publisher  pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "segmentation_processor_node");
  ros::NodeHandle nh;
  SegmentationProcessor proc(nh);
  ros::spin();
  return 0;
}
