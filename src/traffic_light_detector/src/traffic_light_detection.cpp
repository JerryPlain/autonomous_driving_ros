#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <traffic_light_detector/TrafficLightBBox.h>
#include <traffic_light_detector/TrafficLightBBoxArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iomanip>

class TrafficLightDetectionDebug
{
public:
  TrafficLightDetectionDebug(ros::NodeHandle& nh)
    : nh_(nh),
      sub_bbox_(nh_, "traffic_lights_bbox", 1),
      sub_rgb_(nh_, "/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", 1)
  {
    typedef message_filters::sync_policies::ApproximateTime<
      traffic_light_detector::TrafficLightBBoxArray,
      sensor_msgs::Image
    > SyncPolicy;

    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), sub_bbox_, sub_rgb_));
    sync_->registerCallback(boost::bind(&TrafficLightDetectionDebug::callback, this, _1, _2));

    pub_bbox_rgb_ = nh_.advertise<traffic_light_detector::TrafficLightBBoxArray>(
      "traffic_lights_bbox_rgb", 1);

    ROS_INFO("TrafficLightDetectionDebug initialized");
  }

private:
  void callback(
    const traffic_light_detector::TrafficLightBBoxArray::ConstPtr& in_msg,
    const sensor_msgs::ImageConstPtr&                                 rgb_msg)
  {
    // convert image
    cv_bridge::CvImageConstPtr cvb;
    try {
      cvb = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge rgb exception: %s", e.what());
      return;
    }
    cv::Mat img = cvb->image.clone();

    // stereo intrinsics (shared) and baseline (m)
    const float fx = 240.0f, fy = 240.0f;
    const float cx = 320.0f, cy = 240.0f;
    const float baseline = 0.20f;

    // prepare output array
    traffic_light_detector::TrafficLightBBoxArray out_msg;
    out_msg.header = rgb_msg->header;

    // project each depth-box to RGB frame
    for (const auto& box : in_msg->boxes)
    {
      float Z = box.distance;
      // top-left corner
      float u1 = box.x;
      float v1 = box.y;
      // bottom-right corner
      float u2 = box.x + box.width;
      float v2 = box.y + box.height;

      // back-project to camera coordinates (depth frame)
      float X1 = (u1 - cx) * Z / fx;
      float Y1 = (v1 - cy) * Z / fy;
      float X2 = (u2 - cx) * Z / fx;
      float Y2 = (v2 - cy) * Z / fy;

      // account for left camera baseline translation
      float X1_r = X1 + baseline;
      float X2_r = X2 + baseline;

      // project into RGB pixel coordinates
      float ur1 = fx * X1_r / Z + cx;
      float ur2 = fx * X2_r / Z + cx;
      float vr1 = fy * Y1   / Z + cy;
      float vr2 = fy * Y2   / Z + cy;

      // build new box
      int xr = int(std::round(ur1));
      int yr = int(std::round(vr1));
      int wr = int(std::round(ur2 - ur1));
      int hr = int(std::round(vr2 - vr1));
      cv::Rect r_rgb(xr, yr, wr, hr);
      r_rgb &= cv::Rect(0,0, img.cols, img.rows);
      if (r_rgb.empty()) continue;

      // draw rectangle and id/distance
      cv::rectangle(img, r_rgb, cv::Scalar(0,255,0), 2);
      std::ostringstream ss;
      ss << box.id << ":" << std::fixed << std::setprecision(2) << Z << "m";
      cv::putText(img, ss.str(), cv::Point(r_rgb.x, r_rgb.y - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);

      // append to out_msg
      traffic_light_detector::TrafficLightBBox tb;
      tb = box;
      tb.x = r_rgb.x;
      tb.y = r_rgb.y;
      tb.width = r_rgb.width;
      tb.height = r_rgb.height;
      out_msg.boxes.push_back(tb);
    }

    // publish converted bboxes
    pub_bbox_rgb_.publish(out_msg);
  }

  ros::NodeHandle nh_;
  message_filters::Subscriber<traffic_light_detector::TrafficLightBBoxArray> sub_bbox_;
  message_filters::Subscriber<sensor_msgs::Image>                                 sub_rgb_;
  boost::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      traffic_light_detector::TrafficLightBBoxArray,
      sensor_msgs::Image
    >
  >> sync_;
  ros::Publisher pub_bbox_rgb_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traffic_light_detection_debug");
  ros::NodeHandle nh;
  TrafficLightDetectionDebug node(nh);
  ros::spin();
  return 0;
}
