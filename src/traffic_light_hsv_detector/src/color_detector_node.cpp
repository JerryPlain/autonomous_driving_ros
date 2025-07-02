#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <traffic_light_hsv_detector/TrafficLightArray.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

class ColorDetectorNode
{
public:
  ColorDetectorNode(ros::NodeHandle& nh)
    : nh_(nh),
      sub_tl_(nh_,    "traffic_lights_rgb", 1),
      sub_rgb_(nh_,   "/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", 1)
  {
    typedef message_filters::sync_policies::ApproximateTime<
      traffic_light_hsv_detector::TrafficLightArray,
      sensor_msgs::Image
    > SyncPolicy;
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
      SyncPolicy(10), sub_tl_, sub_rgb_));
    sync_->registerCallback(
      boost::bind(&ColorDetectorNode::cb, this, _1, _2));

    pub_color_ = nh_.advertise<std_msgs::String>("traffic_light_color", 1);

    // Deine HSV-Bereiche
    lo_r1_ = cv::Scalar(  0,120,200); hi_r1_ = cv::Scalar( 10,255,255);
    lo_r2_ = cv::Scalar(170,120,200); hi_r2_ = cv::Scalar(179,255,255);
    lo_y_  = cv::Scalar( 25,140,200); hi_y_  = cv::Scalar( 35,225,255);
    lo_g_  = cv::Scalar( 40,130,200); hi_g_  = cv::Scalar( 80,245,255);

    ROS_INFO("ColorDetectorNode ready");
  }

private:
  void cb(
    const traffic_light_hsv_detector::TrafficLightArray::ConstPtr& tl_msg,
    const sensor_msgs::ImageConstPtr&                            img_msg)
  {
    // 1) BGR→HSV
    cv_bridge::CvImageConstPtr cvb;
    try {
      cvb = cv_bridge::toCvShare(img_msg, "bgr8");
    } catch (...) {
      ROS_ERROR("cv_bridge failed");
      return;
    }
    cv::Mat hsv;
    cv::cvtColor(cvb->image, hsv, cv::COLOR_BGR2HSV);

    // 2) Für jede Detektion
    for (auto& tl : tl_msg->lights)
    {
      int u = int(std::round(tl.centroid_pixel.x));
      int v = int(std::round(tl.centroid_pixel.y));

      // 3) ROI-Radius aus Distanz: r ≈ f·(H/2)/Z, hier als Konstante R=16px
      const int R = 16;
      cv::Rect roi(u-R, v-R, 2*R, 2*R);
      roi &= cv::Rect(0,0,hsv.cols,hsv.rows);
      if (roi.empty()) continue;

      cv::Mat patch = hsv(roi);

      // 4) Masken & Counts
      cv::Mat m1,m2, mr, my, mg;
      cv::inRange(patch, lo_r1_, hi_r1_, m1);
      cv::inRange(patch, lo_r2_, hi_r2_, m2);
      cv::bitwise_or(m1,m2, mr);
      cv::inRange(patch, lo_y_, hi_y_, my);
      cv::inRange(patch, lo_g_, hi_g_, mg);

      int cnt_r = cv::countNonZero(mr),
          cnt_y = cv::countNonZero(my),
          cnt_g = cv::countNonZero(mg);
      int total = cnt_r + cnt_y + cnt_g;

      // 5) Entscheidung
      std::string state = "UNKNOWN";
      float       conf  = 0.0f;
      if (total > 0) {
        if      (cnt_r >= cnt_y && cnt_r >= cnt_g) { state="RED";    conf = float(cnt_r)/total; }
        else if (cnt_y >= cnt_r && cnt_y >= cnt_g) { state="YELLOW"; conf = float(cnt_y)/total; }
        else if (cnt_g >= cnt_r && cnt_g >= cnt_y) { state="GREEN";  conf = float(cnt_g)/total; }
      }

      // 6) Publish als String "id:state:confidence"

      if (state != "UNKNOWN") {
        std_msgs::String out;
        std::ostringstream ss;
        ss << tl.id << ":" << state << ":" << conf;
        out.data = ss.str();
        pub_color_.publish(out);
      }
    }
  }

  ros::NodeHandle nh_;
  message_filters::Subscriber<traffic_light_hsv_detector::TrafficLightArray> sub_tl_;
  message_filters::Subscriber<sensor_msgs::Image> sub_rgb_;
  boost::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        traffic_light_hsv_detector::TrafficLightArray,
        sensor_msgs::Image
      >
    >
  > sync_;
  ros::Publisher pub_color_;

  cv::Scalar lo_r1_, hi_r1_, lo_r2_, hi_r2_, lo_y_, hi_y_, lo_g_, hi_g_;


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_detector_node");
  ros::NodeHandle nh;
  ColorDetectorNode node(nh);
  ros::spin();
  return 0;
}
