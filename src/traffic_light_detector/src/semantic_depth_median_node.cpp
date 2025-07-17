#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <traffic_light_detector/TrafficLightBBox.h>
#include <traffic_light_detector/TrafficLightBBoxArray.h>
#include <sstream>
#include <iomanip>

using namespace traffic_light_detector;
namespace enc = sensor_msgs::image_encodings;

class SemanticDepthBBoxes
{
public:
  SemanticDepthBBoxes(ros::NodeHandle& nh)
    : nh_(nh)
  {
    // Subscribers
    sub_sem_.subscribe(nh_, "/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1);
    sub_depth_.subscribe(nh_, "/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image
    > SyncPolicy;
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), sub_sem_, sub_depth_));
    sync_->registerCallback(boost::bind(&SemanticDepthBBoxes::callback, this, _1, _2));

    // Publishers
    pub_bbox_  = nh_.advertise<TrafficLightBBoxArray>("traffic_lights_bbox", 1);
    pub_debug_ = nh_.advertise<sensor_msgs::Image>("traffic_lights_bbox_image", 1);

    ROS_INFO("semantic_depth_bboxes_node initialized");
  }

private:
  void callback(const sensor_msgs::ImageConstPtr& sem_msg,
                const sensor_msgs::ImageConstPtr& depth_msg)
  {
    // 1) Semantic -> MONO8
    cv_bridge::CvImageConstPtr sem_cv;
    try {
      sem_cv = cv_bridge::toCvShare(sem_msg, enc::MONO8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge semantic exception: %s", e.what());
      return;
    }
    cv::Mat sem = sem_cv->image;  // 320x240

    // 2) Mask class 215
    cv::Mat mask_sem = (sem == 215);

    // 3) Connected components
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(mask_sem, labels, stats, centroids);

    // 4) Depth conversion
    cv_bridge::CvImagePtr dcv;
    try {
      dcv = cv_bridge::toCvCopy(depth_msg, enc::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge depth exception: %s", e.what());
      return;
    }
    cv::Mat depth16 = dcv->image;             // 640x480, mm
    cv::Mat depthf; depth16.convertTo(depthf, CV_32F, 1.0/1000.0); // meters

    // 5) Prepare outputs and debug image
    TrafficLightBBoxArray arr;
    arr.header = sem_msg->header;

    // normalize depth for visualization
    cv::Mat depth8;
    double minVal, maxVal;
    cv::minMaxLoc(depth16, &minVal, &maxVal);
    depth16.convertTo(depth8, CV_8U, 255.0/(maxVal>0?maxVal:1));
    cv::Mat dbg; cv::cvtColor(depth8, dbg, cv::COLOR_GRAY2BGR);

    // scaling factors semantic->depth
    float sx = float(depth16.cols) / float(sem.cols);
    float sy = float(depth16.rows) / float(sem.rows);

    for (int lbl = 1; lbl < n; ++lbl) {
      int xs = stats.at<int>(lbl, cv::CC_STAT_LEFT);
      int ys = stats.at<int>(lbl, cv::CC_STAT_TOP);
      int ws = stats.at<int>(lbl, cv::CC_STAT_WIDTH);
      int hs = stats.at<int>(lbl, cv::CC_STAT_HEIGHT);
      // scale to depth size
      int x = int(xs * sx);
      int y = int(ys * sy);
      int w = int(ws * sx);
      int h = int(hs * sy);
      cv::Rect r(x,y,w,h);
      r &= cv::Rect(0,0,depthf.cols, depthf.rows);
      if (r.empty()) continue;

      // collect depth values
      std::vector<float> vals;
      for (int yy = r.y; yy < r.y + r.height; ++yy) {
        const float* dp = depthf.ptr<float>(yy);
        for (int xx = r.x; xx < r.x + r.width; ++xx) {
          float d = dp[xx];
          if (std::isfinite(d) && d > 0.0f)
            vals.push_back(d);
        }
      }
      if (vals.empty()) continue;
      std::nth_element(vals.begin(), vals.begin()+vals.size()/2, vals.end());
      float med = vals[vals.size()/2];

      // fill bbox message
      TrafficLightBBox tl;
      tl.header   = sem_msg->header;
      tl.id       = lbl;
      tl.x        = x;
      tl.y        = y;
      tl.width    = w;
      tl.height   = h;
      tl.distance = med;
      arr.boxes.push_back(tl);

      // draw on debug image
      cv::rectangle(dbg, r, cv::Scalar(0,255,0), 2);
      std::ostringstream txt;
      txt << "ID:" << lbl << " d=" << std::fixed << std::setprecision(2) << med;
      cv::putText(dbg, txt.str(), cv::Point(x, y-5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0,255,0), 1);
    }

    // publish
    pub_bbox_.publish(arr);
    cv_bridge::CvImage out;
    out.header   = sem_msg->header;
    out.encoding = enc::BGR8;
    out.image    = dbg;
    pub_debug_.publish(out.toImageMsg());
  }

  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> sub_sem_, sub_depth_;
  boost::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
  >> sync_;
  ros::Publisher pub_bbox_, pub_debug_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "semantic_depth_bboxes_node");
  ros::NodeHandle nh;
  SemanticDepthBBoxes node(nh);
  ros::spin();
  return 0;
}
