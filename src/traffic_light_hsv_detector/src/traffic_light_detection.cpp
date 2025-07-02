// src/traffic_light_detection.cpp

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <traffic_light_hsv_detector/TrafficLightArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class TrafficLightDetectionNode
{
public:
  TrafficLightDetectionNode(ros::NodeHandle& nh)
    : nh_(nh),
      sub_tl_(nh_, "traffic_lights", 1),
      sub_rgb_(nh_, "/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", 1)
  {
    typedef message_filters::sync_policies::ApproximateTime<
      traffic_light_hsv_detector::TrafficLightArray,
      sensor_msgs::Image
    > SyncPolicy;

    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
      SyncPolicy(10), sub_tl_, sub_rgb_));
    sync_->registerCallback(
      boost::bind(&TrafficLightDetectionNode::callback, this, _1, _2));

    pub_tl_rgb_ = nh_.advertise<traffic_light_hsv_detector::TrafficLightArray>(
      "traffic_lights_rgb", 1);

    ROS_INFO("traffic_light_detection node initialized (publishes traffic_lights_rgb)");
  }

private:
  void callback(
    const traffic_light_hsv_detector::TrafficLightArray::ConstPtr& in_msg,
    const sensor_msgs::ImageConstPtr& rgb_msg)
  {
    // Neues Array für RGB-Projektion
    traffic_light_hsv_detector::TrafficLightArray out_msg;
    out_msg.header = rgb_msg->header;

    // Kameraparameter (Depth und RGB Intrinsics identisch bis auf Translation)
    const float fx = 240.0f, fy = 240.0f;
    const float cx = 320.0f, cy = 240.0f;
    const float baseline = 0.20f;  // 20 cm

    // Für jede Ampel im eingehenden Array
    for (const auto& tl : in_msg->lights)
    {
      // 1) Semantic → Depth-Auflösung (320×240 → 640×480)
      float u_d = tl.centroid_pixel.x * 2.0f;
      float v_d = tl.centroid_pixel.y * 2.0f;

      // 2) 3D-Rekonstruktion im Depth-Frame
      float Z = tl.distance;
      float X = (u_d - cx) * Z / fx;
      float Y = (v_d - cy) * Z / fy;

      // 3) Verschiebung in RGB-Frame (linke Kamera um baseline nach links)
      float X_r = X + baseline;

      // 4) Projektion ins RGB-Bild
      float u_r = fx * X_r / Z + cx;
      float v_r = fy * Y   / Z + cy;

      // 5) Neues TrafficLight-Element füllen
      traffic_light_hsv_detector::TrafficLight tl_rgb;
      tl_rgb.header        = rgb_msg->header;
      tl_rgb.id            = tl.id;
      tl_rgb.centroid_pixel.x = u_r;
      tl_rgb.centroid_pixel.y = v_r;
      tl_rgb.centroid_pixel.z = 0.0f;
      tl_rgb.distance      = Z;

      out_msg.lights.push_back(tl_rgb);
    }

    // 6) Publizieren
    pub_tl_rgb_.publish(out_msg);
  }

  ros::NodeHandle nh_;

  // Input
  message_filters::Subscriber<traffic_light_hsv_detector::TrafficLightArray> sub_tl_;
  message_filters::Subscriber<sensor_msgs::Image>                           sub_rgb_;
  boost::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        traffic_light_hsv_detector::TrafficLightArray,
        sensor_msgs::Image
      >
    >
  > sync_;

  // Output
  ros::Publisher pub_tl_rgb_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traffic_light_detection");
  ros::NodeHandle nh;
  TrafficLightDetectionNode node(nh);
  ros::spin();
  return 0;
}
