// Covers Topics/Messages
// TF publishing
// Service Server

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/transform_listener.h>
#include <ar_pose/ARMarker.h>

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
    ar_sub_ = nh.subscribe<ar_pose::ARMarker>("ar_pose_marker", 1, 
      &Localizer::visionCallback, this);

    server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
  }

  bool localizePart(myworkcell_core::LocalizePart::Request& req,
                    myworkcell_core::LocalizePart::Response& res)
  {
    // Read last message
    ar_pose::ARMarkerConstPtr p = last_msg_;  
    if (!p) return false;
    // Use TF to look up transform between request base frame and the target
    tf::StampedTransform transform;
    listener_.lookupTransform("world", "ar_marker", ros::Time(0), transform);
    tf::poseTFToMsg(transform, res.pose);
    return true;
  }

  void visionCallback(const ar_pose::ARMarkerConstPtr& msg)
  {
    last_msg_ = msg;
  }
  
  tf::TransformListener listener_;
  ros::Subscriber ar_sub_;
  ros::ServiceServer server_;
  ar_pose::ARMarkerConstPtr last_msg_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  Localizer localizer (nh);

  ROS_INFO("Vision node starting");
  ros::spin();
}
