#include <ros/ros.h>

#include <myworkcell_core/LocalizePart.h>
#include <moveit/move_group_interface/move_group.h>

// Covers:
// 1. ROS Node Main
// 2. RoS-Service Client

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
    : group_("manipulator")
  {
    // Read parameters
    // Make service clients
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
  }

  void start()
  {
    ROS_INFO("Starting");
    // Localize the part
    myworkcell_core::LocalizePart srv;
    if (!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);

    // Transform math
    tf::Transform in_world;
    tf::poseMsgToTF(srv.response.pose, in_world);

    tf::Quaternion flip_z (tf::Vector3(1, 0, 0), M_PI);

    tf::Transform flip;
    flip.setIdentity();
    flip.setRotation(flip_z);

    in_world = in_world * flip;


    tf::poseTFToMsg(in_world, srv.response.pose);

    // Plan for robot to move to part
    
    srv.response.pose.position.x -= 0.4;
    group_.setPoseTarget(srv.response.pose);
    ROS_INFO_STREAM("Moving");
    group_.move();
    ROS_INFO("Done moving");
    // Plan cartesian path
    // Execute path
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
  ros::ServiceClient motion_client_;
  ros::ServiceClient cartesian_client_;
  // Action
  moveit::planning_interface::MoveGroup group_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "myworkcell_node");
  ros::NodeHandle nh;

  // Hello World
  ROS_INFO("Hello, World from a ROS Node");

  ScanNPlan app (nh);

  ros::Duration(.5).sleep();

  app.start();

  ros::spin();
}
