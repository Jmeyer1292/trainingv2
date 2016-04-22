#include <ros/ros.h>

#include <myworkcell_core/LocalizePart.h>
#include <myworkcell_core/PlanCartesianPath.h>
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
    cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");

    pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);
  }

  geometry_msgs::Pose transformPose(const geometry_msgs::Pose& in) const
  {
    tf::Transform in_world;
    tf::poseMsgToTF(in, in_world);

    tf::Quaternion flip_z (tf::Vector3(1, 0, 0), M_PI);
    tf::Transform flip (flip_z);

    in_world = in_world * flip;

    geometry_msgs::Pose msg;
    tf::poseTFToMsg(in_world, msg);
    return msg;
  }

  void start()
  {
    ROS_INFO("Starting");
    // Localize the part
    myworkcell_core::LocalizePart srv;
    // if (!vision_client_.call(srv))
    // {
    //   ROS_ERROR("Could not localize part");
    //   return;
    // }
    ROS_INFO_STREAM("part localized: " << srv.response);

    srv.response.pose = transformPose(srv.response.pose);
    srv.response.pose.position.x = 0.3;
    srv.response.pose.position.z = 0.5;
    srv.response.pose.orientation.w = 1;

    // Plan for robot to move to part    
    // group_.setPoseTarget(srv.response.pose);
    // group_.move();

    // Plan cartesian path
    myworkcell_core::PlanCartesianPath cartesian_srv;
    cartesian_srv.request.pose = srv.response.pose;
    if (!cartesian_client_.call(cartesian_srv))
    {
      ROS_ERROR("Could not plan for path");
      return;
    }
    
    // Execute path
    pub_.publish(cartesian_srv.response.trajectory);
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
  ros::ServiceClient motion_client_;
  ros::ServiceClient cartesian_client_;
  // Action
  moveit::planning_interface::MoveGroup group_;
  // Motion?
  ros::Publisher pub_;
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
