#include <ros/ros.h>
#include "myworkcell_core/PlanCartesianPath.h"

#include <descartes_moveit/moveit_state_adapter.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_utilities/ros_conversions.h>
#include <eigen_conversions/eigen_msg.h>

class CartesianPlanner
{
public:
  CartesianPlanner(ros::NodeHandle& nh)
  {
    model_ = boost::make_shared<descartes_moveit::MoveitStateAdapter>();

    // first init descartes
    const std::string robot_description = "robot_description";
    const std::string group_name = "manipulator";
    const std::string world_frame = "base_link"; // Frame in which tool poses are expressed
    const std::string tcp_frame = "tool0";

    if (!model_->initialize(robot_description, group_name, world_frame, tcp_frame))
    {
      ROS_WARN("model didn't init");
      throw std::runtime_error("model init()");
    }

    if (!planner_.initialize(model_))
    {
      ROS_WARN("Planner didn't init");
    }

    // init services
    server_ = nh.advertiseService("plan_path", &CartesianPlanner::planPath, this);
  }

  bool planPath(myworkcell_core::PlanCartesianPathRequest& req,
                myworkcell_core::PlanCartesianPathResponse& res)
  {
    ROS_INFO("Planning");
    // Step 1: Generate path poses
    EigenSTL::vector_Affine3d tool_poses = makeToolPoses();
    // Step 2: Translate that path by the input reference pose
    std::vector<descartes_core::TrajectoryPtPtr> path = makeDescartesTrajectorty(req.pose, tool_poses);
    ROS_INFO("plan path");
    // Step 3: Plan with descartes
    if (!planner_.planPath(path))
    {
      return false;
    }
    ROS_INFO("Get path");
    std::vector<descartes_core::TrajectoryPtPtr> result;
    if (!planner_.getPath(result))
    {
      return false;
    }

    // Step 4: Put the output back into the response
    std::vector<std::string> names;
    ros::NodeHandle nh;
    nh.getParam("controller_joint_names", names);
    res.trajectory.header.stamp = ros::Time::now();
    res.trajectory.header.frame_id = "world";
    res.trajectory.joint_names = names;
    descartes_utilities::toRosJointPoints(*model_, result, 1.0, res.trajectory.points);
    return true;
  }

  EigenSTL::vector_Affine3d makeToolPoses()
  {
    EigenSTL::vector_Affine3d path;
    // Make line
    for (int i = 0; i < 5; ++i)
    {
      Eigen::Affine3d pose;
      pose = Eigen::Translation3d(0, 0.05 * i, 0.02 * i);
      path.push_back(pose);
    }
    return path;
  }

  std::vector<descartes_core::TrajectoryPtPtr>
  makeDescartesTrajectorty(const geometry_msgs::Pose& reference,
                           const EigenSTL::vector_Affine3d& path)
  {
    std::vector<descartes_core::TrajectoryPtPtr> descartes_path; // return value

    Eigen::Affine3d ref;
    tf::poseMsgToEigen(reference, ref);

    for (auto& point : path)
    {
      auto pt = makeTolerancedCartesianPoint(ref * point);
      descartes_path.push_back(pt);
    }
    return descartes_path;
  }

  descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
  {
    using namespace descartes_core;
    using namespace descartes_trajectory;
    return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0, AxialSymmetricPt::Z_AXIS) );
  }


  boost::shared_ptr<descartes_moveit::MoveitStateAdapter> model_;
  descartes_planner::DensePlanner planner_;
  ros::ServiceServer server_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "descartes_node");

  ros::NodeHandle nh;
  CartesianPlanner planner (nh);

  ROS_INFO("Cartesian planning node starting");
  ros::spin();
}
