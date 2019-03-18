#include <cmath>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "pose_test" );
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // create interface for motion planning
  static const std::string PLANNING_GROUP = "rivet_tool";
  moveit::planning_interface::MoveGroupInterface move_group ( PLANNING_GROUP );
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  ROS_INFO_NAMED( "motion_ctrl", "Reference frame: %s", move_group.getPlanningFrame().c_str() );
  ROS_INFO_NAMED( "motion_ctrl", "End effector link: %s", move_group.getEndEffectorLink().c_str() );

  ROS_INFO_STREAM ( "start for planning" );
  geometry_msgs::Pose target_pose1;
  float pose_array[] = {0.231569, -1.53348, 1.85431, 3.12539, 0.783333, 1.56717};
  target_pose1.position.x = pose_array[0];
  target_pose1.position.y = pose_array[1];
  target_pose1.position.z = pose_array[2];
  float rollt  = pose_array[3];
  float pitcht = pose_array[4];
  float yawt   = pose_array[5];
  target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
  move_group.setPoseTarget ( target_pose1 );

  move_group.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = ( move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
  ROS_INFO_NAMED( "motion_ctrl", "planning for goal pose is %s", success ? "success" : "FAILED" );
  if ( success )
  {
    move_group.setMaxVelocityScalingFactor ( 0.05 );
    move_group.move ();
  }

  ros::shutdown();
  return 0;
}
