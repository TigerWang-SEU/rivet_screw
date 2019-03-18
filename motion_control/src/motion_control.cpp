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
  ros::init ( argc, argv, "motion_control" );
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // create interface for motion planning
  static const std::string PLANNING_GROUP = "camera";
  moveit::planning_interface::MoveGroupInterface move_group ( PLANNING_GROUP );
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  ROS_INFO_NAMED( "motion_ctrl", "Reference frame: %s", move_group.getPlanningFrame().c_str() );
  ROS_INFO_NAMED( "motion_ctrl", "End effector link: %s", move_group.getEndEffectorLink().c_str() );

  // listen to the change of table coordinate
  tf::TransformListener listener;
  ros::Rate rate ( 1.0 );
  float x_o = 10000.0;
  float y_o = 10000.0;
  float angle_o = 10000.0;

  while ( ros::ok () )
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform ( "floor", "table", ros::Time ( 0 ), transform );
    }
    catch ( tf::TransformException ex )
    {
      ROS_ERROR ( "%s",ex.what() );
      ros::Duration( 1.0 ).sleep ();
      continue;
    }

    float x_n = transform.getOrigin().x();
    float y_n = transform.getOrigin().y();
    tf::Vector3 axis_n = transform.getRotation().getAxis();
    float angle_n = transform.getRotation().getAngle();

    ROS_INFO_STREAM ( "x_n: " << x_n << " y_n: " << y_n << " axis_n: [" << axis_n.getX() << ", " << axis_n.getY() << ", " << axis_n.getZ() << "], angle_n = " << angle_n );

    if ( std::abs( x_n - x_o ) > 0.03 || std::abs( y_n - y_o ) > 0.03 || std::abs( ( angle_n - angle_o ) / 3.1415 * 180.0 ) > 0.5 )
    {
      ROS_INFO_STREAM ( "start for planning" );
      geometry_msgs::Pose target_pose1;
      target_pose1.position.x = -0.270567446638;
      target_pose1.position.y = -1.163357894959;
      target_pose1.position.z = 1.46762397584;
      target_pose1.orientation.x = 0.429557969934;
      target_pose1.orientation.y = 0.424991905019;
      target_pose1.orientation.z = -0.557796409466;
      target_pose1.orientation.w = 0.568968361794;

      move_group.setPoseTarget(target_pose1);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = ( move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
      ROS_INFO_NAMED( "motion_ctrl", "planning for goal pose is %s", success ? "success" : "FAILED" );
      if ( success )
      {
        move_group.setMaxVelocityScalingFactor ( 0.1 );
        move_group.move ();
      }

      // start scanning the part
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back ( target_pose1 );

      geometry_msgs::Pose target_pose2 = target_pose1;
      target_pose2.position.z += 0.15;
      waypoints.push_back ( target_pose2 );

      geometry_msgs::Pose target_pose3 = target_pose2;
      target_pose3.position.z -= 0.15;
      waypoints.push_back ( target_pose3 );

      move_group.setMaxVelocityScalingFactor ( 0.1 );
      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group.computeCartesianPath ( waypoints, eef_step, jump_threshold, trajectory );
      ROS_INFO_NAMED ( "scan", "scan plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0 );

      robot_trajectory::RobotTrajectory rt( move_group.getCurrentState()->getRobotModel(), "camera" );
      rt.setRobotTrajectoryMsg( *move_group.getCurrentState(), trajectory );
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      success = iptp.computeTimeStamps(rt);
      if ( success )
      {
        rt.getRobotTrajectoryMsg( trajectory );
        my_plan.trajectory_ = trajectory;
        move_group.execute ( my_plan );
      }
    }

    x_o = x_n;
    y_o = y_n;
    angle_o = angle_n;
    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
