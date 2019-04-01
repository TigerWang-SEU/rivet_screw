#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <string>

#include <ros/package.h>
#include <std_srvs/Empty.h>
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

ros::ServiceClient start_profile_merger_, start_point_cloud_writer_, stop_profile_merger_;

void do_scan ( float rotation_deg, float x_s, float y_s, float z_s, float x_e, float y_e, float z_e )
{
  float start_point [ 3 ] { x_s, y_s, z_s };
  float end_point [ 3 ] { x_e, y_e, z_e };

  // create interface for motion planning
  static const std::string PLANNING_GROUP = "me_2900";
  moveit::planning_interface::MoveGroupInterface move_group ( PLANNING_GROUP );
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ROS_INFO_NAMED( "do_scan", "Reference frame: %s", move_group.getPlanningFrame ().c_str () );
  ROS_INFO_NAMED( "do_scan", "End effector link: %s", move_group.getEndEffectorLink ().c_str () );

  ROS_INFO_STREAM ( "Start for scanning" );
  geometry_msgs::Pose target_pose1;
	target_pose1.position.x = start_point [ 0 ];
  target_pose1.position.y = start_point [ 1 ];
	target_pose1.position.z = start_point [ 2 ];
  float rollt = rotation_deg * M_PI / 180.0;
  float pitcht = 0;
  float yawt = 0;
  target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );

  // move to the start point
  move_group.setPoseTarget ( target_pose1 );
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = ( move_group.plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
  ROS_INFO_NAMED ( "do_scan", "planning for the start pose is %s", success ? "success" : "FAILED" );

  if ( success )
  {
    move_group.setMaxVelocityScalingFactor ( 0.1 );
    move_group.setMaxAccelerationScalingFactor ( 0.1 );
    move_group.move ();

    // start scanning the part
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back ( target_pose1 );
    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose2.position.x = end_point [ 0 ];
    target_pose2.position.y = end_point [ 1 ];
    target_pose2.position.z = end_point [ 2 ];
    rollt = rotation_deg * M_PI / 180.0;
    pitcht = 0;
    yawt = 0;
    target_pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
    // just do one way of scanning
    waypoints.push_back ( target_pose2 );

    // geometry_msgs::Pose target_pose3 = target_pose2;
    // target_pose3.position.x = end_point [ 0 ];
    // target_pose3.position.y = end_point [ 1 ];
    // target_pose3.position.z = end_point [ 2 ];
    // rollt = rotation_deg * M_PI / 180.0;
    // pitcht = 0;
    // yawt = 0;
    // target_pose3.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
    // waypoints.push_back ( target_pose3 );
    //
    // geometry_msgs::Pose target_pose4 = target_pose1;
    // target_pose4.position.x = start_point [ 0 ];
    // target_pose4.position.y = start_point [ 1 ];
    // target_pose4.position.z = start_point [ 2 ];
    // rollt = rotation_deg * M_PI / 180.0;
    // pitcht = 0;
    // yawt = 0;
    // target_pose4.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
    // waypoints.push_back ( target_pose4 );

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath ( waypoints, eef_step, jump_threshold, trajectory );
    ROS_INFO_NAMED ( "do_scan", "Cartesian path of the scan plan is %.2f%% acheived", fraction * 100.0 );

    if ( fraction > 0.99 )
    {
      // scale the velocity and the acceleration of the trajectory
      const double scale_factor = 0.015;
      int point_size = trajectory.joint_trajectory.points.size ();
      for ( int point_idx = 0; point_idx < point_size; point_idx++ )
      {
        trajectory_msgs::JointTrajectoryPoint point_tmp = trajectory.joint_trajectory.points [ point_idx ];
        int size_tmp = point_tmp.velocities.size ();
        for ( int i = 0; i <= size_tmp; i++ )
        {
          float velocity_tmp = point_tmp.velocities [ i ];
          trajectory.joint_trajectory.points [ point_idx ].velocities [ i ] = velocity_tmp * scale_factor;
          float acceleration_tmp = point_tmp.accelerations [ i ];
          trajectory.joint_trajectory.points [ point_idx ].accelerations [ i ] = acceleration_tmp * scale_factor;
        }
        ros::Duration time_from_start_tmp = point_tmp.time_from_start;
        trajectory.joint_trajectory.points [ point_idx ].time_from_start.fromSec ( time_from_start_tmp.toSec () / scale_factor );
      }

      my_plan.trajectory_ = trajectory;
      // start the profile_merger service.
      std_srvs::Empty msg;
      // start_profile_merger_.call ( msg );
      move_group.execute ( my_plan );
      // wait for some time to make sure profile merge is over
      // ros::Duration ( 3.0 ) .sleep ();
      // stop the profile_merger service.
      // stop_profile_merger_.call ( msg );
      std::cout << "write merged profile scan" << std::endl;
    }
  }
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "do_scan_c" );
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  float pose_array[] = {0.47, -1.4548, 1.4250, 1.1031}; // 1.1031 1.4250
  do_scan ( 180.0, pose_array [ 0 ], pose_array [ 1 ], pose_array [ 2 ],  pose_array [ 0 ], pose_array [ 1 ], pose_array [ 3 ] );
  ros::waitForShutdown ();
  return 0;
}
