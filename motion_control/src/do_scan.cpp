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
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "head/do_scan.h"
#include "head/motion_plan.h"
#include "/home/syn/ros_ws/src/object_localizer/src/head/planner.h"

void set_rivet_tool_forward ( MotionControl& motion_control )
{
  std::vector < double > joint_group_positions;
  motion_control.get_current_robot_state ( joint_group_positions );
  double wrist_1_angle = joint_group_positions [ 3 ];
  double wrist_2_angle = joint_group_positions [ 4 ];
  double wrist_3_angle = joint_group_positions [ 5 ];
  std::cout << "current wrist 3 angle = " << wrist_3_angle << std::endl;

  motion_control.set_joint_angle ( 4, 0 );
  motion_control.set_joint_angle ( 5, wrist_3_angle - 3.1415 );
  motion_control.set_joint_angle ( 4, wrist_2_angle );
}

void do_scan ( float rotation_deg, float x_s, float y_s, float z_s, float x_e, float y_e, float z_e, float x_final, float y_final, float z_final )
{
  // create motion_control object and the start_profile_merger_ and stop_profile_merger_ service clients.
  std::string PLANNING_GROUP = "me_2900";
  MotionControl motion_control ( PLANNING_GROUP );
  ros::ServiceClient start_profile_merger_, stop_profile_merger_;
  ros::NodeHandle nh_;
  start_profile_merger_ = nh_.serviceClient < std_srvs::Empty > ( "start_profile_merger" );
  stop_profile_merger_ = nh_.serviceClient < std_srvs::Empty > ( "stop_profile_merger" );

  float start_point [ 3 ] { x_s, y_s, z_s };
  float end_point [ 3 ] { x_e, y_e, z_e };
  float final_point [ 3 ] { x_final, y_final, z_final };

  ROS_INFO_STREAM ( "Start for scanning" );
  geometry_msgs::Pose scan_start_pose;
  motion_control.get_current_end_effector_pose ( scan_start_pose );
  bool success = motion_control.move2target ( scan_start_pose, 0.1 );

  if ( success )
  {
    geometry_msgs::Pose target_pose1;
    float rollt, pitcht, yawt;
    target_pose1.position.x = start_point [ 0 ];
    target_pose1.position.y = start_point [ 1 ];
    target_pose1.position.z = start_point [ 2 ];
    rollt = rotation_deg * M_PI / 180.0;
    pitcht = 0;
    yawt = 0;
    target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
    std::vector<geometry_msgs::Pose> waypoints_0;
    waypoints_0.push_back ( scan_start_pose );
    waypoints_0.push_back ( target_pose1 );

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    double fraction = motion_control.get_trajectory ( waypoints_0, my_plan, 0.3 );
    if ( fraction > 0.98 )
    {
      motion_control.execute_trajectory ( my_plan );
    }
    else
    {
      return;
    }

    // start the scanning part
    geometry_msgs::Pose target_pose2;
    target_pose2.position.x = end_point [ 0 ];
    target_pose2.position.y = end_point [ 1 ];
    target_pose2.position.z = end_point [ 2 ];
    rollt = rotation_deg * M_PI / 180.0;
    pitcht = 0;
    yawt = 0;
    target_pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
    // do one way of scanning
    std::vector<geometry_msgs::Pose> waypoints_1;
    waypoints_1.push_back ( target_pose1 );
    waypoints_1.push_back ( target_pose2 );
    waypoints_1.push_back ( target_pose1 );
    fraction = motion_control.get_trajectory ( waypoints_1, my_plan, 0.035 );
    if ( fraction > 0.98 )
    {
      std_srvs::Empty msg;
      start_profile_merger_.call ( msg );
      motion_control.execute_trajectory ( my_plan );
      stop_profile_merger_.call ( msg );
    }

    // move backward and turn the rivet tool around
    geometry_msgs::Pose final_pose;
    final_pose.position.x = final_point [ 0 ];
    final_pose.position.y = final_point [ 1 ];
    final_pose.position.z = final_point [ 2 ];
    rollt = rotation_deg * M_PI / 180.0;
    pitcht = 0;
    yawt = 0;
    final_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
    std::vector<geometry_msgs::Pose> waypoints_2;
    waypoints_2.push_back ( target_pose1 );
    waypoints_2.push_back ( final_pose );
    fraction = motion_control.get_trajectory ( waypoints_2, my_plan, 0.5 );
    if ( fraction > 0.98 )
    {
      motion_control.execute_trajectory ( my_plan );
      set_rivet_tool_forward ( motion_control );
    }
  }
}

bool start_do_scan ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  // read the configuration file
  std::vector < ScanPlan > scan_plan_vector;
  scan_plan_reader ( scan_plan_vector );
  int scan_plan_idx;
  read_idx ( scan_plan_idx );
  if ( scan_plan_idx < scan_plan_vector.size() )
  {
    ScanPlan scan_plan = scan_plan_vector [ scan_plan_idx ];
    do_scan ( scan_plan.rotation_deg, scan_plan.x_s, scan_plan.y_s, scan_plan.z_s, scan_plan.x_e, scan_plan.y_e, scan_plan.z_e, scan_plan.x_final, scan_plan.y_final, scan_plan.z_final );
  }
  return true;
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "do_scan" );
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ros::ServiceServer start_do_scan_;
  start_do_scan_ = nh_.advertiseService ( "start_do_scan", &start_do_scan );
  ros::waitForShutdown ();
  return 0;
}
