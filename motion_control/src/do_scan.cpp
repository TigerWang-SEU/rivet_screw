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
    move_group.setMaxVelocityScalingFactor ( 0.3 );
    move_group.setMaxAccelerationScalingFactor ( 0.3 );
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
      const double scale_factor = 0.035;
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
      start_profile_merger_.call ( msg );
      move_group.execute ( my_plan );
      ros::Duration ( 3.0 ) .sleep ();
      stop_profile_merger_.call ( msg );
      std::cout << "write merged profile scan" << std::endl;
      start_point_cloud_writer_.call ( msg );
    }
  }
}

class ScanPlan
{
public:

  float rotation_deg, x_s, y_s, z_s, x_e, y_e, z_e;

  ScanPlan ( float rotation_deg, float x_s, float y_s, float z_s, float x_e, float y_e, float z_e )
  {
    this->rotation_deg = rotation_deg;
    this->x_s = x_s;
    this->y_s = y_s;
    this->z_s = z_s;
    this->x_e = x_e;
    this->y_e = y_e;
    this->z_e = z_e;
  }

};

bool scanPlanComp ( ScanPlan i,ScanPlan j )
{
  return ( i.y_s < j.y_s );
}

void CfgFileReader ( std::vector< ScanPlan >& scan_plan_vector )
{
  ros::NodeHandle nh_p_ ( "~" );
  std::string scanFileName;
  nh_p_.getParam ( "scan_file", scanFileName );
  std::string cfgFileName = ros::package::getPath ( "motion_control" ) + "/config/" + scanFileName;
  std::cout << "***The path of the do_scan configuration file is: [" << cfgFileName << "]" << std::endl;

  double rotation_deg, x_s, y_s, z_s, x_e, y_e, z_e;
  std::ifstream input ( cfgFileName );
  std::string line;
  int scan_plan_idx = 0;
  while ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> rotation_deg >> x_s >> y_s >> z_s >> x_e >> y_e >> z_e;
    ScanPlan scan_plan ( rotation_deg, x_s, y_s, z_s, x_e, y_e, z_e );
    scan_plan_vector.push_back ( scan_plan );
    // std::cout << "*** scan_plan_idx = [" << scan_plan_idx << "] : [rotation_deg, x_s, y_s, z_s, x_e, y_e, z_e] = [" << rotation_deg << ", " << x_s << ", " << y_s << ", " << z_s << ", " << x_e << ", " << y_e << ", " << z_e << "]" << std::endl;
    scan_plan_idx ++;
  }
  input.close();
}

bool start_do_scan ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  // read the configuration file
  std::vector< ScanPlan > scan_plan_vector;
  CfgFileReader ( scan_plan_vector );
  // std::cout << "Choose a scanning plan using the scan_plan_idx:" << std::endl;
  int scan_plan_idx = 1;
  ros::NodeHandle nh_p_ ( "~" );
  nh_p_.getParam ( "scan_idx", scan_plan_idx );
  std::sort ( scan_plan_vector.begin(), scan_plan_vector.end(), scanPlanComp );
  for ( int i = 0; i < scan_plan_vector.size(); i++ )
  {
    ScanPlan scan_plan = scan_plan_vector [ i ];
    std::cout << "*** scan_plan_idx = [" << i << "] : [rotation_deg, x_s, y_s, z_s, x_e, y_e, z_e] = [" << scan_plan.rotation_deg << ", " << scan_plan.x_s << ", " << scan_plan.y_s << ", " << scan_plan.z_s << ", " << scan_plan.x_e << ", " << scan_plan.y_e << ", " << scan_plan.z_e << "]" << std::endl;
  }
  if ( scan_plan_idx == 1 && scan_plan_vector.size() >= 1 )
  {
    ScanPlan scan_plan = scan_plan_vector [ 0 ];
    do_scan ( scan_plan.rotation_deg, scan_plan.x_s, scan_plan.y_s, scan_plan.z_s, scan_plan.x_e, scan_plan.y_e, scan_plan.z_e );
  } else if ( scan_plan_idx == 2 && scan_plan_vector.size() >= 2 )
  {
    ScanPlan scan_plan = scan_plan_vector [ 1 ];
    do_scan ( scan_plan.rotation_deg, scan_plan.x_s, scan_plan.y_s, scan_plan.z_s, scan_plan.x_e, scan_plan.y_e, scan_plan.z_e );
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
  start_profile_merger_ = nh_.serviceClient < std_srvs::Empty > ( "start_profile_merger" );
  start_point_cloud_writer_ = nh_.serviceClient < std_srvs::Empty > ( "start_point_cloud_writer" );
  stop_profile_merger_ = nh_.serviceClient < std_srvs::Empty > ( "stop_profile_merger" );
  ros::waitForShutdown ();
  return 0;
}
