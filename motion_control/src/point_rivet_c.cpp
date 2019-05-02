#include <cmath>
#include <queue>
#include <iostream>
#include <string>

#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "modbus.h"
#include "modbus_exception.h"

ros::ServiceClient new_nut_, start_screwing_, stop_;

class RivetToolControl
{
private:
  boost::shared_ptr < modbus > mb_ptr;
  std::string ip_address;
  int XDKIO = -1;
public:
  RivetToolControl ();
  ~RivetToolControl ();
  void connect ();
  void new_rivet ();
  void start_screwing ();
};

RivetToolControl::RivetToolControl ()
{
  ros::NodeHandle nh_p_ ( "~" );
  nh_p_.getParam ( "XDK", XDKIO );
  nh_p_.getParam ( "ip_address", ip_address );
  mb_ptr.reset ( new modbus ( ip_address, 502 ) );
}

RivetToolControl::~RivetToolControl ()
{
  mb_ptr->modbus_close ();
}

void RivetToolControl::connect ()
{
  if ( XDKIO == -1 )
  {
    std::cout << "RivetToolControl::connect ip_address = " << ip_address << std::endl;
    return;
  }
  mb_ptr->modbus_set_slave_id ( 1 );
  mb_ptr->modbus_connect ();
}

void RivetToolControl::new_rivet ()
{
  if ( XDKIO == -1 )
  {
    std::cout << "RivetToolControl::new_rivet" << std::endl;
    return;
  }
  mb_ptr->modbus_write_register ( 40003, 1 );
  // ros::Duration ( 0.5 ) .sleep ();
}

void RivetToolControl::start_screwing ()
{
  if ( XDKIO == -1 )
  {
    std::cout << "RivetToolControl::start_screwing" << std::endl;
    return;
  }
  mb_ptr->modbus_write_register(40003, 3);
  // ros::Duration ( 5 ) .sleep ();
}

// boost::shared_ptr < RivetToolControl > rivet_tool_ctrl_ptr;

class Target
{
public:
  int id;
  double x, y, z, roll, pitch, yaw;
  Target ( int id_i, double x_i, double y_i, double z_i, double roll_i, double pitch_i, double yaw_i);
};

Target::Target ( int id_i, double x_i, double y_i, double z_i, double roll_i, double pitch_i, double yaw_i)
{
  id = id_i;
  x = x_i;
  y = y_i;
  z = z_i;
  roll = roll_i;
  pitch = pitch_i;
  yaw = yaw_i;
}

void CfgFileReader ( std::queue< Target >& target_queue )
{
  std::string cfgFileName = ros::package::getPath ( "object_localizer" ) + "/config/point_rivet.cfg";
  std::cout << "***The path of the point_rivet configuration file is: [" << cfgFileName << "]" << std::endl;

  int id;
  double x, y, z, roll, pitch, yaw;
  std::ifstream input ( cfgFileName );
  std::string line;
  while ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> id >> x >> y >> z >> roll >> pitch >> yaw;
    Target target ( id, x, y, z, roll, pitch, yaw );
    target_queue.push ( target );
    std::cout << id << ": [x, y, z, roll, pitch, yaw] = [" << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
  }
  input.close();
}

void move_trajectory ( geometry_msgs::Pose& target_pose1, geometry_msgs::Pose& target_pose2, moveit::planning_interface::MoveGroupInterface& move_group )
{
  std::vector < geometry_msgs::Pose> waypoints;
  waypoints.push_back ( target_pose1 );
  waypoints.push_back ( target_pose2 );

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath ( waypoints, eef_step, jump_threshold, trajectory );
  ROS_INFO_NAMED ( "point_rivet", "trajectory plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0 );

  if ( fraction > 0.98 )
  {
    // scale the velocity and acceleration of the trajectory
    const double scale_factor = 0.5;
    int point_size = trajectory.joint_trajectory.points.size();
    for ( int point_idx = 0; point_idx < point_size; point_idx++ )
    {
      trajectory_msgs::JointTrajectoryPoint point_tmp = trajectory.joint_trajectory.points[point_idx];
      int size_tmp = point_tmp.velocities.size();
      for ( int i = 0; i <= size_tmp; i++ )
      {
        float velocity_tmp = point_tmp.velocities[i];
        trajectory.joint_trajectory.points[point_idx].velocities[i] = velocity_tmp * scale_factor;
        float acceleration_tmp = point_tmp.accelerations[i];
        trajectory.joint_trajectory.points[point_idx].accelerations[i] = acceleration_tmp * scale_factor;
      }
      ros::Duration time_from_start_tmp = point_tmp.time_from_start;
      trajectory.joint_trajectory.points[point_idx].time_from_start.fromSec ( time_from_start_tmp.toSec() / scale_factor );
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;
    move_group.execute ( my_plan );
  }
}

void set_target_pose_1 ( Target& target, geometry_msgs::Pose& target_pose )
{
  target_pose.position.x = target.x + 0.04;
  target_pose.position.y = target.y + 0.05;
  target_pose.position.z = target.z;
  float rollt  = target.roll;
  float pitcht = target.pitch;
  float yawt   = target.yaw;
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
}

void set_target_pose ( Target& target, geometry_msgs::Pose& target_pose )
{
  target_pose.position.x = target.x;
  target_pose.position.y = target.y;
  target_pose.position.z = target.z;
  float rollt  = target.roll;
  float pitcht = target.pitch;
  float yawt   = target.yaw;
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
}

void do_point_rivet ()
{
  std_srvs::Empty msg;
  std::queue< Target > target_queue;
  CfgFileReader ( target_queue );
  // create interface for motion planning
  static const std::string PLANNING_GROUP = "rivet_tool";
  moveit::planning_interface::MoveGroupInterface move_group ( PLANNING_GROUP );
  ROS_INFO_NAMED( "point_rivet", "Reference frame: %s", move_group.getPlanningFrame ().c_str () );
  ROS_INFO_NAMED( "point_rivet", "End effector link: %s", move_group.getEndEffectorLink ().c_str () );

  if ( !target_queue.empty () )
  {
    Target target = target_queue.front ();
    target_queue.pop ();
    geometry_msgs::Pose target_pose1;
    set_target_pose_1 ( target, target_pose1 );
    move_group.setPoseTarget ( target_pose1 );
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO_STREAM ( "Move to start pose:" );
    bool success = ( move_group.plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
    ROS_INFO_NAMED ( "point_rivet", "planning for goal pose is %s", success ? "success" : "FAILED" );

    if ( success )
    {
      move_group.setMaxVelocityScalingFactor ( 0.3 );
      move_group.setMaxAccelerationScalingFactor ( 0.3 );
      move_group.move ();
      ros::Duration ( 0.5 ) .sleep ();

      set_target_pose ( target, target_pose1 );
      move_group.setPoseTarget ( target_pose1 );
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      ROS_INFO_STREAM ( "Move to start pose:" );
      bool success = ( move_group.plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
      ROS_INFO_NAMED ( "point_rivet", "planning for goal pose is %s", success ? "success" : "FAILED" );

      if ( success )
      {
        move_group.setMaxVelocityScalingFactor ( 0.3 );
        move_group.setMaxAccelerationScalingFactor ( 0.3 );
        move_group.move ();
        ros::Duration ( 0.5 ) .sleep ();

        // start the screwing part
        while ( !target_queue.empty () )
        {
          // get the new rivet
          ros::Duration ( 0.5 ) .sleep ();
          new_nut_.call ( msg );
          ros::Duration ( 2.0 ) .sleep ();

          // rivet_tool_ctrl_ptr -> new_rivet ();
          Target target = target_queue.front ();
          target_queue.pop();
          // start screwing the rivet before moving to the rivet
          start_screwing_.call ( msg );
          // rivet_tool_ctrl_ptr -> start_screwing ();
          geometry_msgs::Pose target_pose2;
          set_target_pose ( target, target_pose2 );
          move_trajectory ( target_pose1, target_pose2, move_group );
          ros::Duration ( 5 ) .sleep ();
          target_pose1 = target_pose2;

          // move back the rivet_tool
          target = target_queue.front();
          target_queue.pop();
          set_target_pose ( target, target_pose2 );
          move_trajectory ( target_pose1, target_pose2, move_group );
          stop_.call ( msg );
          ros::Duration ( 0.5 ) .sleep ();
          new_nut_.call ( msg );
          ros::Duration ( 2.0 ) .sleep ();
          stop_.call ( msg );
          target_pose1 = target_pose2;
          // move to the next rivet if there exist the next rivet
          if ( !target_queue.empty () )
          {
            target = target_queue.front();
            target_queue.pop();
            set_target_pose ( target, target_pose2 );
            move_trajectory ( target_pose1, target_pose2, move_group );
            target_pose1 = target_pose2;
          }
          //  ros::Duration ( 1.5 ) .sleep ();  
        }
      //ros::Duration ( 1.0 ) .sleep ();

      }
    }
  }
}

bool start_point_rivet ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  do_point_rivet ();
  return true;
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "point_rivet_b" );
  ros::NodeHandle nh_;

  new_nut_ = nh_.serviceClient < std_srvs::Empty > ( "new_nut" );
  start_screwing_ = nh_.serviceClient < std_srvs::Empty > ( "start_screwing" );
  stop_ = nh_.serviceClient < std_srvs::Empty > ( "stop" );

  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  // rivet_tool_ctrl_ptr.reset ( new RivetToolControl () );
  // rivet_tool_ctrl_ptr -> connect ();
  ros::ServiceServer start_point_rivet_;
  start_point_rivet_ = nh_.advertiseService ( "start_point_rivet", &start_point_rivet );
  ros::waitForShutdown ();
  return 0;
}
