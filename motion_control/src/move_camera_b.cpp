#include <cmath>
#include <iostream>

#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

float motion_scale = 0.03;
ros::ServiceClient start_image_transport, stop_image_transport;
std_srvs::Empty msg;

void move_camera ()
{
  static const std::string PLANNING_GROUP = "camera";
  moveit::planning_interface::MoveGroupInterface move_group ( PLANNING_GROUP );
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup ( PLANNING_GROUP );
  ROS_INFO_NAMED ( "move_camera", "Reference frame: %s", move_group.getPlanningFrame ().c_str () );
  ROS_INFO_NAMED ( "move_camera", "End effector link: %s", move_group.getEndEffectorLink ().c_str () );

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState ();
  std::vector < double > joint_group_positions;
  current_state->copyJointGroupPositions ( joint_model_group, joint_group_positions );
  std::cout << "current wrist 1: " << joint_group_positions [ 3 ] << std::endl;
  // if the start robot pose is scan_start pose, start move the camera
  if ( std::abs ( joint_group_positions [ 3 ] + 1.8434 ) <= 0.01 && std::abs ( joint_group_positions [ 5 ] - 0 ) <= 0.01 )
  {
    int motion_stage_idx = 0;
    while ( motion_stage_idx != 1 )
    {
      if ( motion_stage_idx  == 0 )
      {
        start_image_transport.call ( msg );
        // set pose scan_end in radians
        // 3.1442, -0.8294, -2.0604, -1.5809, 1.5696, -0.0035
        joint_group_positions [ 0 ] =  3.1442;
        joint_group_positions [ 1 ] = -0.8294;
        joint_group_positions [ 2 ] = -2.0604;
        joint_group_positions [ 3 ] = -1.5809;
        joint_group_positions [ 4 ] =  1.5696;
        joint_group_positions [ 5 ] = -0.0035;
      }
      else if ( motion_stage_idx  == 1 )
      {
        // set pose scan_start in radians
        joint_group_positions [ 3 ] = -1.8434;
        ros::Duration ( 0.5 ).sleep ();
        stop_image_transport.call ( msg );
        ros::Duration ( 0.5 ).sleep ();
        stop_image_transport.call ( msg );
      }
      move_group.setJointValueTarget ( joint_group_positions );
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = ( move_group.plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
      if ( success )
      {
        std::cout << "move wrist 1 to: " << joint_group_positions [ 3 ] << std::endl;
        if ( motion_stage_idx == 0 )
        {
          move_group.setMaxVelocityScalingFactor ( motion_scale );
          move_group.setMaxAccelerationScalingFactor ( motion_scale );
          move_group.move ();
        }
        if ( motion_stage_idx == 1 )
        {
          move_group.setMaxVelocityScalingFactor ( 0.1 );
          move_group.setMaxAccelerationScalingFactor ( 0.1 );
          move_group.move ();
        }
      }
      motion_stage_idx ++;
    }
  }
}

bool start_move_camera ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  move_camera ();
  return true;
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "move_camera_b" );
  ros::NodeHandle nh_;
  start_image_transport = nh_.serviceClient < std_srvs::Empty > ( "start_image_transport" );
  stop_image_transport = nh_.serviceClient < std_srvs::Empty > ( "stop_image_transport" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ros::ServiceServer start_move_camera_;
  start_move_camera_ = nh_.advertiseService ( "start_move_camera", &start_move_camera );
  ros::waitForShutdown ();
  return 0;
}
