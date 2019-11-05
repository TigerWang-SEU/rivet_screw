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

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

class MotionControl
{
  std::string PLANNING_GROUP;
  boost::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group;
  std::string reference_frame = "world";
  std::string END_EFFECTOR_FRAME;
  tf::TransformListener listener;

public:

  MotionControl ( std::string PLANNING_GROUP )
  {
    this->PLANNING_GROUP = PLANNING_GROUP;
    move_group.reset ( new moveit::planning_interface::MoveGroupInterface ( PLANNING_GROUP ) );

    ROS_INFO_NAMED ( "motion_control", "Reference frame: %s", move_group->getPlanningFrame().c_str() );
    END_EFFECTOR_FRAME = move_group->getEndEffectorLink();
    ROS_INFO_NAMED ( "motion_control", "End effector link: %s", END_EFFECTOR_FRAME.c_str() );
  }

  bool move2target ( geometry_msgs::Pose& target_pose, double scale_factor = 0.3 )
  {
    move_group->setPoseTarget ( target_pose );
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = ( move_group->plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
    ROS_INFO_NAMED ( "do_scan", "planning for the target pose is %s", success ? "success" : "FAILED" );

    if ( success )
    {
      move_group->setMaxVelocityScalingFactor ( scale_factor );
      move_group->setMaxAccelerationScalingFactor ( scale_factor );
      move_group->move ();
    }

    return success;
  }

  double get_trajectory ( std::vector < geometry_msgs::Pose>& waypoints, moveit::planning_interface::MoveGroupInterface::Plan& my_plan, double scale_factor = 0.3 )
  {
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath ( waypoints, eef_step, jump_threshold, trajectory );
    ROS_INFO_NAMED ( "motion_control", "Cartesian path plan (%.2f%% acheived)", fraction * 100.0 );

    if ( fraction > 0.98 )
    {
      // scale the velocity and acceleration of the trajectory
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
          trajectory.joint_trajectory.points[point_idx].accelerations [ i ] = acceleration_tmp * scale_factor;
        }
        ros::Duration time_from_start_tmp = point_tmp.time_from_start;
        trajectory.joint_trajectory.points [ point_idx ].time_from_start.fromSec ( time_from_start_tmp.toSec () / scale_factor );
      }

      my_plan.trajectory_ = trajectory;
    }

    return fraction;
  }

  double execute_trajectory ( moveit::planning_interface::MoveGroupInterface::Plan& my_plan )
  {
    move_group->execute ( my_plan );
  }

  void set_joint_angle ( int joint_idx, double joint_angle )
  {
    std::vector < double > joint_group_positions;
    get_current_robot_state ( joint_group_positions );

    joint_group_positions [ joint_idx ] = joint_angle;
    move_group->setJointValueTarget ( joint_group_positions );
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = ( move_group->plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
    if ( success )
    {
      move_group->setMaxVelocityScalingFactor ( 0.1 );
      move_group->setMaxAccelerationScalingFactor ( 0.1 );
      move_group->move ();
    }
  }

  void get_current_robot_state ( std::vector < double >& joint_group_positions )
  {
    moveit::core::RobotStatePtr current_state = move_group->getCurrentState ();
    const robot_state::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup ( PLANNING_GROUP );
    current_state->copyJointGroupPositions ( joint_model_group, joint_group_positions );
  }

  void get_current_end_effector_pose ( geometry_msgs::Pose& target_pose )
  {
    std::cout << END_EFFECTOR_FRAME << " to " << reference_frame << std::endl;
    target_pose = move_group->getCurrentPose().pose;
    std::cout << "\tcurrent pose is " << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << ", " << target_pose.orientation.x << ", " << target_pose.orientation.y << ", " << target_pose.orientation.z << ", " << target_pose.orientation.w << std::endl;
  }

};

#endif
