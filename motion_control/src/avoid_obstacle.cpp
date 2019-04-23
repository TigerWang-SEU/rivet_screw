#include <cmath>
#include <queue>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

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
  std::string cfgFileName = ros::package::getPath ( "motion_control" ) + "/config/avoid_obstacle.cfg";
  std::cout << "***The path of the avoid_obstacle configuration file is: [" << cfgFileName << "]" << std::endl;

  int id;
  double x, y, z, roll, pitch, yaw;
  roll = 3.1415;
  pitch = 0.7305; // 0.867855; 0.034 = 2 degree 1 = 6 degree
  yaw = 1.5707;
  std::ifstream input ( cfgFileName );
  std::string line;
  while ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> id >> x >> y >> z >> pitch;
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

void do_move ()
{
  ros::NodeHandle nh_;
  std_srvs::Empty msg;
  ros::ServiceClient start_fix_table_position_ = nh_.serviceClient < std_srvs::Empty > ( "start_fix_table_position" );

  if ( start_fix_table_position_.call ( msg ) )
  {
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
      geometry_msgs::Pose target_pose1, target_pose2;
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

        // start the move part
        while ( !target_queue.empty () )
        {
          target = target_queue.front();
          target_queue.pop();
          set_target_pose ( target, target_pose2 );
          move_trajectory ( target_pose1, target_pose2, move_group );
          target_pose1 = target_pose2;
        }

        // {
        //   ros::NodeHandle nh_;
        //   ros::ServiceClient add_aircraft_frame_ = nh_.serviceClient < std_srvs::Empty > ( "add_aircraft_frame" );
        //   std_srvs::Empty msg;
        //   if ( add_aircraft_frame_.call ( msg ) )
        //   {
        //     // set back to pose 2
        //     std::map < std::string, double > value_map = move_group.getNamedTargetValues ( "pose2" );
        //     move_group.setJointValueTarget ( value_map );
        //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        //     bool success = ( move_group.plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
        //     if ( success )
        //     {
        //       // std::cout << "reset the robot pose to pose 2" << std::endl;
        //       move_group.setMaxVelocityScalingFactor ( 0.1 );
        //       move_group.setMaxAccelerationScalingFactor ( 0.1 );
        //       move_group.move ();
        //     }
        //   }
        // }
      }
    }
  }
}

static const std::string PLANNING_GROUP = "camera";

class ControlNode {

  ros::NodeHandle nh_;
  ros::ServiceClient start_fix_table_position_, stop_fix_table_position_, start_pcl_merger_, stop_pcl_merger_, start_move_camera_, start_avoid_obstacle_planner_;

  boost::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group;
  boost::shared_ptr< moveit::planning_interface::PlanningSceneInterface > planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group;

public:

  ControlNode ()
  {
    // start_fix_table_position and stop_fix_table_position
    start_fix_table_position_ = nh_.serviceClient < std_srvs::Empty > ( "start_fix_table_position" );
    stop_fix_table_position_ = nh_.serviceClient < std_srvs::Empty > ( "stop_fix_table_position" );
    // pcl_merger, move_camera, and avoid_obstacle_planner
    start_pcl_merger_ = nh_.serviceClient < std_srvs::Empty > ( "start_pcl_merge" );
    stop_pcl_merger_ = nh_.serviceClient < std_srvs::Empty > ( "stop_pcl_merge" );
    start_move_camera_ = nh_.serviceClient < std_srvs::Empty > ( "start_move_camera" );
    start_avoid_obstacle_planner_ = nh_.serviceClient < std_srvs::Empty > ( "start_avoid_obstacle_planner" );

    move_group.reset ( new moveit::planning_interface::MoveGroupInterface ( PLANNING_GROUP ) );
    planning_scene_interface.reset ( new moveit::planning_interface::PlanningSceneInterface () );
    joint_model_group = move_group->getCurrentState()->getJointModelGroup ( PLANNING_GROUP );
    ROS_INFO_NAMED ( "control_node", "Reference frame: %s", move_group->getPlanningFrame().c_str() );
    ROS_INFO_NAMED ( "control_node", "End effector link: %s", move_group->getEndEffectorLink().c_str() );
  }

  ~ControlNode ()
  {}

  // scan_start, scan_end, screw_start
  bool set_pose ( std::string pose_name )
  {
    std::map < std::string, double > value_map = move_group->getNamedTargetValues ( pose_name );
    // std::cout << move_group->getNamedTargetValues ( "pose2" ) << std::endl;
    // for ( const auto& value_pair : value_map )
    // {
    //   std::cout << "<" << value_pair.first << "> = <" << value_pair.second << ">\n";
    // }
    move_group->setJointValueTarget ( value_map );
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = ( move_group->plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
    if ( success )
    {
      // std::cout << "reset the robot pose to pose 2" << std::endl;
      move_group->setMaxVelocityScalingFactor ( 0.1 );
      move_group->setMaxAccelerationScalingFactor ( 0.1 );
      move_group->move ();
    }
    return success;
  }

  void execute_pipeline ()
  {
    std_srvs::Empty msg;
    // step 0, update the table position
    std::cout << "0, update the table position" << std::endl;
    stop_fix_table_position_.call ( msg );
    ros::Duration ( 2.0 ) .sleep ();

    // step 1, fix the table position and set robot pose to [scan_start].
    std::cout << "1, fix the table position and set robot pose to [scan_start]" << std::endl;
    if ( start_fix_table_position_.call ( msg ) && set_pose ( "scan_start" ) )
    {
      // step 2, start services pcl_merger.
      std::cout << "2, start services pcl_merger" << std::endl;
      if ( start_pcl_merger_.call ( msg ) )
      {
        // step 3, start service move_camera
        std::cout << "3, start to move the camera" << std::endl;
        start_move_camera_.call ( msg );
        // step 4, stop services pcl_merger
        std::cout << "4, stop services pcl_merger" << std::endl;
        stop_pcl_merger_.call ( msg );
        // step 4.1, set robot pose to [scan_start].
        set_pose ( "scan_start" );

        // // step 5, generate an avoid obstacle path
        // std::cout << "5, start to avoid obstacle path" << std::endl;
        // start_avoid_obstacle_planner_.call ( msg );
        // // step 6, execute the avoid obstacle path
        // std::cout << "6, execute the avoid obstacle path y/n:" << std::endl;
        // std::string answer;
        // std::cin >> answer;
        // if ( answer == "y" )
    		// {
        //   do_move ();
        // }
      }
    }
  }
};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "avoid_obstacle" );
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ControlNode control_node;
  control_node.execute_pipeline ();
  // do_move ();
  ros::waitForShutdown ();
  return 0;
}
