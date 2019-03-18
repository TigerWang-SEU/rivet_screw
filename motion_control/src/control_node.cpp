#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <std_srvs/Empty.h>
#include <ros/package.h>
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

static const std::string PLANNING_GROUP = "camera";

class ControlNode {

  ros::NodeHandle nh_;
  tf2_ros::StaticTransformBroadcaster br;
  ros::ServiceClient start_fix_table_position_, stop_fix_table_position_;
  ros::ServiceClient add_aircraft_frame_, remove_aircraft_frame_, start_pcl_merger_, stop_pcl_merger_,
                     start_rough_localizer_, stop_rough_localizer_, start_box_segmenter_, stop_box_segmenter_,
                     start_scan_planner_, stop_scan_planner_, start_move_camera_;
  ros::ServiceClient start_do_scan_, start_rivet_localizer_, start_point_rivet_;

  boost::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group;
  boost::shared_ptr< moveit::planning_interface::PlanningSceneInterface > planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group;

public:

  ControlNode ()
  {
    //  add_aircraft_frame and remove_aircraft_frame
    add_aircraft_frame_ = nh_.serviceClient < std_srvs::Empty > ( "add_aircraft_frame" );
    remove_aircraft_frame_ = nh_.serviceClient < std_srvs::Empty > ( "remove_aircraft_frame" );
    // start_fix_table_position and stop_fix_table_position
    start_fix_table_position_ = nh_.serviceClient < std_srvs::Empty > ( "start_fix_table_position" );
    stop_fix_table_position_ = nh_.serviceClient < std_srvs::Empty > ( "stop_fix_table_position" );
    // pcl_merger, rough_localizer, box_segmenter, scan_planner and move_camera
    start_pcl_merger_ = nh_.serviceClient < std_srvs::Empty > ( "start_pcl_merge" );
    stop_pcl_merger_ = nh_.serviceClient < std_srvs::Empty > ( "stop_pcl_merge" );
    start_rough_localizer_ = nh_.serviceClient < std_srvs::Empty > ( "start_rough_localizer" );
    stop_rough_localizer_ = nh_.serviceClient < std_srvs::Empty > ( "stop_rough_localizer" );
    start_box_segmenter_ = nh_.serviceClient < std_srvs::Empty > ( "start_box_segmenter" );
    stop_box_segmenter_ = nh_.serviceClient < std_srvs::Empty > ( "stop_box_segmenter" );
    start_scan_planner_ = nh_.serviceClient < std_srvs::Empty > ( "start_scan_planner" );
    stop_scan_planner_ = nh_.serviceClient < std_srvs::Empty > ( "stop_scan_planner" );
    start_move_camera_ = nh_.serviceClient < std_srvs::Empty > ( "start_move_camera" );
    start_do_scan_ = nh_.serviceClient < std_srvs::Empty > ( "start_do_scan" );
    start_rivet_localizer_ = nh_.serviceClient < std_srvs::Empty > ( "start_rivet_localizer" );
    start_point_rivet_ = nh_.serviceClient < std_srvs::Empty > ( "start_point_rivet" );

    move_group.reset ( new moveit::planning_interface::MoveGroupInterface ( PLANNING_GROUP ) );
    planning_scene_interface.reset ( new moveit::planning_interface::PlanningSceneInterface () );
    joint_model_group = move_group->getCurrentState()->getJointModelGroup ( PLANNING_GROUP );
    ROS_INFO_NAMED ( "control_node", "Reference frame: %s", move_group->getPlanningFrame().c_str() );
    ROS_INFO_NAMED ( "control_node", "End effector link: %s", move_group->getEndEffectorLink().c_str() );
  }

  ~ControlNode ()
  {}

  bool set_pose2 ()
  {
    std::map < std::string, double > value_map = move_group->getNamedTargetValues ( "pose2" );
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
    // std::cout << "\tContinue with current position (y/n): ";
    std::string answer_str;
    answer_str = "Y";
    // std::cin >> answer_str;
    if ( answer_str == "n" )
    {
      return;
    }
    // step 1, fix the table position and add aircraft_frame to the planning scene.
    if ( start_fix_table_position_.call ( msg ) && add_aircraft_frame_.call ( msg ) )
    {
      std::cout << "1, fix the table position and add the aircraft frame" << std::endl;
      // step 2, set the robot pose to pose 2.
      std::cout << "2, set the robot pose to pose 2" << std::endl;
      if ( set_pose2 () )
      {
        // step 3, start services pcl_merger, rough_localizer, and box_segmenter
        std::cout << "3, start services pcl_merger, rough_localizer, box_segmenter" << std::endl;
        if ( start_pcl_merger_.call ( msg ) && start_rough_localizer_.call ( msg )
             && start_box_segmenter_.call ( msg ) )
        {
          // step 4, start service move_camera
          std::cout << "4, start to move the camera" << std::endl;
          start_move_camera_.call ( msg );
          // step 5, stop services pcl_merger, rough_localizer, and box_segmenter
          std::cout << "5, stop services pcl_merger, rough_localizer, box_segmenter" << std::endl;
          stop_pcl_merger_.call ( msg );
          if ( stop_rough_localizer_.call ( msg ) && stop_box_segmenter_.call ( msg ) )
          {
            // step 6, generate scanning plans and write it to the configuration file [do_scan]
            std::cout << "6, start to generate scanning plans" << std::endl;
            start_scan_planner_.call ( msg );

            // step 7, remove the aircraft frame and start profile scan
            std::cout << "7, remove the aircraft_frame and start the profile scanning" << std::endl;
            remove_aircraft_frame_.call ( msg );
            start_do_scan_.call ( msg );

            // step 8, move back to pose 2
            std::cout << "8, set the robot pose back to pose 2" << std::endl;
            if ( set_pose2 () )
            {
              // step 9, call the service rivet_localizer
              std::cout << "9, start rivet localizer" << std::endl;
              start_rivet_localizer_.call ( msg );

              // step 10, call the service point_rivet
              std::cout << "10, start to point to rivet" << std::endl;
              start_point_rivet_.call ( msg );

              // step 11, call the service point_rivet
              std::cout << "11, set the robot pose back to pose 2" << std::endl;
              set_pose2 ();
            }
          }
        }
      }
    }
  }

};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "control_node" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ControlNode control_node;
  control_node.execute_pipeline ();
  ros::waitForShutdown ();
  return 0;
}
