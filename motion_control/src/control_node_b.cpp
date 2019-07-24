#include <cmath>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud< PointT > PointCloudT;
static const std::string PLANNING_GROUP = "camera";

class ControlNode {

  ros::ServiceClient start_pcl_merger_, stop_pcl_merger_, start_rough_localizer_, stop_rough_localizer_, start_box_segmenter_, stop_box_segmenter_, start_scan_planner_, stop_scan_planner_, start_move_camera_, start_do_scan_, start_rivet_localizer_, start_point_rivet_, stop_image_transport_;

  boost::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group;
  boost::shared_ptr< moveit::planning_interface::PlanningSceneInterface > planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group;

  ros::NodeHandle nh_;
  pcl::PointCloud<PointT>::Ptr scene_cloud_;
  ros::Subscriber cloud_sub_;
  pcl::PLYWriter writer;

  int current_execution_phase;
  bool is_pause;
  ros::ServiceServer robot_pause, robot_restart;
  ros::Publisher control_node_pub;

public:

  ControlNode () : scene_cloud_ ( new pcl::PointCloud< PointT > )
  {
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
    stop_image_transport_ = nh_.serviceClient < std_srvs::Empty > ( "stop_image_transport" );

    move_group.reset ( new moveit::planning_interface::MoveGroupInterface ( PLANNING_GROUP ) );
    planning_scene_interface.reset ( new moveit::planning_interface::PlanningSceneInterface () );
    joint_model_group = move_group->getCurrentState()->getJointModelGroup ( PLANNING_GROUP );
    ROS_INFO_NAMED ( "control_node", "Reference frame: %s", move_group->getPlanningFrame().c_str() );
    ROS_INFO_NAMED ( "control_node", "End effector link: %s", move_group->getEndEffectorLink().c_str() );

    std::string cloud_in_name = "/profile_merger/points";
    cloud_sub_ = nh_.subscribe ( cloud_in_name, 3, &ControlNode::cloud_cb, this );
    ROS_INFO_STREAM ( "Listening point cloud message on topic " << cloud_in_name );

    current_execution_phase = 0;
    is_pause = false;
    robot_pause = nh_.advertiseService ( "robot_pause", &ControlNode::pause_execution, this );
    robot_restart = nh_.advertiseService ( "robot_restart", &ControlNode::restart_execution, this );
    control_node_pub = nh_.advertise <std_msgs::String> ( "/control_node/status", 100 );
  }

  ~ControlNode ()
  {}

  void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    // if input cloud is empty or is not dense, publish the old point cloud and return
    if ( ( cloud->width * cloud->height ) == 0 ) // && ! cloud->is_dense
    {
      return;
    }

    // convert input ros cloud to pcl cloud
    // and save it in local variable scene_cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL( *cloud, pcl_pc2 );
    pcl::fromPCLPointCloud2 ( pcl_pc2, *scene_cloud_ );
  }

  // camera_start, camera_end, scan_start, screw_start
  bool set_pose ( std::string pose_name )
  {
    std::map < std::string, double > value_map = move_group->getNamedTargetValues ( pose_name );
    move_group->setJointValueTarget ( value_map );
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = ( move_group->plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
    if ( success )
    {
      move_group->setMaxVelocityScalingFactor ( 0.1 );
      move_group->setMaxAccelerationScalingFactor ( 0.1 );
      move_group->move ();
    }
    return success;
  }

  bool save_profile_pc ()
  {
    if ( scene_cloud_->size() > 0 )
    {
      std::string pc_file_path = ros::package::getPath ( "object_localizer" )+ "/data/pc_out_0.ply";
		  std::cout << "\tSaving point cloud to file: \n\t" << pc_file_path << std::endl;
      writer.write ( pc_file_path, *scene_cloud_ );
    }
  }

  void publish_msg ( std::string in_msg )
  {
    std::stringstream ss;
    ss << in_msg;
    std_msgs::String msg;
    msg.data = ss.str();
    control_node_pub.publish ( msg );
  }

  void execute_stage ( int stage_idx )
  {
    std_srvs::Empty msg;
    switch ( stage_idx )
    {
      case 0:
      {
        publish_msg ( "mid_referencing_start" );
        std::cout << "1, set the robot pose to camera_start" << std::endl;
        if ( set_pose ( "camera_start" ) )
        {
          // step 2, start services rough_localizer, and box_segmenter
          publish_msg ( "mid_referencing_start" );
          std::cout << "2, start services rough_localizer, box_segmenter" << std::endl;
          if ( start_rough_localizer_.call ( msg ) && start_box_segmenter_.call ( msg ) )
          {
            // step 3, start service move_camera
            std::cout << "3, start to move the camera" << std::endl;
            start_move_camera_.call ( msg );
            // step 4, stop services rough_localizer and box_segmenter
            std::cout << "call stop_image_transport_" << std::endl;
            stop_image_transport_.call ( msg );
            std::cout << "4, stop services rough_localizer, box_segmenter" << std::endl;
            if ( stop_rough_localizer_.call ( msg ) && stop_box_segmenter_.call ( msg ) )
            {
              set_pose ( "camera_start" );
              set_pose ( "scan_start" );
              // step 5, generate scanning plans and write it to the configuration file [do_scan]
              std::cout << "5, start to generate scanning plans" << std::endl;
              start_scan_planner_.call ( msg );
            }
          }
        }
        publish_msg ( "mid_referencing_end" );
        break;
      }
      case 1:
      {
        publish_msg ( "fine_referencing_start" );
        // step 6, start one profile scan
        std::cout << "6, start profile scanning" << std::endl;
        start_do_scan_.call ( msg );

        // step 7, start profile scan
        std::cout << "7, Saving profile point cloud" << std::endl;
        save_profile_pc ();
        publish_msg ( "fine_referencing_end" );
        break;
      }
      case 2:
      {
        if ( scene_cloud_->size() == 0 )
        {
          return;
        }
        publish_msg ( "path_planning_start" );
        // step 8, call the service rivet_localizer
        std::cout << "8, start rivet localizer" << std::endl;
        start_rivet_localizer_.call ( msg );
        publish_msg ( "path_planning_end" );
        break;
      }
      case 3:
      {
        publish_msg ( "collar_screwing_start" );
        if ( scene_cloud_->size() > 0 )
        {
          // step 9, call the service point_rivet
          std::cout << "9, start to point to rivet" << std::endl;
          start_point_rivet_.call ( msg );
        }
        // step 10, set the robot pose back to pose screw_start
        std::cout << "10, set the robot pose back to pose screw_start" << std::endl;
        set_pose ( "screw_start" );
        publish_msg ( "collar_screwing_end" );
        break;
      }
    }
  }

  bool pause_execution ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    is_pause = true;
    return true;
  }

  bool restart_execution ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    execute_pipeline_new ();
    return true;
  }

  void execute_pipeline_new ()
  {
    is_pause = false;
    for ( int idx = current_execution_phase; idx < 4; idx ++ )
    {
      execute_stage ( idx );
      current_execution_phase ++;
      if ( is_pause )
      {
        break;
      }
    }
  }

  void start_execution ()
  {
    current_execution_phase = 0;
    execute_pipeline_new ();
  }

  void execute_pipeline ()
  {
    // step 1, set the robot pose to camera_start.
    std_srvs::Empty msg;
    std::cout << "1, set the robot pose to camera_start" << std::endl;
    if ( set_pose ( "camera_start" ) )
    {
      // step 2, start services rough_localizer, and box_segmenter
      std::cout << "2, start services rough_localizer, box_segmenter" << std::endl;
      if ( start_rough_localizer_.call ( msg ) && start_box_segmenter_.call ( msg ) )
      {
        // step 3, start service move_camera
        std::cout << "3, start to move the camera" << std::endl;
        start_move_camera_.call ( msg );
        // step 4, stop services rough_localizer and box_segmenter
        std::cout << "4, stop services rough_localizer, box_segmenter" << std::endl;
        if ( stop_rough_localizer_.call ( msg ) && stop_box_segmenter_.call ( msg ) )
        {
          set_pose ( "camera_start" );
          set_pose ( "scan_start" );
          // step 5, generate scanning plans and write it to the configuration file [do_scan]
          std::cout << "5, start to generate scanning plans" << std::endl;
          start_scan_planner_.call ( msg );

          // step 6, start one profile scan
          std::cout << "6, start profile scanning" << std::endl;
          start_do_scan_.call ( msg );

          // step 7, start profile scan
          std::cout << "7, Saving profile point cloud" << std::endl;
          save_profile_pc ();

          // step 8, call the service rivet_localizer
          std::cout << "8, start rivet localizer" << std::endl;
          start_rivet_localizer_.call ( msg );

          // step 9, call the service point_rivet
          std::cout << "9, start to point to rivet" << std::endl;
          start_point_rivet_.call ( msg );

          // step 10, set the robot pose back to pose screw_start
          std::cout << "10, set the robot pose back to pose screw_start" << std::endl;
          set_pose ( "screw_start" );
        }
      }
    }
  }

};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "control_node_b" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ControlNode control_node;
  control_node.start_execution ();
  ros::waitForShutdown ();
  return 0;
}
