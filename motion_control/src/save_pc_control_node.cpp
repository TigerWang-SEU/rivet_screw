#include <cmath>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>

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

  ros::ServiceClient start_pcl_merger_, stop_pcl_merger_, start_move_camera_;

  boost::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group;
  boost::shared_ptr< moveit::planning_interface::PlanningSceneInterface > planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group;

  ros::NodeHandle nh_;
  pcl::PointCloud<PointT>::Ptr scene_cloud_;
  ros::Subscriber cloud_sub_;
  pcl::PLYWriter writer;

public:

  ControlNode () : scene_cloud_ ( new pcl::PointCloud< PointT > )
  {
    // pcl_merger, rough_localizer, box_segmenter, scan_planner and move_camera
    start_pcl_merger_ = nh_.serviceClient < std_srvs::Empty > ( "start_pcl_merge" );
    stop_pcl_merger_ = nh_.serviceClient < std_srvs::Empty > ( "stop_pcl_merge" );
    start_move_camera_ = nh_.serviceClient < std_srvs::Empty > ( "start_move_camera" );

    move_group.reset ( new moveit::planning_interface::MoveGroupInterface ( PLANNING_GROUP ) );
    planning_scene_interface.reset ( new moveit::planning_interface::PlanningSceneInterface () );
    joint_model_group = move_group->getCurrentState()->getJointModelGroup ( PLANNING_GROUP );
    ROS_INFO_NAMED ( "control_node", "Reference frame: %s", move_group->getPlanningFrame().c_str() );
    ROS_INFO_NAMED ( "control_node", "End effector link: %s", move_group->getEndEffectorLink().c_str() );

    std::string cloud_in_name = "/point_cloud_merger/points";
    cloud_sub_ = nh_.subscribe ( cloud_in_name, 3, &ControlNode::cloud_cb, this );
    ROS_INFO_STREAM ( "Listening point cloud message on topic " << cloud_in_name );
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

  bool save_pc ()
  {
    std::string pc_file_path = ros::package::getPath ( "object_localizer" )+ "/data/pcl_out_0.ply";
		std::cout << "\tSaving point cloud to file: \n\t" << pc_file_path << std::endl;
		writer.write ( pc_file_path, *scene_cloud_ );
  }

  void execute_pipeline ()
  {

    // step 1, set the robot pose to pose.
    std_srvs::Empty msg;
    std::cout << "1, set the robot pose to scan_start" << std::endl;
    if ( set_pose ( "scan_start" ) )
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

        // step 5, start profile scan
        std::cout << "5, Saving profile point cloud" << std::endl;
        save_pc ();
      }
    }
  }

};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "save_pc_control_node" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ControlNode control_node;
  control_node.execute_pipeline ();
  ros::waitForShutdown ();
  return 0;
}
