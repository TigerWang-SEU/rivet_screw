#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include "object_localizer_msg/BBox_int.h"
#include "object_localizer_msg/BBox_float.h"
#include "object_localizer_msg/BBox_list.h"
#include "object_localizer_msg/Segment_list.h"

#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "head/planner.h"
#include "head/PCL_name.h"

std::string reference_frame = "world";
float x_adjust = -0.01; // adjustment for the x poistion
float scan_distance = 0.075; // set the distance to the scanning part
float back_distance = 0.120; // backward distance
float scan_length = 0.10; // scanning path length
float s_scale = 0.2; // scale of the start scanning scan_length
float e_scale = 1.1; // scale of the end scanning scan_length

void read_scan_planner_cfg_file ( )
{
  std::string scan_planner_cfg_file = ros::package::getPath ( "object_localizer" ) + "/config/scan_planner.cfg";
  std::cout << "***Reading the configuration file scan_planner at [" << scan_planner_cfg_file << "]" << std::endl;

  std::ifstream input ( scan_planner_cfg_file );
  std::string line;
  if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> x_adjust;
  }
  if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> scan_distance;
  }
  if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> back_distance;
  }
  if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> scan_length;
  }
  if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> s_scale;
  }
  if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> e_scale;
  }
  input.close();
}

float calculate_theta ( PointCloudT::ConstPtr cloudSegmented, Eigen::Vector3f& central_point )
{
  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid ( *cloudSegmented, pcaCentroid );
  central_point = pcaCentroid.head< 3 >();

  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized ( *cloudSegmented, pcaCentroid, covariance );

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver ( covariance, Eigen::ComputeEigenvectors );
	Eigen::Vector3f eigenvaluesPCA = eigen_solver.eigenvalues ();
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors ();

	int max_idx_1_r, max_idx_2_r, max_idx_3_r, max_idx_1_c, max_idx_2_c, max_idx_3_c;
	eigenVectorsPCA.block ( 0, 0, 3, 1 ).cwiseAbs().maxCoeff( &max_idx_1_r, &max_idx_1_c );
	eigenVectorsPCA.block ( 0, 1, 3, 1 ).cwiseAbs().maxCoeff( &max_idx_2_r, &max_idx_2_c );
	eigenVectorsPCA.block ( 0, 2, 3, 1 ).cwiseAbs().maxCoeff( &max_idx_3_r, &max_idx_3_c );
  std::cout << "*** eigen value 1 = [" << eigenvaluesPCA (0) << "] ***\n\t eigen vector 1: [" << eigenVectorsPCA.block ( 0, 0, 3, 1 ).transpose () << "] \n\t max_idx = " << max_idx_1_r << std::endl;
  std::cout << "*** eigen value 2 = [" << eigenvaluesPCA (1) << "] ***\n\t eigen vector 2: [" << eigenVectorsPCA.block ( 0, 1, 3, 1 ).transpose () << "] \n\t max_idx = " << max_idx_2_r << std::endl;
  std::cout << "*** eigen value 3 = [" << eigenvaluesPCA (2) << "] ***\n\t eigen vector 3: [" << eigenVectorsPCA.block ( 0, 2, 3, 1 ).transpose () << "] \n\t max_idx = " << max_idx_3_r << std::endl;

  float x_tmp, y_tmp, z_tmp;
  if ( max_idx_3_r != 0 )
  {
    x_tmp = eigenVectorsPCA ( 0, 2 );
    y_tmp = eigenVectorsPCA ( 1, 2 );
    z_tmp = eigenVectorsPCA ( 2, 2 );
  }
  else
  {
    x_tmp = eigenVectorsPCA ( 0, 1 );
    y_tmp = eigenVectorsPCA ( 1, 1 );
    z_tmp = eigenVectorsPCA ( 2, 1 );
  }
  // scale_vector ( x_tmp, y_tmp, z_tmp, 0.2 );
  float y_new, z_new;
  y_new = - z_tmp;
  z_new = y_tmp;

  // show_arrow ( get_id (), pcaCentroid ( 0 ),  pcaCentroid ( 1 ), pcaCentroid ( 2 ), x_tmp, y_new, z_new );
  std::cout << "[y_tmp, z_tmp] = [" << y_tmp << ", " << z_tmp << "]" << std::endl;

  float theta = 0.0;
  float z_avg = central_point ( 2 );
  float y_avg = central_point ( 1 );
  if ( z_avg > tableheight )
  {
    theta = atan2 ( z_new, y_new ) * 180.0 / M_PI;
    if ( theta < 0 )
    {
      theta = theta + 180;
    }
  }
  else
  {
    if ( y_avg > 0 )
    {
      if ( y_new < 0 )
      {
        y_new = -y_new;
        z_new = -z_new;
      }
      theta = atan2 ( z_new, y_new ) * 180.0 / M_PI;
    }
    else
    {
      if ( y_new > 0 )
      {
        y_new = -y_new;
        z_new = -z_new;
      }
      theta = atan2 ( z_new, y_new ) * 180.0 / M_PI;
      if ( theta < 0 )
      {
        theta = theta + 360;
      }
    }
  }
  return theta;
}

class ScanPlanner
{
public:

  void segment_list_cb ( const object_localizer_msg::BBox_list::ConstPtr& segment_list_in )
  {
    if ( segment_list_in->BBox_list_float.size() > 0 )
    {
      segment_list.reset ( new object_localizer_msg::Segment_list () );
      int bbox_idx = 0;
			std::cout << "BBox_list_float.size = " << segment_list_in->BBox_list_float.size() << " Segment_list.size = " << segment_list_in->Segment_list.size() << std::endl;
      for ( object_localizer_msg::BBox_float bbox : segment_list_in->BBox_list_float )
      {
        segment_list->BBox_list_float.push_back ( bbox );
        segment_list->Segment_list.push_back ( segment_list_in->Segment_list [ bbox_idx ] );
        bbox_idx ++;
      }
    }
  }

  void show_half_pc ( PointCloudT::Ptr left_half_pc,  PointCloudT::Ptr right_half_pc )
  {
    left_half_pc->header.frame_id = reference_frame;
    pcl_conversions::toPCL ( ros::Time::now(), left_half_pc->header.stamp );
    left_cloud_pub_.publish ( left_half_pc );
    right_half_pc->header.frame_id = reference_frame;
    pcl_conversions::toPCL ( ros::Time::now(), right_half_pc->header.stamp );
    right_cloud_pub_.publish ( right_half_pc );
  }

  bool start_scan_planner ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    read_scan_planner_cfg_file ();
    std::cout << "[x_adjust, scan_distance, back_distance, scan_length, s_scale, e_scale] = ["<< x_adjust << ", " << scan_distance << ", " << back_distance << ", " << scan_length << ", " << s_scale << ", " << e_scale << "]"<< std::endl;

    if ( segment_list->BBox_list_float.size() > 0 )
    {
      PointCloudT::Ptr left_half_pc ( new PointCloudT ), right_half_pc ( new PointCloudT );
      check_boundary ( segment_list, left_half_pc, right_half_pc );
      show_half_pc ( left_half_pc, right_half_pc );
      std::string boundary = read_boundary_file ();
      if ( boundary == "right" )
      {
        x_adjust -= 0.02;
      }
      else
      {
        x_adjust -= 0.01;
      }

      ofstream do_scan_fs;
      std::string cfgFileName = ros::package::getPath ( "motion_control" ) + "/config/scan_plan.cfg";
      do_scan_fs.open ( cfgFileName );

      // iterate through all segments
			int bbox_idx = 0;
      for ( object_localizer_msg::BBox_float bbox : segment_list->BBox_list_float )
      {
        std::cout << "Bounding box [" << bbox_idx << "] has [x1, x2, y1, y2]: [" << bbox.x1 << "," << bbox.x2 << "," << bbox.y1 << "," << bbox.y2  << "]" << std::endl;

        // step 1, convert a segment message to a point cloud.
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL ( segment_list->Segment_list [ bbox_idx ], pcl_pc2 );
        PointCloudT::Ptr segment_cloud ( new PointCloudT );
        pcl::fromPCLPointCloud2 ( pcl_pc2, *segment_cloud );
        int color_r = 0, color_g = 0, color_b = 0;
        color_r = bbox_idx % 5 * 50;
        color_g = bbox_idx % 3 * 80;
        color_b = bbox_idx % 7 * 30;
        if ( bbox_idx == 0 )
        {
          color_r = 255;
          color_g = 255;
          color_b = 255;
        }
        bbox_idx ++;

        // step 2, filter out point clouds which is not longer than 0.08
        float min_x, max_x;
        PointT minPt, maxPt;
        getMinMax3D ( *segment_cloud, minPt, maxPt );
        std::cout << "Min [x, y, z]: = [" << minPt.x << ", " << minPt.y << ", " << minPt.z << "]" << std::endl;
        min_x = minPt.x;
        std::cout << "Max [x, y, z]: = [" << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "]" << std::endl;
        max_x = maxPt.x;

        float distance_ = std::sqrt ( std::pow ( ( maxPt.y - minPt.y ), 2 ) + std::pow ( ( maxPt.z - minPt.z ), 2 ) );
        std::cout << "### distance = [" << distance_ << "]" << std::endl;
        if ( distance_  < 0.08 )
        {
          continue;
        }

        // step 2.1, filter the segment point cloud
        PointCloudT::Ptr filtered_segment_cloud	( new PointCloudT );
        for ( PointT temp_point: segment_cloud->points )
        {
          float x = temp_point.x;
          float y = temp_point.y;
          float z = temp_point.z;
          if ( boundary == "right" )
          {
            if ( ( x - min_x ) / ( max_x - min_x ) < 0.6 )
            {
              continue;
            }
          }
          else
          {
            if ( ( x - min_x ) / ( max_x - min_x ) > 0.4 )
            {
              continue;
            }
          }
          PointT new_point;
          new_point.x = x;
          new_point.y = y;
          new_point.z = z;
          filtered_segment_cloud->points.push_back( new_point );
        }
        segment_cloud = filtered_segment_cloud;

        // step 3, find the central point of the segment point cloud
        Eigen::Vector3f central_point, scan_start_point, scan_end_point, scan_back_point;
        float theta = calculate_theta ( segment_cloud, central_point );
        std::cout << std::endl << "*** Rotation around x is [" << theta << "] degrees" << std::endl << "*** central_point = " << central_point.transpose () << std::endl;
        calculate_start_end_point ( central_point, scan_start_point, scan_end_point, scan_back_point, theta, x_adjust, scan_distance, scan_length, s_scale, e_scale, back_distance );

        // step 5, write scanning plannings into the scanning plan file.
        do_scan_fs << theta << " " << scan_start_point.transpose () << " " << scan_end_point.transpose () << " " << scan_back_point.transpose () << " " << color_r << " " << color_g << " " << color_b << std::endl;
      }
    	do_scan_fs.close();
    }
    return true;
  }

  ScanPlanner ()
  {
    start_scan_planner_ = nh_.advertiseService ( "start_scan_planner", &ScanPlanner::start_scan_planner, this );
    ros::Duration ( 0.5 ) .sleep ();

    std::string segment_list_in_name = "/rough_localizer/bbox_list";
    segment_list_sub_ = nh_.subscribe ( segment_list_in_name, 10, &ScanPlanner::segment_list_cb, this );
    ROS_INFO_STREAM ( "Listening for segment list on topic: " << segment_list_in_name );

    left_cloud_pub_ = nh_.advertise < pcl::PointCloud < PointT > > ( "/planner_h/left_half_pc", 30 );
    right_cloud_pub_ = nh_.advertise < pcl::PointCloud < PointT > > ( "/planner_h/right_half_pc", 30 );
  }

  ~ScanPlanner () { }

private:
  ros::NodeHandle nh_;
  object_localizer_msg::Segment_list::Ptr segment_list;
  ros::ServiceServer start_scan_planner_;
  ros::Subscriber segment_list_sub_;
  ros::Publisher left_cloud_pub_, right_cloud_pub_;
};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "scan_planner_b" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ScanPlanner SP;
  ros::waitForShutdown ();
  return 0;
}
