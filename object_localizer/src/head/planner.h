#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <float.h>

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

#include "PCL_name.h"

#ifndef PLANNER_H
#define PLANNER_H

std::string boundary_file_name = ros::package::getPath ( "motion_control" ) + "/config/boundary.cfg";
float tableheight = 0.855;

void write_boundary_file ( std::string in_string )
{
  std::cout << "***Writing boundary_file [" << boundary_file_name << "]" << std::endl;
  ofstream boundary_fs;
  boundary_fs.open ( boundary_file_name );
  boundary_fs << in_string << std::endl;
  boundary_fs.close();
}

std::string read_boundary_file ()
{
  std::cout << "***Reading boundary_file [" << boundary_file_name << "]" << std::endl;
  std::ifstream input ( boundary_file_name );
  std::string line;
  std::getline ( input, line );
  input.close();
  return line;
}

void check_boundary ( object_localizer_msg::Segment_list::Ptr segment_list, PointCloudT::Ptr left_half_pc, PointCloudT::Ptr right_half_pc )
{
  int left_boundary_counter = 0, right_boundary_counter = 0;
  for ( sensor_msgs::PointCloud2 segment_cloud_pc : segment_list->Segment_list )
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL ( segment_cloud_pc, pcl_pc2 );
    PointCloudT::Ptr segment_cloud ( new PointCloudT );
    pcl::fromPCLPointCloud2 ( pcl_pc2, *segment_cloud );

    PointT minPt, maxPt;
    getMinMax3D ( *segment_cloud, minPt, maxPt );
    std::cout << "Min [x, z]: = [" << minPt.x << ", " << minPt.z << "]" << std::endl;
    std::cout << "Max [x, z]: = [" << maxPt.x << ", " << maxPt.z << "]" << std::endl;

    if ( ( maxPt.z - minPt.z ) > 0.05 && ( maxPt.z - tableheight ) < 1.5 )
    {
      double left_y_max = -DBL_MAX, right_y_max = -DBL_MAX;
      for ( PointT temp_point: segment_cloud->points )
      {
        float x = temp_point.x;
        float y = temp_point.y;
        float z = temp_point.z;
        if ( ( x - minPt.x ) / ( maxPt.x - minPt.x ) < 0.5 )
        {
          if ( right_y_max < y )
          {
            right_y_max = y;
          }
          PointT new_point;
          new_point.x = x;
          new_point.y = y;
          new_point.z = z;
          right_half_pc->points.push_back( new_point );
        }
        else if ( ( x - minPt.x ) / ( maxPt.x - minPt.x ) > 0.5 )
        {
          if ( left_y_max < y )
          {
            left_y_max = y;
          }
          PointT new_point;
          new_point.x = x;
          new_point.y = y;
          new_point.z = z;
          left_half_pc->points.push_back( new_point );
        }
      }

      std::cout << "left_y_max = " << left_y_max << ", right_y_max = " << right_y_max << std::endl;

      if ( right_y_max - left_y_max > 0.01 )
      {
        right_boundary_counter ++;
      }
      else if ( left_y_max - right_y_max > 0.01 )
      {
        left_boundary_counter ++;
      }
    }
  }

  std::cout << "left_boundary_counter = " << left_boundary_counter << ", right_boundary_counter = " << right_boundary_counter << std::endl;
  if ( right_boundary_counter > left_boundary_counter )
  {
    write_boundary_file ( "right" );
  }
  else
  {
    write_boundary_file ( "left" );
  }
}

void calculate_start_end_point ( Eigen::Vector3f& central_point, Eigen::Vector3f& scan_start_point, Eigen::Vector3f& scan_end_point, Eigen::Vector3f& scan_back_point, float theta, float x_adjust, float scan_distance, float scan_length, float s_scale, float e_scale, float back_distance )
{
  float x_0 = central_point ( 0 );
  float y_0 = central_point ( 1 );
  float z_0 = central_point ( 2 );

  float theta_tmp = ( theta - 90.0 ) * M_PI / 180.0;
  float x_tmp = x_0 + x_adjust;
  float y_tmp = y_0 + scan_distance * std::sin ( theta_tmp );
  float z_tmp = z_0 - scan_distance * std::cos ( theta_tmp );

  scan_start_point ( 0 ) = x_tmp;
  scan_start_point ( 1 ) = y_tmp - scan_length * std::cos ( theta_tmp ) * s_scale;
  scan_start_point ( 2 ) = z_tmp - scan_length * std::sin ( theta_tmp ) * s_scale;
  std::cout << "[***] Scan start point is [x, y, z] = [" << scan_start_point.transpose () << "]" << std::endl;

  scan_end_point ( 0 ) = x_tmp;
  scan_end_point ( 1 ) = y_tmp + scan_length * std::cos ( theta_tmp ) * e_scale;
  scan_end_point ( 2 ) = z_tmp + scan_length * std::sin ( theta_tmp ) * e_scale;
  std::cout << "[***] Scan end point is [x, y, z] = [" << scan_end_point.transpose () << "]" << std::endl << std::endl;

  scan_back_point ( 0 ) = x_0 + x_adjust;
  scan_back_point ( 1 ) = y_0 + ( scan_distance + back_distance ) * std::sin ( theta_tmp );
  scan_back_point ( 2 ) = z_0 - ( scan_distance + back_distance ) * std::cos ( theta_tmp );
}

#endif
