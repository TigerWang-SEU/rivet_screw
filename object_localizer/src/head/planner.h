#ifndef PLANNER_H
#define PLANNER_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <float.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "object_localizer_msg/BBox_int.h"
#include "object_localizer_msg/BBox_float.h"
#include "object_localizer_msg/BBox_list.h"
#include "object_localizer_msg/Segment_list.h"

#include "PCL_name.h"
#include "rviz_show.h"

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

// function for calculating rotation around the x axis
float calculate_theta ( PointCloudT::ConstPtr cloudSegmented, Eigen::Vector3f& central_point )
{
  // step 1, get min_x and max_x
  float min_x, max_x;
  PointT minPt, maxPt;
  getMinMax3D ( *cloudSegmented, minPt, maxPt );
  min_x = minPt.x;
  show_point ( get_id (), minPt.x, minPt.y, minPt.z );
  std::cout << "\tmin_x = " << min_x << std::endl;
  max_x = maxPt.x;
  show_point ( get_id (), maxPt.x, maxPt.y, maxPt.z );
  std::cout << "\tmax_x = " << max_x << std::endl;

  // step 2, filter the segment point cloud
  std::string boundary = read_boundary_file ();
  PointCloudT::Ptr filtered_segment_cloud	( new PointCloudT );
  float y_avg = 0, z_avg = 0;
  for ( PointT temp_point: cloudSegmented->points )
  {
    float x = temp_point.x;
    float y = temp_point.y;
    float z = temp_point.z;
    if ( boundary == "right" )
    {
      if ( ( x - min_x ) / ( max_x - min_x ) < 0.5 )
      {
        continue;
      }
    }
    else
    {
      if ( ( x - min_x ) / ( max_x - min_x ) > 0.5 )
      {
        continue;
      }
    }
    PointT new_point;
    new_point.x = x;
    new_point.y = y;
    y_avg += y;
    new_point.z = z;
    z_avg += z;
    filtered_segment_cloud->points.push_back( new_point );
  }
  y_avg = y_avg / filtered_segment_cloud->points.size();
  z_avg = z_avg / filtered_segment_cloud->points.size();

  // step 3, compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid ( *filtered_segment_cloud, pcaCentroid );
  central_point = pcaCentroid.head< 3 >();

  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized ( *filtered_segment_cloud, pcaCentroid, covariance );

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver ( covariance, Eigen::ComputeEigenvectors );
  Eigen::Vector3f eigenvaluesPCA = eigen_solver.eigenvalues ();
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors ();

  // get eigen vectors and find the maximum value column for each eigen vector
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
  scale_vector ( x_tmp, y_tmp, z_tmp, 0.2 );
  float y_new, z_new;
  y_new = - z_tmp;
  z_new = y_tmp;

  show_arrow ( get_id (), pcaCentroid ( 0 ),  pcaCentroid ( 1 ), pcaCentroid ( 2 ), x_tmp, y_new, z_new );
  std::cout << "[y_tmp, z_tmp] = [" << y_tmp << ", " << z_tmp << "]" << std::endl;
  //###################### need more test #######################
  float theta = 0.0;
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

#endif
