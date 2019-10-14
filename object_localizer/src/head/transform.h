#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <limits>
#include <boost/algorithm/string/predicate.hpp>
#include <sstream>
#include <math.h>
#include <cmath>
#include <vector>
#include <array>
#include <list>
#include <set>
#include <map>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common_headers.h>

#ifndef TRANSFORM_H
#define TRANSFORM_H

void getInverseMatrix ( Eigen::Matrix4f& original_transform, Eigen::Matrix4f& inverse_transform )
{
  inverse_transform.block< 3, 3 >( 0, 0 ) = original_transform.block< 3, 3 >( 0, 0 ).transpose();
  inverse_transform.block< 3, 1 >( 0, 3 ) = -1.0f * ( inverse_transform.block< 3, 3 >( 0, 0 ) * original_transform.block< 3, 1 >( 0, 3 ) );
}

void get_rpy_from_matrix ( Eigen::Matrix4f rotation_matrix, double& roll, double& pitch, double& yaw )
{
  tf::Matrix3x3 object_m ( rotation_matrix (0, 0), rotation_matrix (0, 1), rotation_matrix (0, 2), rotation_matrix (1, 0), rotation_matrix (1, 1), rotation_matrix (1, 2), rotation_matrix (2, 0), rotation_matrix (2, 1), rotation_matrix (2, 2) );
  object_m.getRPY ( roll, pitch, yaw );
}

void get_matrix_from_rpy ( Eigen::Matrix4f& rotation_matrix, double roll, double pitch, double yaw )
{
  rotation_matrix << cos(yaw)*cos(pitch),
                     cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),  cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll), 0,
                     sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), 0,
                    -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), 0,
                              0,                    0,                    0, 1;
}

#endif
