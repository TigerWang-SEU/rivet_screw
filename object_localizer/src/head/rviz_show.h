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

#include "reference_frame.h"
#include "transform.h"

#ifndef RVIZ_SHOW_H
#define RVIZ_SHOW_H

ros::Publisher vis_pub;

int get_id ()
{
  static int count = -1;
  count = ( count + 1 ) % 10000;
  return count;
}

void scale_vector ( float& x_d, float& y_d, float& z_d, float scale = 0.15 )
{
  float vector_scale = std::pow ( std::pow ( x_d, 2 ) + std::pow ( y_d, 2 ) + std::pow ( z_d, 2 ), 0.5 );
  x_d = x_d / vector_scale * scale;
  y_d = y_d / vector_scale * scale;
  z_d = z_d / vector_scale * scale;
}

void show_point ( int id, double x, double y, double z )
{
  visualization_msgs::Marker points;
  points.header.frame_id = reference_frame;
  points.header.stamp = ros::Time ();
  points.ns = "my_arrow";
  points.id = id;
  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point p_start;
  p_start.x = x;
  p_start.y = y;
  p_start.z = z;
  points.points.push_back ( p_start );
  points.scale.x = 0.0035;
  points.scale.y = 0.0035;
  points.color.r = 1.0;
  points.color.a = 1.0;
  vis_pub.publish( points );
}

void show_arrow ( int id, double x, double y, double z, double x_d, double y_d, double z_d, float r_in = 0.0, float g_in = 1.0, float b_in = 0.0 )
{
  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = reference_frame;
  points.header.stamp = line_strip.header.stamp = ros::Time ();
  points.ns = line_strip.ns = "my_arrow";
  points.id = id + 10000;
  line_strip.id = id;
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point p_start;
  p_start.x = x;
  p_start.y = y;
  p_start.z = z;
  line_strip.points.push_back ( p_start );
  geometry_msgs::Point p_end;
  p_end.x = x + x_d;
  p_end.y = y + y_d;
  p_end.z = z + z_d;
  line_strip.points.push_back ( p_end );
  points.points.push_back ( p_end );
  points.scale.x = 0.0035;
  points.scale.y = 0.0035;
  points.color.r = 1.0;
  points.color.g = 0.5;
  points.color.a = 1.0;
  line_strip.scale.x = 0.002;
  line_strip.color.a = 1.0;
  line_strip.color.r = r_in;
  line_strip.color.g = g_in;
  line_strip.color.b = b_in;
  vis_pub.publish( points );
  vis_pub.publish( line_strip );
}

void show_transformation ( Eigen::Matrix4f transform )
{
  Eigen::Matrix4f transform_inverse ( Eigen::Matrix4f::Identity () );
  getInverseMatrix ( transform, transform_inverse );
  float x = transform_inverse ( 0, 3 );
  float y = transform_inverse ( 1, 3 );
  float z = transform_inverse ( 2, 3 );
  float x_d = transform ( 0, 0 ), y_d = transform ( 0, 1 ), z_d = transform ( 0, 2 );
  scale_vector ( x_d, y_d,  z_d );
  show_arrow ( get_id (), x, y, z, x_d, y_d, z_d, 1.0, 0.0, 0.0 );
  x_d = transform ( 1, 0 ), y_d = transform ( 1, 1 ), z_d = transform ( 1, 2 );
  scale_vector ( x_d, y_d,  z_d );
  show_arrow ( get_id (), x, y, z, x_d, y_d, z_d, 0.0, 1.0, 0.0 );
  x_d = transform ( 2, 0 ), y_d = transform ( 2, 1 ), z_d = transform ( 2, 2 );
  scale_vector ( x_d, y_d,  z_d );
  show_arrow ( get_id (), x, y, z, x_d, y_d, z_d, 0.0, 0.0, 1.0 );
}

void show_frame ( std::string frame_name, double x, double y, double z, double roll, double pitch, double yaw )
{
  static tf2_ros::StaticTransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	// create a frame for each object
  transformStamped.header.stamp = ros::Time::now ();
  transformStamped.header.frame_id = reference_frame;
  transformStamped.child_frame_id = frame_name;
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transformStamped.transform.rotation.x = q.x ();
  transformStamped.transform.rotation.y = q.y ();
  transformStamped.transform.rotation.z = q.z ();
  transformStamped.transform.rotation.w = q.w ();
  ros::Rate loop_rate ( 100 );
  for ( int i = 0; i < 3; i++ )
	{
      br.sendTransform ( transformStamped );
      loop_rate.sleep ();
  }
  std::cout << "\tFrame " << frame_name << " is added" << std::endl;
}

#endif
