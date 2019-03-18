#include <iostream>
#include <cmath>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#define PI 3.14159265
bool is_fix_table_position = false;
float point1_x = 0.0;
float point1_y = 0.0;
float point2_x = 0.0;
float point2_y = 0.0;
float point1_x_o = 1000000.0;
float point1_y_o = 1000000.0;
float point2_x_o = 1000000.0;
float point2_y_o = 1000000.0;

float a_f_offset = 156; //cm
float scan_offset = 40; //cm
float old_x_t = 10000.0; //cm
float old_y_t = 10000.0; //cm
float old_theta2 = 100.0;

void publish_tf ( float x_in, float y_in, float theta2_in )
{
  // publish transform between table and floor
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( x_in, -y_in, 0.0 ) );
  tf::Quaternion q;
  q.setRPY ( 0, 0, theta2_in );
  transform.setRotation ( q );
  br.sendTransform ( tf::StampedTransform ( transform, ros::Time::now(), "floor", "table" ) );
}

void get_the_central_point()
{
  // from meter to millimeter
  float x1 = std::round ( point1_x * 100.0 );
  if ( point1_x_o == 1000000.0 )
  {
    point1_x_o = x1;
  } else
  {
    x1 = ( point1_x_o + x1 ) / 2.0;
    point1_x_o = x1;
  }
  float y1 = std::round ( point1_y * 100.0 );
  if ( point1_y_o == 1000000.0 )
  {
    point1_y_o = y1;
  } else
  {
    y1 = ( point1_y_o + y1 ) / 2.0;
    point1_y_o = y1;
  }
  float x2 = std::round ( point2_x * 100.0 );
  if ( point2_x_o == 1000000.0 )
  {
    point2_x_o = x2;
  } else
  {
    x2 = ( point2_x_o + x2 ) / 2.0;
    point2_x_o = x2;
  }
  float y2 = std::round ( point2_y * 100.0 );
  if ( point2_y_o == 1000000.0 )
  {
    point2_y_o = y2;
  } else
  {
    y2 = ( point2_y_o + y2 ) / 2.0;
    point2_y_o = y2;
  }
  float dist_p1_p2 = std::sqrt ( std::pow ( x1 - x2, 2 ) + std::pow ( y1 - y2, 2 ) );
  // ROS_INFO_STREAM ( "Point1 (p1): [" << x1 << ", " << y1 << "], point2 (p2): [" << x2 << ", " << y2 << "]; Dist(p1, p2):" << dist_p1_p2 );

  if ( std::abs ( dist_p1_p2 - 88.0 ) > 3 )
  {
    // ROS_INFO_STREAM ( "Point1 (p1): [" << x1 << ", " << y1 << "], point2 (p2): [" << x2 << ", " << y2 << "]; Dist(p1, p2):" << dist_p1_p2 );
    // ROS_ERROR_STREAM ( "ERROR -> The robot needs to face the aircraft_frame and have no people in the front." );
    return;
  }

  // start to calculate the central point of the table w.r.t the world frame
  float v_p1_p2_x = x2 - x1;
  float v_p1_p2_y = y2 - y1;
  float v_p1_s_x = 0.0 - x1;
  float v_p1_s_y = 0.0 - y1;
  float dist_p1_sp = ( v_p1_s_x * v_p1_p2_x + v_p1_s_y * v_p1_p2_y ) / dist_p1_p2;
  float v_p1_sp_x = dist_p1_sp * ( v_p1_p2_x / dist_p1_p2 );
  float v_p1_sp_y = dist_p1_sp * ( v_p1_p2_y / dist_p1_p2 );
  float v_s_sp_x = v_p1_s_x - v_p1_sp_x;
  float v_s_sp_y = v_p1_s_y - v_p1_sp_y;
  double theta = std::atan2( v_s_sp_y, v_s_sp_x );
  double theta2 = ( PI / 2.0 ) - theta;
  if ( old_theta2 == 100.0 )
  {
    old_theta2 = theta2;
  }
  double x_offset = std::sin( theta2 ) * scan_offset;
  double y_offset = std::cos( theta2 ) * scan_offset;
  // std::cout << "Degree = " << theta * 180 / PI << " [x_offset, y_offset] = [" << x_offset << ", " << y_offset << "]" << std::endl;

  double dist_s_sp = std::sqrt( std::pow( v_s_sp_x, 2 ) + std::pow( v_s_sp_y, 2 ) );
  double dist_sp_0 = 0.0;
  if ( dist_p1_sp > dist_p1_p2 / 2.0 )
  {
    dist_sp_0 = ( dist_p1_sp - ( dist_p1_p2 / 2.0 ) );
  }  else
  {
    dist_sp_0 = - ( ( dist_p1_p2 / 2.0 ) - dist_p1_sp );
  }
  float x_t = (float) (dist_sp_0 - x_offset);
  float y_t = (float) (a_f_offset - dist_s_sp - y_offset);

  x_t = x_t / 100.0;
  y_t = y_t / 100.0;

  if ( old_x_t == 10000.0 )
  {
    old_x_t = x_t;
  }

  if ( old_y_t == 10000.0 )
  {
    old_y_t = y_t;
  }

  if ( std::abs( old_x_t - x_t ) > 0.05 || std::abs( old_y_t - y_t ) > 0.05 || std::abs( old_theta2 - theta2 ) * 180 / PI > 2 )
  {
    old_x_t = x_t;
    old_y_t = y_t;
    old_theta2 = theta2;
    std::cout << ros::Time::now() << ": publish new [x, y, theta] = [" << old_x_t << ", " << old_y_t << ", " << old_theta2 << "]" << std::endl;
  } else
  {
    std::cout << ros::Time::now() << ": publish old [x, y, theta] = [" << old_x_t << ", " << old_y_t << ", " << old_theta2 << "]" << std::endl;
  }
  publish_tf ( old_x_t, old_y_t, old_theta2 );
}

laser_geometry::LaserProjection projector_;
ros::Publisher pub;
int K = 6;

void scanCallback ( const sensor_msgs::LaserScan::ConstPtr& scan_in )
{
  // convert from laserscan to pcl point cloud
  sensor_msgs::PointCloud2 sensor_pc2;
  projector_.projectLaser( *scan_in, sensor_pc2 );

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL( sensor_pc2, pcl_pc2 );

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud( new pcl::PointCloud< pcl::PointXYZ > );
  pcl::fromPCLPointCloud2( pcl_pc2, *temp_cloud );
  temp_cloud->header.frame_id = "scan";
  pcl_conversions::toPCL( ros::Time::now(), temp_cloud->header.stamp );
  //pub.publish( temp_cloud );

  if( temp_cloud->height == 1 )
  {
    // filter point within the range x = [-2.2, 0.0], y = [-2.0, 0.0] and x = [0.0, 2.2], y = [-2.0, 0.0]
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_left (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointXYZ> temp_vector_left;
    int point_counter_left = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_right (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointXYZ> temp_vector_right;
    int point_counter_right = 0;
    for( pcl::PointXYZ temp_point : temp_cloud->points )
    {
        if( temp_point.y > -2 && temp_point.y < -0.2 )
        {
          if ( temp_point.x > -2.2 && temp_point.x < 0.0 )
          {
            temp_vector_left.push_back ( temp_point );
            point_counter_left++;
          } else if ( temp_point.x > 0.0 && temp_point.x < 2.2 )
          {
            temp_vector_right.push_back ( temp_point );
            point_counter_right++;
          }
        }
    }

    temp_cloud_left->header.frame_id = "scan";
    temp_cloud_left->width = point_counter_left;
    temp_cloud_left->height = 1;
    for( pcl::PointXYZ temp_point : temp_vector_left )
    {
      temp_cloud_left->points.push_back (temp_point);
    }
    pcl_conversions::toPCL(ros::Time::now(), temp_cloud_left->header.stamp);
    //pub.publish (temp_cloud_left);

    temp_cloud_right->header.frame_id = "scan";
    temp_cloud_right->width = point_counter_right;
    temp_cloud_right->height = 1;
    for( pcl::PointXYZ temp_point : temp_vector_right )
    {
      temp_cloud_right->points.push_back (temp_point);
    }
    pcl_conversions::toPCL(ros::Time::now(), temp_cloud_right->header.stamp);
    //pub.publish (temp_cloud_right);

    // serch for K nearest point
    pcl::KdTreeFLANN< pcl::PointXYZ > kdtree;
    pcl::PointXYZ searchPoint;
    searchPoint.x = 0.0;
    searchPoint.y = 0.0;
    searchPoint.z = 0.0;
    std::vector<int> pointIdxNKNSearch ( K );
    std::vector<float> pointNKNSquaredDistance ( K );

    std::vector<pcl::PointXYZ> final_vector_left;
    if ( point_counter_left > 0 )
    {
      kdtree.setInputCloud ( temp_cloud_left );
      if ( kdtree.nearestKSearch ( searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance ) > 0 )
      {
        float x_sum = 0.0;
        float y_sum = 0.0;
        for ( size_t i = 0; i < pointIdxNKNSearch.size (); ++i )
        {
          pcl::PointXYZ temp_point;
          temp_point.x = temp_cloud_left->points[ pointIdxNKNSearch[i] ].x;
          x_sum += temp_point.x;
          temp_point.y = temp_cloud_left->points[ pointIdxNKNSearch[i] ].y;
          y_sum += temp_point.y;
          temp_point.z = temp_cloud_left->points[ pointIdxNKNSearch[i] ].z;
          final_vector_left.push_back ( temp_point );
          //std::cout << "[" << final_vector_left[i].x << ", " << final_vector_left[i].y << ", " << final_vector_left[i].z << "]; ";
        }
        point1_x = x_sum / K;
        point1_y = y_sum / K;
      }
      //std::cout << final_vector_left.size () << std::endl;
    }

    std::vector<pcl::PointXYZ> final_vector_right;
    if ( point_counter_right > 0 )
    {
      kdtree.setInputCloud ( temp_cloud_right );
      if ( kdtree.nearestKSearch ( searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
        float x_sum = 0.0;
        float y_sum = 0.0;
        for ( size_t i = 0; i < pointIdxNKNSearch.size (); ++i )
        {
          pcl::PointXYZ temp_point;
          temp_point.x = temp_cloud_right->points[ pointIdxNKNSearch[i] ].x;
          x_sum += temp_point.x;
          temp_point.y = temp_cloud_right->points[ pointIdxNKNSearch[i] ].y;
          y_sum += temp_point.y;
          temp_point.z = temp_cloud_right->points[ pointIdxNKNSearch[i] ].z;
          final_vector_right.push_back ( temp_point );
          //std::cout << "[" << final_vector_right[i].x << ", " << final_vector_right[i].y << ", " << final_vector_right[i].z << "]; ";
        }
        point2_x = x_sum / K;
        point2_y = y_sum / K;
      }
      //std::cout << final_vector_right.size () << std::endl;
    }

    // merge temp_cloud_left and temp_cloud_right and publish the point cloud message
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final ( new pcl::PointCloud<pcl::PointXYZ> );
    cloud_final->header.frame_id = "scan";
    cloud_final->width = final_vector_left.size () + final_vector_right.size ();
    //std::cout << cloud_final->width << std::endl;
    cloud_final->height = 1;
    for( pcl::PointXYZ temp_point : final_vector_left )
    {
      cloud_final->points.push_back ( temp_point );
    }
    for( pcl::PointXYZ temp_point : final_vector_right )
    {
      cloud_final->points.push_back ( temp_point );
    }
    pcl_conversions::toPCL(ros::Time::now(), cloud_final->header.stamp);
    pub.publish( cloud_final );

    // if the car is fixed, output old table position.
    if ( is_fix_table_position )
    {
      std::cout << ros::Time::now() << ": publish fixed [x, y, theta] = [" << old_x_t << ", " << old_y_t << ", " << old_theta2 << "]" << std::endl;
      publish_tf ( old_x_t, old_y_t, old_theta2 );
      return;
    }

    // calculate the central point of the table w.r.t the world frame
    get_the_central_point();
  }
}

bool start_fix_table_position ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  is_fix_table_position = true;
  std::cout << "is_fix_table_position = " << is_fix_table_position << std::endl;
  return true;
}

bool stop_fix_table_position ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  is_fix_table_position = false;
  std::cout << "is_fix_table_position = " << is_fix_table_position << std::endl;
  return true;
}

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "scan_reader" );
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner ( 4 );
  spinner.start();

  is_fix_table_position = false;
  ros::ServiceServer start_fix_table_position_;
  start_fix_table_position_ = nh_.advertiseService ( "start_fix_table_position", &start_fix_table_position );
  ros::ServiceServer stop_fix_table_position_;
  stop_fix_table_position_ = nh_.advertiseService ( "stop_fix_table_position", &stop_fix_table_position );

  ros::Subscriber sub = nh_.subscribe ( "/r2000_node/scan", 10, scanCallback );
  pub = nh_.advertise< pcl::PointCloud< pcl::PointXYZ > > ( "/scan_reader/points2", 10 );
  ros::waitForShutdown ();
  return 0;
}
