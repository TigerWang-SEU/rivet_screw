#include <stdio.h>
#include <iostream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

// converse header file between ros points2 and pcl point cloud
#include <pcl_conversions/pcl_conversions.h>
#include "object_localizer_msg/BBox_int.h"
#include "object_localizer_msg/BBox_float.h"
#include "object_localizer_msg/BBox_list.h"

// PCL head files
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

// boost geometry head files
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <vector>
#include <deque>

#include "head/reference_frame.h"

namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> point_t_b;
typedef bg::model::box<point_t_b> box_t_b;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

void downSampling ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_sampled )
{
  static pcl::VoxelGrid<PointT> grid;
  grid.setInputCloud ( cloud );
  grid.setLeafSize ( 0.005f, 0.005f, 0.005f );
  grid.filter ( *cloud_sampled );
  std::printf( "Downsampled cloud size is %lu\n", cloud_sampled->size() );
}

class RoughLocalizer
{

public:

  void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    // if input cloud is empty, return
    if ( ( cloud->width * cloud->height ) == 0 )
      return;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL ( *cloud, pcl_pc2 );
    pcl::fromPCLPointCloud2 ( pcl_pc2, *saved_cloud );
  }

  void publish_bbox_list ()
  {
    static int list_size = 0;
    object_localizer_msg::BBox_list::Ptr bbox_list_msg ( new object_localizer_msg::BBox_list() );
    bbox_list_msg->header.frame_id = reference_frame;
    bbox_list_msg->header.stamp = ros::Time::now();
    for ( box_t_b box_tmp : box_t_b_list )
    {
      double min_x = box_tmp.min_corner().get<0>();
      double min_y = box_tmp.min_corner().get<1>();
      double max_x = box_tmp.max_corner().get<0>();
      double max_y = box_tmp.max_corner().get<1>();
      object_localizer_msg::BBox_float bbox;
      bbox.x1 = min_x;
      bbox.x2 = max_x;
      bbox.y1 = min_y;
      bbox.y2 = max_y;
      bbox_list_msg->BBox_list_float.push_back( bbox );
    }
    for ( pcl::PointCloud<PointT>::Ptr segment_cloud : segment_list )
    {
      pcl::PCLPointCloud2 segment_cloud_pc2;
      pcl::toPCLPointCloud2 ( *segment_cloud, segment_cloud_pc2 );
      sensor_msgs::PointCloud2 segment_cloud_sm;
      pcl_conversions::fromPCL ( segment_cloud_pc2, segment_cloud_sm );
      bbox_list_msg->Segment_list.push_back ( segment_cloud_sm );
    }
    if ( box_t_b_list.size() != list_size )
    {
      list_size = box_t_b_list.size();
      bbox_pub_.publish( bbox_list_msg );
    }
  }

  void get_theta ( box_t_b& box_in, float& theta_min, float& theta_max )
  {
    double min_y = box_in.min_corner().get<0>();
    double min_z = box_in.min_corner().get<1>();
    theta_min = atan2 ( min_z, min_y ) * 180.0 / M_PI;
    double max_y = box_in.max_corner().get<0>();
    double max_z = box_in.max_corner().get<1>();
    theta_max = atan2 ( max_z, max_y ) * 180.0 / M_PI;
    if ( theta_min > theta_max )
    {
      double theta_tmp = theta_max;
      theta_max = theta_min;
      theta_min = theta_tmp;
    }
    std::cout << "[theta_min, theta_max] = [" <<  theta_min << ", " << theta_max << "]" << std::endl;
  }

  void handle_box_tb_theta ( float theta_min, float theta_max, pcl::PointCloud<PointT>::Ptr box_cloud )
  {
    box_t_b box_n { { theta_min, theta_min }, { theta_max, theta_max } } ;
    if ( box_t_b_list.size() == 0 )
    {
      box_t_b_list.push_back ( box_n );
      segment_list.push_back ( box_cloud );
      std::cout << "case 0, size " << box_t_b_list.size() << std::endl;
    }
    else
    {
      // iterate for the first time, find the bounding box within another bounding box
      for ( box_t_b box_o : box_t_b_list )
      {
        float theta_min_2 = box_o.min_corner().get<0>();
        float theta_max_2 = box_o.max_corner().get<0>();
        if ( theta_min_2 <= theta_min && theta_max_2 >= theta_max )
        {
          std::cout << "case 1, size " << box_t_b_list.size () << std::endl;
          return;
        }
        else if ( theta_min <= theta_min_2 && theta_max >= theta_max_2 )
        {
          std::cout << "case 2, size " << box_t_b_list.size () << std::endl;
          return;
        }
      }
      // iterate for second time, find the bounding boxes overlap with each another
      for ( box_t_b box_o : box_t_b_list )
      {
        float theta_min_2 = box_o.min_corner().get<0>();
        float theta_max_2 = box_o.max_corner().get<0>();
        if ( theta_min_2 >= theta_min && theta_min_2 <= theta_max )
        {
          float ratio = ( theta_max - theta_min_2 ) / ( theta_max - theta_min );
          float ratio_2 = ( theta_max - theta_min_2 ) / ( theta_max_2 - theta_min_2 );
          if ( ratio > 0.5 || ratio_2 > 0.5 )
          {
            std::cout << "case 3, [ratio, ratio_2] = [" << ratio << ", " << ratio_2 << "]" << std::endl;
            return;
          }
        }
        if ( theta_max_2 >= theta_min && theta_max_2 <= theta_max )
        {
          float ratio = ( theta_max_2 - theta_min ) / ( theta_max - theta_min );
          float ratio_2 = ( theta_max_2 - theta_min ) / ( theta_max_2 - theta_min_2 );
          if ( ratio > 0.5 || ratio_2 > 0.5 )
          {
            std::cout << "case 4, [ratio, ratio_2] = [" << ratio << ", " << ratio_2 << "]" << std::endl;
            return;
          }
        }
      }
      // don't have overlap with other box_t_b
      box_t_b_list.push_back ( box_n );
      segment_list.push_back ( box_cloud );
      std::cout << "case 5, size " << box_t_b_list.size() << std::endl;
    }
  }

  void handle_box_tb ( box_t_b& box_n, pcl::PointCloud<PointT>::Ptr box_cloud )
  {
    if ( box_t_b_list.size() == 0 )
    {
      box_t_b_list.push_back( box_n );
      segment_list.push_back ( box_cloud );
      std::cout << "case 0, size " << box_t_b_list.size() << std::endl;
    }
    else
    {
      // iterate for first time, find the bounding box within another bounding box and choose the smaller one
      int idx_counter = 0;
      for ( box_t_b box_o : box_t_b_list )
      {
        if ( bg::within ( box_o, box_n ) )
        {
          std::cout << "case 1, size " << box_t_b_list.size () << std::endl;
          return;
        } else if ( bg::within ( box_n, box_o ) )
        {
          // box_t_b_list.erase ( box_t_b_list.begin() + idx_counter );
          // box_t_b_list.push_back ( box_n );
          // segment_list.erase ( segment_list.begin() + idx_counter );
          // segment_list.push_back ( box_cloud );
          // std::cout << "case 2, size " << box_t_b_list.size () << std::endl;
          return;
        }
        idx_counter++;
      }
      // iterate for second time, find the bounding boxes overlap with each another and save the overlaped part
      idx_counter = 0;
      for ( box_t_b box_o : box_t_b_list )
      {
        if ( bg::overlaps ( box_n, box_o ) )
        {
          box_t_b box_intersection;
          bg::intersection ( box_n, box_o, box_intersection );
          float intersection_a = bg::area ( box_intersection );
          float box_o_a = bg::area ( box_o );
          float box_n_a = bg::area ( box_n );
          std::cout << "case 3, intersection_a = " << intersection_a << " ratio 1 = " << intersection_a / box_n_a << " ratio 2 = " << intersection_a / box_o_a << std::endl;
          if ( (intersection_a / box_n_a) > 0.6 || (intersection_a / box_o_a) > 0.6 )
          {
            // box_t_b_list.erase ( box_t_b_list.begin() + idx_counter );
            // box_t_b_list.push_back ( box_intersection );
            // double min_x = box_intersection.min_corner().get<0>();
            // double min_y = box_intersection.min_corner().get<1>();
            // double max_x = box_intersection.max_corner().get<0>();
            // double max_y = box_intersection.max_corner().get<1>();
            // pcl::PointCloud<PointT>::Ptr box_cloud_intersection ( new PointCloudT );
            // int point_counter = 0;
            // for ( size_t i = 0; i < box_cloud->points.size (); ++i )
            // {
            //   PointT temp_point = box_cloud->points[i];
            //   if ( temp_point.x > min_x && temp_point.x < max_x &&  temp_point.y > min_y && temp_point.y < max_y )
            //   {
            //     box_cloud_intersection->points.push_back ( temp_point );
            //     point_counter ++;
            //   }
            // }
            // box_cloud_intersection->width = point_counter;
            // box_cloud_intersection->height = 1;
            // box_cloud_intersection->header.frame_id = reference_frame;
            // segment_list.erase ( segment_list.begin() + idx_counter );
            // segment_list.push_back ( box_cloud_intersection );
            return;
          }
        }
        idx_counter++;
      }
      // don't have overlap with other box_t_b
      box_t_b_list.push_back ( box_n );
      segment_list.push_back ( box_cloud );
      std::cout << "case 4, size " << box_t_b_list.size() << std::endl;
    }
  }

  void bbox_cb ( const object_localizer_msg::BBox_list::ConstPtr& bbox_list )
  {
    if ( !is_publish_ )
      return;

    if ( bbox_list->BBox_list_int.size() > 0 )
    {
      try
      {
        ros::Time bbox_time = bbox_list->header.stamp;
        ros::Time cloud_time;
        pcl_conversions::fromPCL ( saved_cloud->header.stamp, cloud_time );
        std::cout << "Time diff is " << cloud_time - bbox_time << std::endl;

        std::string camera_frame = saved_cloud->header.frame_id;
        tf::StampedTransform transform;
        tf_listener.lookupTransform ( reference_frame, camera_frame, cloud_time, transform );

        tf::Vector3 point ( 0, 0, 0 );
        for ( object_localizer_msg::BBox_int bbox : bbox_list->BBox_list_int )
        {
          pcl::PointCloud<PointT>::Ptr box_cloud ( new PointCloudT );
          double box_min_x = DBL_MAX, box_max_x = -DBL_MAX;
          double box_min_y = DBL_MAX, box_max_y = -DBL_MAX;
          double box_min_z = DBL_MAX, box_max_z = -DBL_MAX;
          std::cout << "Bounding box has [x1, x2, y1, y2]: [" << bbox.x1 << "," << bbox.x2 << "," << bbox.y1 << "," << bbox.y2  << "]" << std::endl;
          int middle_idx = (int) ( ( bbox.x1 + bbox.x2 ) / 2.0 );
          for ( int idx_x = bbox.x1; idx_x <= bbox.x2 && idx_x < saved_cloud->height; idx_x++ )
          {
            for ( int idx_y = bbox.y1; idx_y <= bbox.y2 && idx_y < saved_cloud->width; idx_y++ )
            {
              PointT temp_point = saved_cloud->at ( idx_y, idx_x );
              if ( !pcl_isfinite ( temp_point.x ) ||
                   !pcl_isfinite ( temp_point.y ) ||
                   !pcl_isfinite ( temp_point.z ) )
                  continue;
              point.setX ( temp_point.x );
              point.setY ( temp_point.y );
              point.setZ ( temp_point.z );
              tf::Vector3 point_n = transform * point;

              temp_point.x = point_n.getX ();
              if ( box_min_x > temp_point.x )
              {
                box_min_x = temp_point.x;
              }
              if ( box_max_x < temp_point.x )
              {
                box_max_x = temp_point.x;
              }
              temp_point.y = point_n.getY ();
              if ( box_min_y > temp_point.y && idx_x == middle_idx )
              {
                box_min_y = temp_point.y;
              }
              if ( box_max_y < temp_point.y && idx_x == middle_idx )
              {
                box_max_y = temp_point.y;
              }
              temp_point.z = point_n.getZ ();
              if ( box_min_z > temp_point.z && idx_x == middle_idx )
              {
                box_min_z = temp_point.z;
              }
              if ( box_max_z < temp_point.z && idx_x == middle_idx )
              {
                box_max_z = temp_point.z;
              }
              box_cloud->points.push_back ( temp_point );
            }
          }

          if ( box_cloud->size () > 0 )
          {
            std::cout << "box point size: " << box_cloud->size () << std::endl;
            std::cout << "\trange of x is [" << box_min_x << ", " << box_max_x << "];"<< std::endl;
            std::cout << "\trange of y is [" << box_min_y << ", " << box_max_y << "]" << std::endl;
            std::cout << "\trange of z is [" << box_min_z << ", " << box_max_z << "]" << std::endl;
            box_t_b theta_box { { box_min_y, box_min_z }, { box_max_y, box_max_z } } ;
            float theta_min, theta_max;
            get_theta ( theta_box, theta_min, theta_max );
            if ( theta_max - theta_min <= 20.0 )
            {
              std::cout << "\033[31;47m" << "\trange of theta is [" << theta_min << ", " << theta_max << "]" << "\033[0m" << std::endl;
              handle_box_tb_theta ( theta_min, theta_max, box_cloud );
            }
            // if ( std::abs ( box_max_y - box_min_y ) >= std::abs ( box_max_z - box_min_z ) )
            // {
            //   box_t_b box_n { { box_min_x, box_min_y }, { box_max_x, box_max_y } } ;
            //   handle_box_tb ( box_n,  box_cloud );
            // }
            // else
            // {
            //   box_t_b box_n { { box_min_x, box_min_z }, { box_max_x, box_max_z } } ;
            //   handle_box_tb ( box_n,  box_cloud );
            // }
          }
        }
        // publish the box_t_b_list topic;
        publish_bbox_list ();
      }
      catch ( tf::TransformException ex )
      {
        // ROS_ERROR("%s", ex.what());
      }
    }
  }

  bool start_rough_localizer ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    // clear up old bounding boxes
    box_t_b_list.clear ();
    segment_list.clear ();
    is_publish_ = true;
    return true;
  }

  bool stop_rough_localizer ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    is_publish_ = false;
    return true;
  }

  RoughLocalizer () : saved_cloud ( new pcl::PointCloud < PointT > )
  {
    is_publish_ = false;
    start_rough_localizer_ = nh_.advertiseService ( "start_rough_localizer", &RoughLocalizer::start_rough_localizer, this );
    stop_rough_localizer_ = nh_.advertiseService ( "stop_rough_localizer", &RoughLocalizer::stop_rough_localizer, this );

    std::string cloud_in_name = "/camera/depth_registered/points";
    cloud_sub_ = nh_.subscribe ( cloud_in_name, 30, &RoughLocalizer::cloud_cb, this );
    ROS_INFO_STREAM ( "Listening for point cloud on topic: " << cloud_in_name );

    std::string bbox_in_name = "/object_localizer/bbox_list";
    bbox_sub_ = nh_.subscribe ( bbox_in_name, 30, &RoughLocalizer::bbox_cb, this );
    ROS_INFO_STREAM ( "Listening for bounding box list on topic: " << bbox_in_name );

    std::string bbox_out_name = "/rough_localizer/bbox_list";
    bbox_pub_ = nh_.advertise < object_localizer_msg::BBox_list > ( bbox_out_name, 30 );
    ROS_INFO_STREAM ( "Publishing bounding box list on topic: " << bbox_out_name );
  }

  ~RoughLocalizer () { }

private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener;
  pcl::PointCloud < PointT > ::Ptr saved_cloud;
  bool is_publish_;
  ros::ServiceServer start_rough_localizer_, stop_rough_localizer_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber bbox_sub_;
  ros::Publisher bbox_pub_;
  std::vector < box_t_b > box_t_b_list;
  std::vector < pcl::PointCloud < PointT > ::Ptr > segment_list;
};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "rough_localizer" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  RoughLocalizer RL;
  ros::waitForShutdown ();
  return 0;
}
