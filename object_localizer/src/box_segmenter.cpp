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
#include "object_localizer_msg/Segment_list.h"

// PCL head files
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

// boost geometry head files
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <vector>
#include <deque>

#include "head/reference_frame.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

class BoxSegmenter
{
public:

  void bbox_cb ( const object_localizer_msg::BBox_list::ConstPtr& bbox_list )
  {
    if ( is_publish_ && bbox_list->Segment_list.size() > 0 )
    {
      cropped_cloud->clear ();
      int box_counter = 0;
      for ( sensor_msgs::PointCloud2 segment_cloud_sm : bbox_list->Segment_list )
      {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL ( segment_cloud_sm, pcl_pc2 );
        PointCloudT::Ptr segment_cloud ( new PointCloudT );
        pcl::fromPCLPointCloud2 ( pcl_pc2, *segment_cloud );
        for ( PointT temp_point: segment_cloud->points )
        {
          float x = temp_point.x;
          float y = temp_point.y;
          float z = temp_point.z;
          PointT new_point;
					new_point.x = x;
    			new_point.y = y;
		      new_point.z = z;
					uint8_t r = 0, g = 0, b = 0;
					r = box_counter % 5 * 50;
					g = box_counter % 3 * 80;
					b = box_counter % 7 * 30;
          if ( box_counter == 0 )
          {
            r = 255;
            g = 255;
            b = 255;
          }
					uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
					new_point.rgb = *reinterpret_cast<float*>( &rgb );
          cropped_cloud->points.push_back( new_point );
        }
        box_counter++;
      }

      std::cout << "cropped_cloud has " << cropped_cloud->size()  << std::endl;
      cropped_cloud->header.frame_id = reference_frame;
      pcl_conversions::toPCL ( ros::Time::now(), cropped_cloud->header.stamp );
      cloud_pub_.publish ( cropped_cloud );
    }
  }

  bool start_box_segmenter ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    // clear old cropped point cloud
    cropped_cloud->clear ();
    cropped_cloud->header.frame_id = reference_frame;
    pcl_conversions::toPCL ( ros::Time::now(), cropped_cloud->header.stamp );
    cloud_pub_.publish ( cropped_cloud );
    is_publish_ = true;
    return true;
  }

  bool stop_box_segmenter ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    is_publish_ = false;
    return true;
  }

  BoxSegmenter () : cropped_cloud ( new PointCloudT )
  {
    is_publish_ = false;
    start_box_segmenter_ = nh_.advertiseService ( "start_box_segmenter", &BoxSegmenter::start_box_segmenter, this );
    stop_box_segmenter_ = nh_.advertiseService ( "stop_box_segmenter", &BoxSegmenter::stop_box_segmenter, this );

    std::string bbox_in_name = "/rough_localizer/bbox_list";
    bbox_sub_ = nh_.subscribe ( bbox_in_name, 6, &BoxSegmenter::bbox_cb, this );
    ROS_INFO_STREAM ( "Listening for bounding box list on topic: " << bbox_in_name );

    std::string cloud_out_name = "/box_segmenter/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_out_name, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud on topic: " << cloud_out_name );
  }

  ~BoxSegmenter () { }

private:
  ros::NodeHandle nh_;
  PointCloudT::Ptr cropped_cloud;
  bool is_publish_;
  ros::ServiceServer start_box_segmenter_, stop_box_segmenter_;
  ros::Subscriber bbox_sub_;
  ros::Publisher cloud_pub_;
};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "box_segmenter" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  BoxSegmenter BS;
  ros::waitForShutdown ();
  return 0;
}
