#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include "head/reference_frame.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

void downSampling ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_sampled )
{
  static pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setInputCloud ( cloud );
  grid.setLeafSize ( 0.008f, 0.008f, 0.008f );
  grid.filter ( *cloud_sampled );
  std::printf ( "Downsampled cloud size is %lu\n", cloud_sampled->size() );
}

class PointCloudMerger
{
public:
  void transform_point_cloud ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_out)
  {
    tf::StampedTransform transform;
    ros::Time sample_time;
    pcl_conversions::fromPCL ( cloud->header.stamp, sample_time );
    std::string camera_frame = cloud->header.frame_id;

    bool is_lookuped = false;
    while ( is_lookuped == false )
    {
      try
      {
        listener.lookupTransform ( reference_frame, camera_frame, sample_time, transform );
        is_lookuped = true;
      }
      catch ( tf::TransformException ex )
      {
        if ( boost::starts_with ( ex.what(), "Lookup would require extrapolation into the future." ) )
        {
          ros::Duration ( 0.05 ) .sleep ();
        }
        else
        {
          ROS_ERROR ( "%s", ex.what() );
          return;
        }
      }
    }

    tf::Vector3 point_o ( 0, 0, 0 );
    tf::Vector3 point_n ( 0, 0, 0 );
    for ( PointT temp_point : cloud->points )
    {
      point_o.setX ( temp_point.x );
      point_o.setY ( temp_point.y );
      point_o.setZ ( temp_point.z );
      if ( !pcl_isfinite ( temp_point.x ) )
      {
        continue;
      }
      if ( temp_point.z < 0.40 || temp_point.z > 1.22 )
      {
        continue;
      }
      tf::Vector3 point_n = transform * point_o;
      temp_point.x = point_n.getX ();
      temp_point.y = point_n.getY ();
      temp_point.z = point_n.getZ ();
      if ( temp_point.z < 1.49 || temp_point.z > 1.82 )
      {
        continue;
      }
      cloud_out->points.push_back ( temp_point );
    }
  }

  void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    if ( !is_merge_ || ( cloud->width * cloud->height ) == 0 )
    {
      scene_cloud_total->header.frame_id = reference_frame;
      pcl_conversions::toPCL ( ros::Time::now(), scene_cloud_total->header.stamp );
      cloud_pub_.publish ( scene_cloud_total );
      return;
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL( *cloud, pcl_pc2 );
    PointCloudT::Ptr scene_cloud_ ( new PointCloudT );
    pcl::fromPCLPointCloud2 ( pcl_pc2, *scene_cloud_ );
    ros::Time sample_time = cloud->header.stamp;
    std::cout << "[" << sample_time << "] Input cloud has [" << scene_cloud_->size () << "] data points." << std::endl;

    // transforming the input cloud
    PointCloudT::Ptr scene_cloud_world	( new PointCloudT );
    transform_point_cloud ( scene_cloud_, scene_cloud_world );
    std::cout << "scene_cloud_world has [" << scene_cloud_world->size() << "] data points" << std::endl;

    // merging the input point cloud
    *scene_cloud_total += *scene_cloud_world;
    std::cout << "scene_cloud_total has [" << scene_cloud_total->size() << "] data points" << std::endl;
    PointCloudT::Ptr scene_cloud_total_temp ( new PointCloudT );
    downSampling ( scene_cloud_total, scene_cloud_total_temp );
    std::cout << "scene_cloud_total_temp has [" << scene_cloud_total_temp->size() << "] data points" << std::endl;
    scene_cloud_total = scene_cloud_total_temp;
    scene_cloud_total->header.frame_id = reference_frame;
    pcl_conversions::toPCL ( ros::Time::now(), scene_cloud_total->header.stamp );
    cloud_pub_.publish ( scene_cloud_total );
  }

  bool start_pcl_merge ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    scene_cloud_total->clear ();
    is_merge_ = true;
    return true;
  }

  bool end_pcl_merge ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    is_merge_ = false;
    return true;
  }

  PointCloudMerger () : scene_cloud_total ( new pcl::PointCloud< PointT > )
  {
    is_merge_ = false;
    start_pcl_merge_ = nh_.advertiseService ( "start_pcl_merge", &PointCloudMerger::start_pcl_merge, this );
    end_pcl_merge_ = nh_.advertiseService ( "stop_pcl_merge", &PointCloudMerger::end_pcl_merge, this );

    std::string cloud_topic_in_ = "/camera/depth_registered/points";
    cloud_sub_ = nh_.subscribe ( cloud_topic_in_, 200, &PointCloudMerger::cloud_cb, this );
    ROS_INFO_STREAM ( "Listening point cloud message on topic " << cloud_topic_in_ );

    std::string cloud_topic_out_ = "/point_cloud_merger/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_topic_out_, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << cloud_topic_out_ );
  }

  ~PointCloudMerger () {}

private:
  ros::NodeHandle nh_;
  tf::TransformListener listener;
  pcl::PointCloud<PointT>::Ptr scene_cloud_total;
  bool is_merge_;
  ros::ServiceServer start_pcl_merge_, end_pcl_merge_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
};

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "pcl_merger" );
	ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  PointCloudMerger pcm;
  ros::waitForShutdown ();
  return 0;
}
