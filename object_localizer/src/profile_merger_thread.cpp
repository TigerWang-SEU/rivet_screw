#include <stdio.h>
#include <iostream>
#include <string>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <ros/package.h>
#include <ctime>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

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

#include "head/reference_frame.h"
#include "head/ProfilesCallback.h"
#include "head/ThreadSafeQueue.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;
uint8_t r = 255, g = 255, b = 255;
uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );

double lag_compensation_ = 0.001;
static const int num_threads = 4;
bool is_stop = false;
bool is_publish_ = false;
int current_thread_index = 0;
ThreadSafeQueue < PointCloudT::Ptr > profile_thread_queue [ num_threads ];
ThreadSafeQueue < PointCloudT::Ptr > scene_pc_queue;

void downSampling ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_sampled )
{
  static pcl::VoxelGrid < PointT > grid;
  grid.setInputCloud ( cloud );
  grid.setLeafSize ( 0.0002f, 0.0002f, 0.0002f );
  grid.filter ( *cloud_sampled );
  std::printf ( "Downsampled cloud size is %lu\n", cloud_sampled->size () );
}

class ProfileMerger
{
public:
  void transform_point_cloud ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_out )
  {
    tf::StampedTransform transform;
    ros::Time sample_time = pcl_conversions::fromPCL ( cloud->header.stamp );
    bool is_lookuped = false;
    int delay_counter = 0;
    while ( is_lookuped == false )
    {
      try
      {
        listener.lookupTransform ( reference_frame, scanner_frame, sample_time, transform );
        is_lookuped = true;
      }
      catch ( tf::TransformException ex )
      {
        if ( boost::starts_with ( ex.what(), "Lookup would require extrapolation into the future." ) )
        {
          ros::Duration ( 0.01 ).sleep ();
          if ( delay_counter == 4 )
          {
            return;
          }
          delay_counter++;
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
      if ( temp_point.y < 0.085 )
      {
        continue;
      }
      point_o.setX ( temp_point.x );
      point_o.setY ( temp_point.y );
      point_o.setZ ( temp_point.z );
      tf::Vector3 point_n = transform * point_o;
      temp_point.x = point_n.getX ();
      temp_point.y = point_n.getY ();
      temp_point.z = point_n.getZ ();
      temp_point.rgb = *reinterpret_cast<float*> ( &rgb );
      cloud_out->points.push_back ( temp_point );
    }
  }

  void transform_cb ( int thread_id )
  {
    std::cout << "Transform thread [" << thread_id << "] is started." << std::endl;
    while ( !is_stop )
    {
      if ( !is_publish_ )
      {
        ros::Duration ( 0.01 * num_threads ).sleep ();
        continue;
      }

      // get a profile cloud
      PointCloudT::Ptr in_cloud ( new PointCloudT );
      profile_thread_queue [ thread_id ].pop ( in_cloud );
      if ( in_cloud->size() == 0 )
      {
        continue;
      }

      // transforming the profile cloud to the frame [world]
      PointCloudT::Ptr in_cloud_transformed ( new PointCloudT );
      transform_point_cloud ( in_cloud, in_cloud_transformed );
      if ( in_cloud_transformed->size() == 0 )
      {
        continue;
      }

      scene_pc_queue.push ( in_cloud_transformed );
    }

    std::cout << "Transform thread [" << thread_id << "] is stopped." << std::endl;
  }

  void merger_cb ()
  {
    std::cout << "Merger thread is started." << std::endl;
    while ( !is_stop )
    {
      if ( !is_publish_ )
      {
        ros::Duration ( 0.01 * num_threads ).sleep ();
        continue;
      }

      int buffer_length = 18;
      if ( scene_pc_queue.length () <= buffer_length )
      {
        ros::Duration ( 0.01 * num_threads ).sleep ();
        continue;
      }

      int buffer_counter = 0;
      PointCloudT::Ptr in_cloud ( new PointCloudT );
      while ( buffer_counter < buffer_length )
      {
        PointCloudT::Ptr _buffer_in_cloud ( new PointCloudT );
        scene_pc_queue.pop ( _buffer_in_cloud );
        *in_cloud += *_buffer_in_cloud;
        buffer_counter ++;
      }

      if ( in_cloud->size() == 0 )
      {
        return;
      }

      // merge input point cloud with scene point cloud
      *scene_cloud += *in_cloud;
      PointCloudT::Ptr scene_cloud_sampled	( new PointCloudT );
      downSampling ( scene_cloud, scene_cloud_sampled );
      scene_cloud = scene_cloud_sampled;
      std::cout << "Scene point cloud has [" << scene_cloud->size() << "] data points" << std::endl;

      // show the scene point cloud
      scene_cloud->header.frame_id = reference_frame;
      pcl_conversions::toPCL ( ros::Time::now(), scene_cloud->header.stamp );
      cloud_pub_.publish ( scene_cloud );
    }
    std::cout << "Merger thread is stopped." << std::endl;
  }

  bool start_profile_merger ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    is_publish_ = false;
    ros::Duration ( 0.01 * ( num_threads + 1 ) ).sleep ();
    // clear the profile_thread_queue, scene_pc_queue, and scene point cloud.
    for ( int idx = 0; idx < num_threads; ++idx )
    {
      profile_thread_queue [ idx ].clear ();
    }
    scene_pc_queue.clear ();
    scene_cloud->clear ();
    power_on ();
    is_publish_ = true;
    return true;
  }

  bool stop_profile_merger ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ros::Duration ( 0.5 ).sleep ();
    is_publish_ = false;
    power_off ();
    return true;
  }

  ProfileMerger () : scene_cloud ( new pcl::PointCloud< PointT > )
  {
    is_publish_ = false;
    start_profile_merger_ = nh_.advertiseService ( "start_profile_merger", &ProfileMerger::start_profile_merger, this );
    stop_profile_merger_ = nh_.advertiseService ( "stop_profile_merger", &ProfileMerger::stop_profile_merger, this );

    std::string cloud_out_name = "/profile_merger/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_out_name, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << cloud_out_name );
  }

  ~ProfileMerger () { }

private:
  ros::NodeHandle nh_;
  tf::TransformListener listener;
  ros::ServiceServer start_profile_merger_, stop_profile_merger_;
  pcl::PointCloud<PointT>::Ptr scene_cloud;
  ros::Publisher cloud_pub_;
};

double average ( double a, double b )
{
  return ( a + b ) / 100000.0 / 2.0;
}

void scanner_cb ()
{
  gint32 ret = 0;

  // 1. connect the laser scanner
  if ( ! connect_scanner ( serial_number ) )
  {
    std::cout << "Error in connecting the laser scanner!" << std::endl;
    return;
  }

  std::vector < double > value_x, value_z;
  profile_buffer.resize ( resolution * 64 );
  value_x.resize ( resolution );
  value_z.resize ( resolution );
  CInterfaceLLT::ResetEvent ( event );

  // 2. power off the laser scanner
  power_off ();

  // 3. loop until receive the end signal
  while ( !is_stop )
  {
    if ( CInterfaceLLT::WaitForSingleObject ( event, 2000 ) != WAIT_OBJECT_0 )
    {
      continue;
    }
    CInterfaceLLT::ResetEvent ( event );

    if ( !is_publish_ )
    {
      ros::Duration ( 0.01 * num_threads ).sleep ();
      continue;
    }

    // 3.1. get x and z arrays
    if ( ( ret = CInterfaceLLT::ConvertProfile2Values ( &profile_buffer [ 0 ], profile_buffer.size(), resolution, PROFILE, llt_type, 0, NULL, NULL, NULL, &value_x [ 0 ], &value_z [ 0 ], NULL, NULL ) ) != ( CONVERT_X | CONVERT_Z ) )
    {
      std::cout << "Error while extracting profiles! - [" << ret << "]" << std::endl;
      continue;
    }

    // 3.2. get profile_counter and time information
    guint32 profile_counter = 0;
    double shutter_closed = 0, shutter_opened = 0;
    CInterfaceLLT::Timestamp2TimeAndCount ( &profile_buffer[ ( resolution * 64 ) - 16 ], &shutter_closed, &shutter_opened, &profile_counter, NULL );

    // 3.3. create a new profile point cloud
    PointCloudT::Ptr profile_cloud ( new PointCloudT );
    profile_cloud->header.frame_id = scanner_frame;
    ros::Time profile_time = ros::Time::now() - ros::Duration ( average ( shutter_opened, shutter_closed ) + lag_compensation_ );
    pcl_conversions::toPCL ( profile_time, profile_cloud->header.stamp );
    PointT temp_point;
    for ( int i = 0; i < value_x.size (); ++i )
    {
      if ( value_z [ i ] > 30 )
      {
        temp_point.x = - value_x [ i ] / 1000.0;
        temp_point.y = value_z [ i ] / 1000.0;
        temp_point.z = 0.0;
        temp_point.rgb = *reinterpret_cast<float*> ( &rgb );
        profile_cloud->points.push_back ( temp_point );
      }
    }

    if ( profile_cloud->size () == 0 )
    {
      continue;
    }

    // 3.4. put the new profile point cloud into each profile_thread_queue
    profile_thread_queue [ current_thread_index ].push ( profile_cloud );
    current_thread_index = ( current_thread_index + 1 ) % num_threads;
  }

  // 4. disconnect the laser scanner
  disconnect_scanner ();
  std::cout << "scanner thread is stopped!" << std::endl;
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "profile_merger_thread" );
  ros::AsyncSpinner spinner ( 3 );
  spinner.start ();
  ProfileMerger pm;

  // create a merger thread, [num_threads] transform threads, and a scanner thread
  std::thread merger_thread =  std::thread ( &ProfileMerger::merger_cb, &pm );
  std::thread transform_thread [ num_threads ];
  for ( int idx = 0; idx < num_threads; ++idx )
  {
    transform_thread [ idx ] = std::thread ( &ProfileMerger::transform_cb, &pm, idx );
  }
  std::thread scanner_thread =  std::thread ( &scanner_cb );

  // wait for stop signal
  ros::waitForShutdown ();
  is_stop = true;

  // Join the threads with the main thread
  scanner_thread.join ();
  for ( int idx = 0; idx < num_threads; ++idx )
  {
    transform_thread [ idx ].join ();
  }
  merger_thread.join ();

  return 0;
}
