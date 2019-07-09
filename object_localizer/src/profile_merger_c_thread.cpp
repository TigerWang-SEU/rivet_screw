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

#include "ProfilesCallback/ProfilesCallback.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud< PointT > PointCloudT;
std::string reference_frame = "world";
std::string scanner_frame = "scanCONTROL_2900-50_scanner_laser_link";
double lag_compensation_ = 0.001;
static const int num_threads = 4;
bool is_stop = false;
int current_thread_index = 0;
std::mutex scene_cloud_mutex;
std::mutex thread_profile_queue_mutex [ num_threads ];
std::queue < PointCloudT::Ptr > thread_profile_queue [ num_threads ];

// filtering point cloud
void filterOutliner ( PointCloudT::Ptr cloud )
{
	static pcl::StatisticalOutlierRemoval < PointT > sor;
	sor.setStddevMulThresh ( 1.0 );
  sor.setInputCloud ( cloud );
  sor.setMeanK ( cloud->size()/2 );
  sor.filter ( *cloud );
}

void downSampling ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_sampled )
{
  static pcl::VoxelGrid<PointT> grid;
  grid.setInputCloud ( cloud );
  grid.setLeafSize ( 0.0002f, 0.0002f, 0.0002f );
  grid.filter ( *cloud_sampled );
	std::printf( "Downsampled cloud size is [%d, %d]\n", cloud_sampled->width, cloud_sampled->height );
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
					// wait for 0.01 second every time
					ros::Duration ( 0.01 ) .sleep ();
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

		tf::Vector3 point ( 0, 0, 0 );
		tf::Vector3 point_n ( 0, 0, 0 );
		for ( PointT temp_point : cloud->points )
		{
			point.setX ( temp_point.x );
			point.setY ( temp_point.y );
			point.setZ ( temp_point.z );
			if ( temp_point.y < 0.085 )
      {
        continue;
      }
			tf::Vector3 point_n = transform * point;
			temp_point.x = point_n.getX ();
			temp_point.y = point_n.getY ();
			temp_point.z = point_n.getZ ();
			cloud_out->points.push_back (temp_point);
		}
	}

  void merger_cb ( int thread_id )
  {
		std::cout << "\tThread [" << thread_id << "] is started" << std::endl;
		while ( !is_stop )
		{
			if ( !thread_profile_queue [ thread_id ].empty () )
			{
				PointCloudT::Ptr in_cloud;
				{
					std::lock_guard<std::mutex> guard ( thread_profile_queue_mutex [ thread_id ] );
					in_cloud = thread_profile_queue [ thread_id ].front ();
	    		thread_profile_queue [ thread_id ].pop ();
				}

				// downsampling and transforming the input point cloud
				PointCloudT::Ptr in_cloud_sampled	( new PointCloudT );
				downSampling ( in_cloud, in_cloud_sampled );
				PointCloudT::Ptr in_cloud_world	( new PointCloudT );
				transform_point_cloud ( in_cloud_sampled, in_cloud_world );
				std::cout << "\tThread [" << thread_id << "]: input point cloud after downSampling and transforming has [" << in_cloud_world->size() << "] data points" << std::endl;
				if ( in_cloud_world->size() == 0 )
				{
					continue;
				}

				// merging input point cloud into scene point cloud
				std::lock_guard<std::mutex> guard ( scene_cloud_mutex );
				*scene_cloud += *in_cloud_world;
				PointCloudT::Ptr scene_cloud_temp	( new PointCloudT );
				downSampling ( scene_cloud, scene_cloud_temp );
				scene_cloud = scene_cloud_temp;
				std::cout << "\tThread [" << thread_id << "]:scene point cloud has [" << scene_cloud->size() << "] data points" << std::endl;

				// show the scene point cloud
				scene_cloud->header.frame_id = reference_frame;
				pcl_conversions::toPCL ( ros::Time::now(), scene_cloud->header.stamp );
		    cloud_pub_.publish ( scene_cloud );
			}
			else
			{
				ros::Duration ( 0.01 * num_threads ).sleep ();
			}
		}
  }

	bool start_profile_merger ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
	{
	  is_publish_ = true;
		// clear the scene point cloud
		scene_cloud->clear ();
	  return true;
	}

	bool stop_profile_merger ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
	{
	  is_publish_ = false;
	  return true;
	}

  ProfileMerger () : scene_cloud ( new pcl::PointCloud< PointT > )
  {
		is_publish_ = false;
		start_profile_merger_ = nh_.advertiseService ( "start_profile_merger", &ProfileMerger::start_profile_merger, this );
		stop_profile_merger_ = nh_.advertiseService ( "stop_profile_merger", &ProfileMerger::stop_profile_merger, this );
		ros::Duration ( 1 ).sleep ();

    std::string cloud_out_name = "/profile_merger/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_out_name, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << cloud_out_name );
  }

  ~ProfileMerger () { }

private:
  ros::NodeHandle nh_;
	tf::TransformListener listener;
	bool is_publish_;
	ros::ServiceServer start_profile_merger_, stop_profile_merger_;
	pcl::PointCloud<PointT>::Ptr scene_cloud;
  ros::Publisher cloud_pub_;
};

void scanner_cb ()
{
  gint32 ret = 0;

  // 1. connect the laser scanner
  if ( ! connect_scanner ( serial_number ) )
  {
    return;
  }

  std::vector<double> value_x, value_z;
  profile_buffer.resize ( resolution * 64 );
  value_x.resize ( resolution );
  value_z.resize ( resolution );
  CInterfaceLLT::ResetEvent ( event );

  // 2. start transfer
  if ( ( ret = hLLT->TransferProfiles ( NORMAL_TRANSFER, true ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error in profile transfer! - [" << ret << "]" << std::endl;
    return;
  }

  // 3. loop until receive the end signal
  while ( !is_stop )
  {
    if ( CInterfaceLLT::WaitForSingleObject ( event, 2000 ) != WAIT_OBJECT_0 )
    {
      std::cout << "Timeout!" << std::endl;
      continue;
    }
    CInterfaceLLT::ResetEvent ( event );

		// 3.1. get x and z arrays
    if ( ( ret = CInterfaceLLT::ConvertProfile2Values ( &profile_buffer [ 0 ], profile_buffer.size(), resolution, PROFILE, llt_type, 0, NULL, NULL, NULL, &value_x[0], &value_z[0], NULL, NULL ) ) != ( CONVERT_X | CONVERT_Z ) )
    {
      std::cout << "Error while extracting profiles! - [" << ret << "]" << std::endl;
      continue;
    }

    // 3.2. get time information
    guint32 profile_counter = 0;
    double shutter_closed = 0, shutter_opened = 0;
    CInterfaceLLT::Timestamp2TimeAndCount ( &profile_buffer[ (resolution * 64) - 16 ], &shutter_closed, &shutter_opened, &profile_counter, NULL );

    // 3.3. create a new profile point cloud
    PointCloudT::Ptr profile_cloud	( new PointCloudT );
    profile_cloud->header.frame_id = scanner_frame;
    ros::Time profile_time = ros::Time::now() - ros::Duration ( ( shutter_opened + shutter_closed ) / 2.0 + lag_compensation_ );
    pcl_conversions::toPCL ( profile_time, profile_cloud->header.stamp );
    PointT temp_point;
    for ( int i = 0; i < value_x.size (); ++i )
    {
      if ( value_z [ i ] > 30 )
      {
        temp_point.x = - value_x [ i ] / 1000.0;
        temp_point.y = value_z [ i ] / 1000.0;
        temp_point.z = 0.0;
        profile_cloud->points.push_back ( temp_point );
      }
    }

		// 3.4. put the new profile point cloud into each thread_profile_queue
		{
			std::lock_guard<std::mutex> guard ( thread_profile_queue_mutex [ current_thread_index ] );
			thread_profile_queue [ current_thread_index ].push ( profile_cloud );
		}
		current_thread_index ++;
		current_thread_index = current_thread_index % num_threads;
  }

  // 4. stop transfer
  if ( ( ret = hLLT->TransferProfiles ( NORMAL_TRANSFER, false ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while stopping transmission! - [" << ret << "]" << std::endl;
  }

  // 5. disconnect the laser scanner
  disconnect_scanner ();
}

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "profile_merger_c_thread" );
	ros::AsyncSpinner spinner ( 3 );
  spinner.start ();
	ProfileMerger pm;

	// create a scanner thread and [num_threads] merger threads
	std::thread merger_thread [ num_threads ];
  for ( int idx = 0; idx < num_threads; ++idx )
  {
    merger_thread [ idx ] = std::thread ( &ProfileMerger::merger_cb, &pm, idx );
  }
	std::thread scanner_thread =  std::thread ( &scanner_cb );

	// wait for stop signal
  ros::waitForShutdown ();
	is_stop = true;

	//Join the threads with the main thread
	scanner_thread.join();
  for ( int idx = 0; idx < num_threads; ++idx )
  {
    merger_thread[idx].join();
  }
  return 0;
}
