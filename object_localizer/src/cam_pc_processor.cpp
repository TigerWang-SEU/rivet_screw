#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>

#include <ros/ros.h>
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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;
std::string camera_frame = "camera_color_optical_frame";

int filter_mean_k = 60;
float filter_stddev = 1.0;

// filtering an input point cloud
void filterOutliner ( PointCloudT::Ptr cloud )
{
	static pcl::StatisticalOutlierRemoval < PointT > sor;
  sor.setInputCloud ( cloud );
  sor.setMeanK ( filter_mean_k );
  sor.setStddevMulThresh ( filter_stddev );
  sor.filter ( *cloud );
}

class CamPCProcessor
{
public:

  void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    if ( cloud->width * cloud->height == 0 )
		{
			return;
		}

    pcl::PCLPointCloud2 pcl_pc2;
    PointCloudT::Ptr scene_cloud_ ( new PointCloudT );
    pcl_conversions::toPCL( *cloud, pcl_pc2 );
    pcl::fromPCLPointCloud2 ( pcl_pc2, *scene_cloud_ );
		std::cout << "[" << cloud->header.stamp << "] Input point cloud has [" << scene_cloud_->width << "*" << scene_cloud_->height << " = " << scene_cloud_->width * scene_cloud_->height << "] data points" << std::endl;

		// downsampling and transforming the input point cloud
		// filterOutliner ( scene_cloud_ );
		tf::Vector3 point(0, 0, 0);
		tf::Vector3 point_n(0, 0, 0);
    float x_v, y_v, z_v, z_max, z_min;
		x_v = y_v = z_v = 0;
    z_max = 0;
    z_min = 100;
    int point_counter = 0;

		for ( PointT temp_point : scene_cloud_->points )
		{
      if (!pcl_isfinite (temp_point.x) ||
          !pcl_isfinite (temp_point.y) ||
          !pcl_isfinite (temp_point.z))
          continue;
			x_v += temp_point.x;
			y_v += temp_point.y;
			z_v += temp_point.z;
      point_counter++;
      if ( z_max < temp_point.z )
      {
        z_max = temp_point.z;
      }
      if ( z_min > temp_point.z )
      {
        z_min = temp_point.z;
      }
		}
    x_v = x_v / point_counter;
    y_v = y_v / point_counter;
    z_v = z_v / point_counter;
		std::cout << "There are total " << point_counter << " none empty points.";
    std::cout << "[x_v, y_v, z_v, z_max, z_min] = [" << x_v << ", " << y_v << ", " << z_v << ", " << z_max << ", " << z_min << "]" << std::endl;

		scene_cloud_->header.frame_id = camera_frame;
    // scene_cloud_->width = scene_cloud_->size();
		// scene_cloud_->height = 1;
		pcl_conversions::toPCL ( ros::Time::now(), scene_cloud_->header.stamp );
    cloud_pub_.publish ( scene_cloud_ );
  }

  CamPCProcessor ()
  {
    std::string cloud_topic_in_ = "/camera/depth_registered/points";
    cloud_sub_ = nh_.subscribe ( cloud_topic_in_, 15, &CamPCProcessor::cloud_cb, this );
    ROS_INFO_STREAM ( "Listening point cloud message on topic " << cloud_topic_in_ );

    std::string cloud_topic_out_ = "/cam_pc_processor/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_topic_out_, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << cloud_topic_out_ );
  }

  ~CamPCProcessor () { }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
};

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "cam_pc_processor" );
	ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  CamPCProcessor cpp;
  ros::waitForShutdown ();
  return 0;
}
