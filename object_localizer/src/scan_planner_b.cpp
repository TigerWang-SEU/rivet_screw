#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <ros/package.h>
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
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

std::string reference_frame = "world";

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

float x_adjust = -0.01; // adjustment for the x poistion
float scan_distance = 0.075; // set the distance to the scanning part
float scan_half_length = 0.10; // scanning path length
float s_scale = 0.2; // scale of the start scanning scan_half_length
float e_scale = 1.1; // scale of the end scanning scan_half_length

void read_scan_planner_cfg_file ( )
{
  std::string scan_planner_cfg_file_name = "scan_planner.cfg";
  std::string scan_planner_cfg_file = ros::package::getPath ( "object_localizer" ) + "/config/" + scan_planner_cfg_file_name;
  std::cout << "***The path of the scan_planner file is: [" << scan_planner_cfg_file << "]" << std::endl;

  std::ifstream input ( scan_planner_cfg_file );
  std::string line;
  if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
		// adjustment for the x poistion
    iss >> x_adjust;
  }
	if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
		// set the distance to the scanning part
    iss >> scan_distance;
  }
	if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
		// scanning path length
    iss >> scan_half_length;
  }
	if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
		// scale of the start scanning scan_half_length
    iss >> s_scale;
  }
	if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
		// scale of the end scanning scan_half_length
    iss >> e_scale;
  }

  input.close();
}

// filtering an input point cloud
int filter_mean_k = 40;
float filter_stddev = 1.0;

void filterOutliner ( PointCloudT::Ptr cloud )
{
	static pcl::StatisticalOutlierRemoval < PointT > sor;
  sor.setInputCloud ( cloud );
  sor.setMeanK ( filter_mean_k );
  sor.setStddevMulThresh ( filter_stddev );
  sor.filter ( *cloud );
}

void show_segment_cloud ( PointCloudT::ConstPtr cloud )
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer ( new pcl::visualization::PCLVisualizer ( "Segment Viewer" ) );
  viewer->setBackgroundColor ( 0, 0, 0 );
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb( cloud );
  viewer->addPointCloud< PointT > ( cloud, rgb, "segment cloud" );
  viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segment cloud" );
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  while ( !viewer->wasStopped () )
  {
    viewer->spinOnce ( 100 );
    boost::this_thread::sleep ( boost::posix_time::microseconds (100000) );
  }
}

void getMinMax3D ( const pcl::PointCloud<PointT> &cloud, PointT &min_pt, PointT &max_pt )
{
  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if ( cloud.is_dense )
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) ||
          !pcl_isfinite (cloud.points[i].y) ||
          !pcl_isfinite (cloud.points[i].z))
          continue;
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  min_pt.x = min_p[0]; min_pt.y = min_p[1]; min_pt.z = min_p[2];
  max_pt.x = max_p[0]; max_pt.y = max_p[1]; max_pt.z = max_p[2];
}

void calculate_bounding_box ( PointCloudT::ConstPtr cloudSegmented )
{
  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid( *cloudSegmented, pcaCentroid );
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized( *cloudSegmented, pcaCentroid, covariance );

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver( covariance, Eigen::ComputeEigenvectors );
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  // This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
  // the signs are different and the box doesn't get correctly oriented in some cases.
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

  std::cout << "eigen vector 0: [" << eigenVectorsPCA(0, 0) << ", " << eigenVectorsPCA(1, 0) << ", " << eigenVectorsPCA(2, 0) << "]" << std::endl;
  std::cout << "eigen vector 1: [" << eigenVectorsPCA(0, 1) << ", " << eigenVectorsPCA(1, 1) << ", " << eigenVectorsPCA(2, 1) << "]" << std::endl;
  std::cout << "eigen vector 2: [" << eigenVectorsPCA(0, 2) << ", " << eigenVectorsPCA(1, 2) << ", " << eigenVectorsPCA(2, 2) << "]" << std::endl;
  // std::cout << "eigen vector 1: " << eigenVectorsPCA.col(1) << std::endl;
  // std::cout << "eigen vector 2: " << eigenVectorsPCA.col(2) << std::endl;

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform( Eigen::Matrix4f::Identity() );
  projectionTransform.block< 3,3 >( 0, 0 ) = eigenVectorsPCA.transpose();
  projectionTransform.block< 3,1 >( 0, 3 ) = -1.0f * ( projectionTransform.block< 3, 3 >( 0, 0 ) * pcaCentroid.head< 3 >() );
  pcl::PointCloud< PointT >::Ptr cloudPointsProjected ( new pcl::PointCloud< PointT > );
  pcl::transformPointCloud( *cloudSegmented, *cloudPointsProjected, projectionTransform );
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZRGB minPoint, maxPoint;
  getMinMax3D( *cloudPointsProjected, minPoint, maxPoint );
  const Eigen::Vector3f meanDiagonal = 0.5f * ( maxPoint.getVector3fMap() + minPoint.getVector3fMap() );

  // calculate quaternion from eigenvectors
  const Eigen::Quaternionf bboxQuaternion( eigenVectorsPCA );
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head< 3 >();

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer ( new pcl::visualization::PCLVisualizer ( "Segment Viewer" ) );
  viewer->setBackgroundColor ( 0, 0, 0 );
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb( cloudSegmented );
  viewer->addPointCloud< PointT > ( cloudSegmented, rgb, "segment cloud" );
  viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segment cloud" );
  viewer->addCoordinateSystem ( 1.0 );
  viewer->addCube ( bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox" );
  viewer->initCameraParameters ();
  while ( !viewer->wasStopped () )
  {
    viewer->spinOnce ( 100 );
    boost::this_thread::sleep ( boost::posix_time::microseconds (100000) );
  }
}

template < typename T > int sgn ( T val )
{
    return ( T ( 0 ) < val ) - ( val < T ( 0 ) );
}

float calculate_theta ( PointCloudT::ConstPtr cloudSegmented, Eigen::Vector3f& central_point )
{
  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid ( *cloudSegmented, pcaCentroid );
  central_point = pcaCentroid.head< 3 >();

  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized ( *cloudSegmented, pcaCentroid, covariance );

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver ( covariance, Eigen::ComputeEigenvectors );
	Eigen::Vector3f eigenvaluesPCA = eigen_solver.eigenvalues ();
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors ();

	int max_idx_1_r, max_idx_2_r, max_idx_3_r, max_idx_1_c, max_idx_2_c, max_idx_3_c;
	eigenVectorsPCA.block ( 0, 0, 3, 1 ).cwiseAbs().maxCoeff( &max_idx_1_r, &max_idx_1_c );
	eigenVectorsPCA.block ( 0, 1, 3, 1 ).cwiseAbs().maxCoeff( &max_idx_2_r, &max_idx_2_c );
	eigenVectorsPCA.block ( 0, 2, 3, 1 ).cwiseAbs().maxCoeff( &max_idx_3_r, &max_idx_3_c );
  std::cout << "****** eigen value 1 = [" << eigenvaluesPCA (0) << "] ******\n\t eigen vector 0: [" << eigenVectorsPCA (0, 0) << ", " << eigenVectorsPCA (1, 0) << ", " << eigenVectorsPCA (2, 0) << "] \n\t max_idx = " << max_idx_1_r << std::endl;
  std::cout << "****** eigen value 2 = [" << eigenvaluesPCA (1) << "] ******\n\t eigen vector 1: [" << eigenVectorsPCA ( 0, 1 ) << ", " << eigenVectorsPCA ( 1, 1 ) << ", " << eigenVectorsPCA ( 2, 1 ) << "] \n\t max_idx = " << max_idx_2_r << std::endl;
  std::cout << "****** eigen value 3 = [" << eigenvaluesPCA (2) << "] ******\n\t eigen vector 2: [" << eigenVectorsPCA ( 0, 2 ) << ", " << eigenVectorsPCA ( 1, 2 ) << ", " << eigenVectorsPCA ( 2, 2 ) << "] \n\t max_idx = " << max_idx_3_r << std::endl;

	float y_tmp, z_tmp;
	if ( max_idx_3_r != 0 )
	{
		y_tmp = eigenVectorsPCA ( 1, 2 );
	  z_tmp = eigenVectorsPCA ( 2, 2 );
	}
	else
	{
		y_tmp = eigenVectorsPCA ( 1, 1 );
	  z_tmp = eigenVectorsPCA ( 2, 1 );
	}
  if ( y_tmp < 0.0 )
  {
    y_tmp = - y_tmp;
    z_tmp = - z_tmp;
  }
  std::cout << "\t[y_tmp, z_tmp] = [" << y_tmp << ", " << z_tmp << "]\n";
	float theta = atan2 ( z_tmp, y_tmp ) * 180.0 / M_PI + 90.0;
	return theta;
}

float get_central_point ( PointCloudT::Ptr segment_cloud, Eigen::Vector3f& central_point )
{
  filterOutliner ( segment_cloud );
  PointT minPt, maxPt, searchPoint, midPt;
  getMinMax3D ( *segment_cloud, minPt, maxPt );
  // get the (x, y, z) of maximum and minimum points
  std::cout << "Min [x, y, z]: = [" << minPt.x << ", " << minPt.y << ", " << minPt.z << "]" << std::endl;
  std::cout << "Max [x, y, z]: = [" << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "]" << std::endl;
  // calculate the middle point
  searchPoint.x = ( maxPt.x + minPt.x ) / 2.0;
  searchPoint.y = ( maxPt.y + minPt.y ) / 2.0;
  searchPoint.z = ( maxPt.z + minPt.z ) / 2.0;
  // search the points within the radius of 4.5 centimeter
  std::vector < int > pointIdxRadiusSearch;
  std::vector < float > pointRadiusSquaredDistance;
  float radius = 0.045;
  std::cout << "Neighbors within radius search at (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;
  pcl::KdTreeFLANN < PointT > kdtree;
  kdtree.setInputCloud ( segment_cloud );
  midPt.x = midPt.y = midPt.z = 0;
  minPt.z = maxPt.z = searchPoint.z;
  if ( kdtree.radiusSearch ( searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
  {
    for ( size_t i = 0; i < pointIdxRadiusSearch.size (); ++i )
    {
      midPt.x += segment_cloud->points[ pointIdxRadiusSearch[i] ].x;
      midPt.y += segment_cloud->points[ pointIdxRadiusSearch[i] ].y;
      midPt.z += segment_cloud->points[ pointIdxRadiusSearch[i] ].z;
      if ( segment_cloud->points[ pointIdxRadiusSearch[i] ].z >= maxPt.z )
      {
        maxPt = segment_cloud->points[ pointIdxRadiusSearch[i] ];
      }
      if ( segment_cloud->points[ pointIdxRadiusSearch[i] ].z <= minPt.z )
      {
        minPt = segment_cloud->points[ pointIdxRadiusSearch[i] ];
      }
    }
    midPt.x = midPt.x / pointIdxRadiusSearch.size ();
    midPt.y = midPt.y / pointIdxRadiusSearch.size ();
    midPt.z = midPt.z / pointIdxRadiusSearch.size ();
  }
  central_point ( 0 ) = midPt.x;
  central_point ( 1 ) = midPt.y;
  central_point ( 2 ) = midPt.z;

  PointT minPt_2, maxPt_2;
  minPt_2.x = minPt_2.y = minPt_2.z = 0;
  maxPt_2.x = maxPt_2.y = maxPt_2.z = 0;
  radius = 0.025;
  if ( kdtree.radiusSearch ( maxPt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
  {
    for ( size_t i = 0; i < pointIdxRadiusSearch.size (); ++i )
    {
      maxPt_2.x += segment_cloud->points[ pointIdxRadiusSearch[i] ].x;
      maxPt_2.y += segment_cloud->points[ pointIdxRadiusSearch[i] ].y;
      maxPt_2.z += segment_cloud->points[ pointIdxRadiusSearch[i] ].z;
    }
    maxPt_2.x = maxPt_2.x / pointIdxRadiusSearch.size ();
    maxPt_2.y = maxPt_2.y / pointIdxRadiusSearch.size ();
    maxPt_2.z = maxPt_2.z / pointIdxRadiusSearch.size ();
  }

  radius = 0.01;
  if ( kdtree.radiusSearch ( minPt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance ) > 0 )
  {
    for ( size_t i = 0; i < pointIdxRadiusSearch.size (); ++i )
    {
      minPt_2.x += segment_cloud->points[ pointIdxRadiusSearch[i] ].x;
      minPt_2.y += segment_cloud->points[ pointIdxRadiusSearch[i] ].y;
      minPt_2.z += segment_cloud->points[ pointIdxRadiusSearch[i] ].z;
    }
    minPt_2.x = minPt_2.x / pointIdxRadiusSearch.size ();
    minPt_2.y = minPt_2.y / pointIdxRadiusSearch.size ();
    minPt_2.z = minPt_2.z / pointIdxRadiusSearch.size ();
  }

  std::cout << "new Mid [x, y, z]: = [" << midPt.x << ", " << midPt.y << ", " << midPt.z << "]" << std::endl;
  std::cout << "new Min [x, y, z]: = [" << minPt_2.x << ", " << minPt_2.y << ", " << minPt_2.z << "]" << std::endl;
  std::cout << "new Max [x, y, z]: = [" << maxPt_2.x << ", " << maxPt_2.y << ", " << maxPt_2.z << "]" << std::endl;
  float theta = atan2 ( std::abs ( maxPt_2.z - minPt_2.z ), std::abs ( maxPt_2.y - minPt_2.y ) ) * 180.0 / M_PI + 90.0;
  return theta;
}

class ScanPlanner
{
public:

  void segment_list_cb ( const object_localizer_msg::BBox_list::ConstPtr& segment_list_in )
  {
    if ( segment_list_in->BBox_list_float.size() > 0 )
    {
      segment_list.reset ( new object_localizer_msg::Segment_list () );
      int bbox_idx = 0;
			std::cout << "BBox_list_float.size = " << segment_list_in->BBox_list_float.size() << " Segment_list.size = " << segment_list_in->Segment_list.size() << std::endl;
      for ( object_localizer_msg::BBox_float bbox : segment_list_in->BBox_list_float )
      {
        segment_list->BBox_list_float.push_back ( bbox );
        segment_list->Segment_list.push_back ( segment_list_in->Segment_list [ bbox_idx ] );
        bbox_idx ++;
      }
    }
  }

  bool start_scan_planner ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
		read_scan_planner_cfg_file ( );
		std::cout << "[x_adjust, scan_distance, scan_half_length, s_scale, e_scale] = ["<< x_adjust << ", " << scan_distance << ", " << scan_half_length << ", " << s_scale << ", " << e_scale << "]"<< std::endl;

    if ( segment_list->BBox_list_float.size() > 0 )
    {
      ofstream do_scan_fs;
      std::string cfgFileName = ros::package::getPath ( "motion_control" ) + "/config/scan_plan.cfg";
      do_scan_fs.open ( cfgFileName );

      // iterate through all segments
			int bbox_idx = 0;
      for ( object_localizer_msg::BBox_float bbox : segment_list->BBox_list_float )
      {
        std::cout << "Bounding box [" << bbox_idx << "] has [x1, x2, y1, y2]: [" << bbox.x1 << "," << bbox.x2 << "," << bbox.y1 << "," << bbox.y2  << "]" << std::endl;

        // step 1, convert a segment message to a point cloud.
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL ( segment_list->Segment_list [ bbox_idx ], pcl_pc2 );
        PointCloudT::Ptr segment_cloud ( new PointCloudT );
        pcl::fromPCLPointCloud2 ( pcl_pc2, *segment_cloud );
        bbox_idx ++;
        // show_segment_cloud( segment_cloud );
        // calculate_bounding_box( segment_cloud );

        ///////////////////////////////////////////////////////////////////////
        // step 2, filter out point clouds which is not longer than 0.08
        float min_x, max_x;
        PointT minPt, maxPt;
        getMinMax3D ( *segment_cloud, minPt, maxPt );
        std::cout << "Min [x, y, z]: = [" << minPt.x << ", " << minPt.y << ", " << minPt.z << "]" << std::endl;
        min_x = minPt.x;
        std::cout << "Max [x, y, z]: = [" << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << "]" << std::endl;
        max_x = maxPt.x;

        float distance_ = std::sqrt ( std::pow ( ( maxPt.y - minPt.y ), 2 ) + std::pow ( ( maxPt.z - minPt.z ), 2 ) );
        std::cout << "$$$$$$ distance_ = [" << distance_ << "]" << std::endl;
        if ( distance_  < 0.08 )
        {
          continue;
        }

        // step 2.1, filter the segment point cloud
        PointCloudT::Ptr filtered_segment_cloud	( new PointCloudT );
        for ( PointT temp_point: segment_cloud->points )
        {
          float x = temp_point.x;
          float y = temp_point.y;
          float z = temp_point.z;
          if ( ( x - min_x ) / ( max_x - min_x ) < 0.6 )
          {
            continue;
          }
          PointT new_point;
          new_point.x = x - 0.02;
          new_point.y = y;
          new_point.z = z;
          filtered_segment_cloud->points.push_back( new_point );
        }
        segment_cloud = filtered_segment_cloud;
        ///////////////////////////////////////////////////////////////////////

        Eigen::Vector3f central_point;
        // step 3, find the central point of the segment point cloud
        // float theta = get_central_point ( segment_cloud, central_point );
        float theta = calculate_theta ( segment_cloud, central_point );
        if ( theta == 0 )
        {
          theta = 90;
        }
        std::cout << std::endl << "[***] Rotation around x is [" << theta << "] degrees" << std::endl;
        float x_0 = central_point ( 0 );
        float y_0 = central_point ( 1 );
        float z_0 = central_point ( 2 );

        // step 4, calculate the scanning start point and the scanning end point
        float theta_tmp = ( theta - 90.0 ) * M_PI / 180.0;
        float x_tmp = x_0 + x_adjust;
        float y_tmp = y_0 + scan_distance * std::sin ( theta_tmp );
        float z_tmp = z_0 - scan_distance * std::cos ( theta_tmp );
        // std::cout << "[***] Scan central point is [x, y, z] = [" << x_tmp << ", " << y_tmp << ", " << z_tmp << "]" << std::endl;
        float x_s = x_tmp;
        float y_s = y_tmp - scan_half_length * std::cos ( theta_tmp ) * s_scale;
        float z_s = z_tmp - scan_half_length * std::sin ( theta_tmp ) * s_scale;
        std::cout << "[***] Scan start point is [x, y, z] = [" << x_s << ", " << y_s << ", " << z_s << "]" << std::endl;
        float x_e = x_tmp;
        float y_e = y_tmp + scan_half_length * std::cos ( theta_tmp ) * e_scale;
        float z_e = z_tmp + scan_half_length * std::sin ( theta_tmp ) * e_scale;
        std::cout << "[***] Scan end point is [x, y, z] = [" << x_e << ", " << y_e << ", " << z_e << "]" << std::endl << std::endl;

        // step 5, write scanning plannings into the scanning plan file.
        do_scan_fs << theta << " " << x_s << " " << y_s << " " << z_s << " " << x_e << " " << y_e << " " << z_e << std::endl;

      }
    	do_scan_fs.close();
    }
    return true;
  }

  ScanPlanner ()
  {
    start_scan_planner_ = nh_.advertiseService ( "start_scan_planner", &ScanPlanner::start_scan_planner, this );
    ros::Duration ( 0.5 ) .sleep ();

    std::string segment_list_in_name = "/rough_localizer/bbox_list";
    segment_list_sub_ = nh_.subscribe ( segment_list_in_name, 10, &ScanPlanner::segment_list_cb, this );
    ROS_INFO_STREAM ( "Listening for segment list on topic: " << segment_list_in_name );
  }

  ~ScanPlanner () { }

private:
  ros::NodeHandle nh_;
  object_localizer_msg::Segment_list::Ptr segment_list;
  ros::ServiceServer start_scan_planner_;
  ros::Subscriber segment_list_sub_;
};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "scan_planner_b" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ScanPlanner SP;
  ros::waitForShutdown ();
  return 0;
}
