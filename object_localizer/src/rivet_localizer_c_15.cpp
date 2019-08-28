#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <list>
#include <set>
#include <map>
#include <ctime>
#include <cmath>
#include <limits>
#include <boost/algorithm/string/predicate.hpp>
#include <sstream>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

#define PI 3.14159265
std::string reference_frame = "world";
std::string SceneFileName;
int filter_mean_k = 40;
float filter_stddev = 1.0;
float scale_factor = 1.0;
float rivet_height = 0.008; // unit meter
float rivet_height_2 = 0.0084; // unit meter
float rivet_radius = 0.007; // unit meter
bool show_viz = false;
float out_distance = 0.012;
float in_distance = 0.0065;
float tool_angle = 0.0;
uint8_t r = 0, g = 255, b = 0;
uint32_t rgb = 0;
int degree_num_threshold = 30;

/** Fits a circle to the points in the given 3D pointcloud
    and saves them in the CENTER and RADIUS.
    It assumes that the points lie on the SAME XY plane,
    i.e. their Z-component is the same.  */
void get_circle ( pcl::PointCloud< PointT >::Ptr cloud_in_, double min_circle_radius, double max_circle_radius, float ransac_thresh, Eigen::Vector3f &center, float &radius, std::vector< int > &inliers )
{
  boost::shared_ptr < pcl::SampleConsensusModelCircle2D < PointT > > model ( new pcl::SampleConsensusModelCircle2D < PointT > ( cloud_in_ ) );
	model->setRadiusLimits ( min_circle_radius, max_circle_radius );

  pcl::RandomSampleConsensus < PointT > sac ( model, ransac_thresh );
  sac.setMaxIterations ( 200 );

  // compute the model and get parameters
  bool result = sac.computeModel ();
  sac.getInliers ( inliers );
  Eigen::VectorXf xyr;
  sac.getModelCoefficients ( xyr );
  center ( 0 ) = xyr ( 0 );
  center ( 1 ) = xyr ( 1 );
  center ( 2 ) = cloud_in_->points [ 0 ].z;
  radius = xyr ( 2 );
}

bool check_circle ( pcl::PointCloud < PointT >::Ptr cloud_in_ )
{
  std::set < double > degree_set;
  for ( PointT temp_point: cloud_in_->points )
	{
		float x = temp_point.x;
		float y = temp_point.y;
		float z = temp_point.z;
    double degree = atan2 ( y, z ) * 180 / PI;
    degree_set.insert ( floor ( degree / 5.0 ) );
	}
  std::cout << "\t### degree_set.size () = " << degree_set.size () << std::endl;
  if ( degree_set.size () < degree_num_threshold )
  {
    return false;
  }
  return true;
}

void get_plane ( pcl::PointCloud< PointT >::Ptr cloud_in_, float ransac_thresh, Eigen::Vector3f &surface_normal, float &d, std::vector< int > &inliers )
{
  pcl::SampleConsensusModelPlane < PointT >::Ptr model_p ( new pcl::SampleConsensusModelPlane < PointT > ( cloud_in_ ) );

  pcl::RandomSampleConsensus < PointT > ransac ( model_p );
  ransac.setDistanceThreshold ( ransac_thresh );
  ransac.setMaxIterations ( 200 );

  // compute the model and get parameters
  bool result = ransac.computeModel();
  ransac.getInliers(inliers);
  Eigen::VectorXf xyzd;
  ransac.getModelCoefficients ( xyzd );
  surface_normal ( 0 ) = xyzd ( 0 );
  surface_normal ( 1 ) = xyzd ( 1 );
  surface_normal ( 2 ) = xyzd ( 2 );
  d = xyzd ( 3 );
}

// find the points within the search_radius around the search point
void find_near_point ( PointCloudT::Ptr cloud_in_, PointCloudT::Ptr cloud_out_, PointT &searchPoint, float search_radius = 0.007 )
{
	pcl::KdTreeFLANN < PointT > kdtree;
  kdtree.setInputCloud ( cloud_in_ );
	std::vector < int > pointIdx;
	std::vector < float > pointRadius;

	uint8_t r = 0, g = 255, b = 0;
	uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );

	if ( kdtree.radiusSearch ( searchPoint, search_radius, pointIdx, pointRadius ) > 0 )
	{
		for ( size_t j = 0; j < pointIdx.size (); ++j )
		{
			PointT new_point;
			new_point.x = cloud_in_->points [ pointIdx [ j ] ].x;
			new_point.y = cloud_in_->points [ pointIdx [ j ] ].y;
			new_point.z = cloud_in_->points [ pointIdx [ j ] ].z;
			new_point.rgb = *reinterpret_cast<float*> ( &rgb );
			cloud_in_->points [ pointIdx [ j ] ].rgb = *reinterpret_cast < float* > ( &rgb );
			cloud_out_->points.push_back ( new_point );
		}
		cloud_out_->header.frame_id = reference_frame;
	}
}

// define the union-find data structure
class UF
{
	int size, cnt, *id, *sz;
public:
	// Create an empty union find data structure with N isolated sets.
	UF ( int N )
	{
		size = N;
		cnt = N;
		id = new int[N];
		sz = new int[N];
		for ( int i = 0; i < N; i++ )
		{
			id[i] = i;
	    sz[i] = 1;
		}
	}

	~UF ()
	{
		delete [] id;
		delete [] sz;
	}

	// Return the id of component corresponding to object p.
	int find ( int p )
	{
		int root = p;
		while ( root != id [ root ] )
			root = id[root];
		while ( p != root )
		{
			int newp = id [ p ];
      id [ p ] = root;
      p = newp;
    }
		return root;
	}

	// Replace sets containing x and y with their union.
  void merge ( int x, int y )
	{
		int i = find ( x );
		int j = find ( y );
		if ( i == j ) return;

		// make smaller root point to larger one
		if ( sz [ i ] < sz [ j ] )
		{
			id [ i ] = j;
			sz [ j ] += sz [ i ];
		} else
		{
			id [ j ] = i;
			sz [ i ] += sz [ j ];
		}
    cnt--;
  }

	// Are objects x and y in the same set?
	bool connected ( int x, int y )
	{
		return find ( x ) == find ( y );
  }

	// Return the number of disjoint sets.
	int count()
	{
		return cnt;
  }

	// return the list of connected components
	std::map < int, std::list < int > > get_components ()
	{
		std::map < int, std::list < int > > component_map;
		for ( int i = 0; i < size; i++ )
		{
			int component_id = find ( i );
			if ( component_map.find ( component_id ) != component_map.end() )
			{
				component_map[ component_id ].push_back( i );
			}
			else
			{
				std::list < int > component = {i};
				component_map.insert ( std::pair< int, std::list < int > > ( component_id, component ) );
			}
		}
		return component_map;
	}

};

// downsampling an input point cloud
void downSampling ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_sampled )
{
  static pcl::VoxelGrid<PointT> grid;
  grid.setInputCloud ( cloud );
  grid.setLeafSize ( 0.0001f, 0.0001f, 0.0001f );
  grid.filter ( *cloud_sampled );
	std::printf( "Downsampled cloud size is %d, %d\n", cloud_sampled->width, cloud_sampled->height );
}

// filtering an input point cloud
void filterOutliner ( PointCloudT::Ptr cloud )
{
	static pcl::StatisticalOutlierRemoval < PointT > sor;
  sor.setInputCloud ( cloud );
  sor.setMeanK ( filter_mean_k );
  sor.setStddevMulThresh ( filter_stddev );
  sor.filter ( *cloud );
}

void calculate_transform ( PointCloudT::Ptr cloud_in,  Eigen::Matrix4f& projectionTransform )
{
  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid ( *cloud_in, pcaCentroid );
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized ( *cloud_in, pcaCentroid, covariance );

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver ( covariance, Eigen::ComputeEigenvectors );
  Eigen::Vector3f eigenvaluesPCA = eigen_solver.eigenvalues ();
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors ();
  eigenVectorsPCA.col ( 2 ) = eigenVectorsPCA.col ( 0 ).cross ( eigenVectorsPCA.col ( 1 ) );


  std::cout << "eigen value = [" << eigenvaluesPCA.transpose() << "]" << std::endl;
  std::cout << "eigen vector 0: [" << eigenVectorsPCA(0, 0) << ", " << eigenVectorsPCA(1, 0) << ", " << eigenVectorsPCA(2, 0) << "]" << std::endl;
  std::cout << "eigen vector 1: [" << eigenVectorsPCA(0, 1) << ", " << eigenVectorsPCA(1, 1) << ", " << eigenVectorsPCA(2, 1) << "]" << std::endl;
  std::cout << "eigen vector 2: [" << eigenVectorsPCA(0, 2) << ", " << eigenVectorsPCA(1, 2) << ", " << eigenVectorsPCA(2, 2) << "]" << std::endl;

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  projectionTransform.block< 3, 3 >( 0, 0 ) = eigenVectorsPCA.transpose();
  projectionTransform.block< 3, 1 >( 0, 3 ) = -1.0f * ( projectionTransform.block< 3, 3 >( 0, 0 ) * pcaCentroid.head< 3 >() );
}

void scale_and_color_point_cloud ( PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out, float scale_factor = 1.0 )
{
	cloud_out->points.clear();
	uint8_t r = 255, g = 255, b = 255;
	uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );

  for ( PointT temp_point: cloud_in->points )
  {
    float x = temp_point.x * scale_factor;
    float y = temp_point.y * scale_factor;
    float z = temp_point.z * scale_factor;
    PointT new_point;
    new_point.x = x;
    new_point.y = y;
    new_point.z = z;
    new_point.rgb = *reinterpret_cast<float*> ( &rgb );
    cloud_out->points.push_back( new_point );
  }
  cloud_out->header.frame_id = reference_frame;
}

void getMinMax3D ( const pcl::PointCloud<PointT> &cloud, PointT &min_pt, PointT &max_pt )
{
  Eigen::Array4f min_p, max_p;
  min_p.setConstant ( FLT_MAX );
  max_p.setConstant ( -FLT_MAX );

  // If the data is dense, we don't need to check for NaN
  if ( cloud.is_dense )
  {
    for ( size_t i = 0; i < cloud.points.size (); ++i )
    {
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for ( size_t i = 0; i < cloud.points.size (); ++i )
    {
      // Check if the point is invalid
      if ( !pcl_isfinite (cloud.points[i].x) ||
           !pcl_isfinite (cloud.points[i].y) ||
           !pcl_isfinite (cloud.points[i].z) )
          continue;
      pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap ();
      min_p = min_p.min (pt);
      max_p = max_p.max (pt);
    }
  }
  min_pt.x = min_p[0]; min_pt.y = min_p[1]; min_pt.z = min_p[2];
  max_pt.x = max_p[0]; max_pt.y = max_p[1]; max_pt.z = max_p[2];
}

void getInverseMatrix ( Eigen::Matrix4f& original_transform, Eigen::Matrix4f& inverse_transform )
{
	inverse_transform.block< 3, 3 >( 0, 0 ) = original_transform.block< 3, 3 >( 0, 0 ).transpose();
  inverse_transform.block< 3, 1 >( 0, 3 ) = -1.0f * ( inverse_transform.block< 3, 3 >( 0, 0 ) * original_transform.block< 3, 1 >( 0, 3 ) );
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

void get_rpy_from_matrix ( Eigen::Matrix3f rotation_matrix, double& roll, double& pitch, double& yaw )
{
  tf::Matrix3x3 object_m ( rotation_matrix (0, 0), rotation_matrix (0, 1), rotation_matrix (0, 2), rotation_matrix (1, 0), rotation_matrix (1, 1), rotation_matrix (1, 2), rotation_matrix (2, 0), rotation_matrix (2, 1), rotation_matrix (2, 2) );
  object_m.getRPY ( roll, pitch, yaw );
}

//#############################################################################
// function for calculating rotation around the x axis
float u_10_base_y = -0.212;

float calculate_theta ( PointCloudT::ConstPtr cloudSegmented )
{
  // step 1, get min_x and max_x
  float min_x, max_x;
  PointT minPt, maxPt;
  getMinMax3D ( *cloudSegmented, minPt, maxPt );
  min_x = minPt.x;
  std::cout << "\tmin_x = " << min_x << std::endl;
  max_x = maxPt.x;
  std::cout << "\tmax_x = " << max_x << std::endl;

  // step 2, filter the segment point cloud
  PointCloudT::Ptr filtered_segment_cloud	( new PointCloudT );
  float y_avg = 0;
  for ( PointT temp_point: cloudSegmented->points )
  {
    float x = temp_point.x;
    float y = temp_point.y;
    float z = temp_point.z;
    if ( ( x - min_x ) / ( max_x - min_x ) < 0.2 )
    {
      continue;
    }
    PointT new_point;
    new_point.x = x;
    new_point.y = y;
    y_avg += y;
    new_point.z = z;
    filtered_segment_cloud->points.push_back( new_point );
  }
  y_avg = y_avg / filtered_segment_cloud->points.size();

  // step 3, compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid ( *filtered_segment_cloud, pcaCentroid );

  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized ( *filtered_segment_cloud, pcaCentroid, covariance );

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver ( covariance, Eigen::ComputeEigenvectors );
  Eigen::Vector3f eigenvaluesPCA = eigen_solver.eigenvalues ();
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors ();

  int max_idx_1_r, max_idx_2_r, max_idx_3_r, max_idx_1_c, max_idx_2_c, max_idx_3_c;
  eigenVectorsPCA.block ( 0, 0, 3, 1 ).cwiseAbs().maxCoeff( &max_idx_1_r, &max_idx_1_c );
  eigenVectorsPCA.block ( 0, 1, 3, 1 ).cwiseAbs().maxCoeff( &max_idx_2_r, &max_idx_2_c );
  eigenVectorsPCA.block ( 0, 2, 3, 1 ).cwiseAbs().maxCoeff( &max_idx_3_r, &max_idx_3_c );
  std::cout << "****** eigen value 1 = [" << eigenvaluesPCA (0) << "] ******\n\t eigen vector 1: [" << eigenVectorsPCA ( 0, 0 ) << ", " << eigenVectorsPCA ( 1, 0 ) << ", " << eigenVectorsPCA ( 2, 0 ) << "] \n\t max_idx = " << max_idx_1_r << std::endl;
  std::cout << "****** eigen value 2 = [" << eigenvaluesPCA (1) << "] ******\n\t eigen vector 2: [" << eigenVectorsPCA ( 0, 1 ) << ", " << eigenVectorsPCA ( 1, 1 ) << ", " << eigenVectorsPCA ( 2, 1 ) << "] \n\t max_idx = " << max_idx_2_r << std::endl;
  std::cout << "****** eigen value 3 = [" << eigenvaluesPCA (2) << "] ******\n\t eigen vector 3: [" << eigenVectorsPCA ( 0, 2 ) << ", " << eigenVectorsPCA ( 1, 2 ) << ", " << eigenVectorsPCA ( 2, 2 ) << "] \n\t max_idx = " << max_idx_3_r << std::endl;

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
  //############# need to be test with more experiments #######################
  //############# I think the method used right now is wrong ##################
  float theta = atan2 ( z_tmp, y_tmp ) * 180.0 / M_PI + 90.0;
  if ( y_avg - u_10_base_y  < -0.2 )
  {
    if ( theta < 90.0 )
    {
      theta = theta + 180.0;
    }
  }
  if ( y_avg - u_10_base_y  > 0.3 )
  {
    if ( theta > 90.0 )
    {
      theta = theta - 180.0;
    }
  }
  return theta;

}

// find orientation and central point for each rivet
void get_rivet_center_orientation ( PointCloudT::Ptr cloud_in, PointCloudT::Ptr search_cloud_, PointT &searchPoint, Eigen::Matrix4f transform_1, PointCloudT::Ptr& rivet_support_plane_cloud, PointCloudT::Ptr& rivet_cloud, double& roll, double& pitch, double& yaw, Eigen::Vector4f& rivet_point_new_final, Eigen::Vector4f& rivet_point_in_final )
{
  // step 1, search for points near the original rivet center ( searchPoint )
  PointCloudT::Ptr search_result_cloud ( new PointCloudT );
  PointT search_point_1;
  search_point_1.x = 0.0;
  search_point_1.y = searchPoint.y;
  search_point_1.z = searchPoint.z - 0.0035;
  find_near_point ( search_cloud_, search_result_cloud, search_point_1 );
  PointT search_point_2;
  search_point_2.x = 0.0;
  search_point_2.y = searchPoint.y;
  search_point_2.z = searchPoint.z + 0.0035;
  find_near_point ( search_cloud_, search_result_cloud, search_point_2 );

  //step 2, fit a plane with the point cloud search_result_cloud
  Eigen::Vector3f surface_normal;
  float d_coef;
  float ransac_thresh = 0.0002;
  std::vector < int > inliers;
  get_plane ( search_result_cloud, ransac_thresh, surface_normal, d_coef, inliers );
  if ( inliers.size () == 0 )
  {
    PCL_ERROR ( "Could not find a planar model for the given point cloud." );
    rivet_point_new_final ( 0 ) = NAN;
    return;
  }
  // std::cout << "surface_normal : [" << surface_normal.transpose() << "], d_coef = " << d_coef << std::endl;

  // step 3 show the search_result_cloud fit the plane using color yellow
  r = 255, g = 255, b = 0;
  rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
  for ( int const& idx: inliers )
  {
    PointT new_point;
    new_point.x = search_result_cloud->points [ idx ].x;
    new_point.y = search_result_cloud->points [ idx ].y;
    new_point.z = search_result_cloud->points [ idx ].z;
    new_point.rgb = *reinterpret_cast < float* > ( &rgb );
    rivet_support_plane_cloud->points.push_back ( new_point );
  }

  // step 4, calculate the new orientation from the rivet_support_plane_cloud
  Eigen::Matrix4f transform_1_inverse ( Eigen::Matrix4f::Identity () );
  getInverseMatrix ( transform_1, transform_1_inverse );
  pcl::transformPointCloud ( *rivet_support_plane_cloud, *rivet_support_plane_cloud, transform_1_inverse );
  Eigen::Matrix4f transform_2 ( Eigen::Matrix4f::Identity() );
  calculate_transform ( rivet_support_plane_cloud, transform_2 );

  // step 5, check the transform_total to make sure the x axis is along the direction of the rivet
  Eigen::Matrix4f r_x_180, r_y_180, r_z_180;
  r_x_180 <<  1,  0,  0, 0,
              0, -1,  0, 0,
              0,  0, -1, 0,
              0,  0,  0, 1;
  r_y_180 << -1,  0,  0, 0,
              0,  1,  0, 0,
              0,  0, -1, 0,
              0,  0,  0, 1;
  r_z_180 << -1,  0,  0, 0,
              0, -1,  0, 0,
              0,  0,  1, 0,
              0,  0,  0, 1;
  Eigen::Matrix4f transform_total = transform_2;
  std::cout << "\t transform_total = \n" << transform_total << std::endl;
  if ( transform_total ( 0, 1 ) < 0 && transform_total ( 1, 0 ) < 0 )
  {
    std::cout << "\t r_z_180 " << std::endl;
    transform_total = r_z_180 * transform_total;
  }
  else if ( transform_total ( 0, 1 ) < 0 && transform_total ( 1, 0 ) > 0 )
  {
    std::cout << "\t r_y_180 " << std::endl;
    transform_total = r_y_180 * transform_total;
  }
  else if ( transform_total ( 0, 1 ) > 0 && transform_total ( 1, 0 ) < 0 )
  {
    std::cout << "\t r_x_180 " << std::endl;
    transform_total = r_x_180 * transform_total;
  }
  std::cout << "\t transform_total = \n" << transform_total << std::endl;

	// step 6, calculate roll, pitch, yaw from the transform_total
	Eigen::Matrix4f transform_total_inverse ( Eigen::Matrix4f::Identity() );
	getInverseMatrix ( transform_total, transform_total_inverse );
	get_rpy_from_matrix ( transform_total_inverse.block< 3, 3 >( 0, 0 ), roll, pitch, yaw );
	std::cout << "\tRoll, Pitch, Yaw = [" << roll << ", " << pitch << ", " << yaw << "]" << std::endl;

	// step 7, transform the rivet_support_plane and cloud_in
  PointCloudT::Ptr rivet_support_plane_cloud_transformed ( new PointCloudT );
	pcl::transformPointCloud ( *rivet_support_plane_cloud, *rivet_support_plane_cloud_transformed, transform_total );
	PointCloudT::Ptr cloud_in_transformed	( new PointCloudT );
	scale_and_color_point_cloud ( cloud_in, cloud_in_transformed );
	pcl::transformPointCloud ( *cloud_in_transformed, *cloud_in_transformed, transform_total );

	// step 8, filter out points belongs to the rivet and save them in cloud_rivet
	PointT minPoint, maxPoint;
  getMinMax3D ( *rivet_support_plane_cloud_transformed, minPoint, maxPoint );
	std::cout << "minPoint = " << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << std::endl;
	std::cout << "maxPoint = " << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << std::endl;
  r = 255, g = 0, b = 0;
	rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
	PointCloudT::Ptr cloud_rivet_above ( new PointCloudT );
	PointCloudT::Ptr cloud_rivet_below ( new PointCloudT );
	for ( PointT temp_point: cloud_in_transformed->points )
	{
		float x = temp_point.x;
		float x_compare = std::abs ( std::abs ( x ) - rivet_height_2 );
		float y = temp_point.y;
		float z = temp_point.z;
		if ( y >= minPoint.y && y <= maxPoint.y && z >= minPoint.z && z <= maxPoint.z && x_compare <= 0.0005 )
		{
	    PointT new_point;
	    new_point.x = x;
	    new_point.y = y;
	    new_point.z = z;
	    new_point.rgb = *reinterpret_cast<float*> ( &rgb );
			if ( x > 0 )
			{
				cloud_rivet_above->points.push_back ( new_point );
			}
			else
			{
				cloud_rivet_below->points.push_back ( new_point );
			}
		}
	}
	PointCloudT::Ptr cloud_rivet;
	if ( cloud_rivet_above->points.size() > cloud_rivet_below->points.size() )
	{
		cloud_rivet = cloud_rivet_above;
	}
	else
	{
		cloud_rivet = cloud_rivet_below;
	}

  // step 8.1, map points in cloud_rivet on the y, z plane
  PointCloudT::Ptr cloud_rivet_xy ( new PointCloudT );
  for ( PointT temp_point: cloud_rivet->points )
	{
		float x = temp_point.x;
		float y = temp_point.y;
		float z = temp_point.z;
		PointT new_point;
	  new_point.x = y;
	  new_point.y = z;
	  new_point.z = 0.0;
	  cloud_rivet_xy->points.push_back ( new_point );
	}

	// step 9, calculate the new central point by fitting a circle to the point cloud
  double min_circle_radius = 0.00145, max_circle_radius = 0.00155;
  ransac_thresh = 0.0003;
  Eigen::Vector3f center;
  float radius;
  inliers.clear ();
  get_circle ( cloud_rivet_xy, min_circle_radius, max_circle_radius, ransac_thresh, center, radius, inliers );

  if ( inliers.size () == 0 )
  {
    PCL_ERROR ( "Could not find a circle model from the given point cloud." );
    rivet_point_new_final ( 0 ) = NAN;
    return;
  }
  std::cout << "circle coefficients: [" << center.transpose() << "], radius = " << radius << std::endl;
  std::cout << "%%% inliers.size = " << inliers.size() << "; cloud_rivet_xy.size = " << cloud_rivet_xy->points.size() << std::endl;

  r = 255, g = 0, b = 0;
	rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
  for ( int const& idx: inliers )
	{
		PointT new_point;
    new_point.x = cloud_rivet->points [ idx ].x;
    new_point.y = cloud_rivet->points [ idx ].y;
    new_point.z = cloud_rivet->points [ idx ].z;
    new_point.rgb = *reinterpret_cast < float* > ( &rgb );
		rivet_cloud->points.push_back ( new_point );
  }

  // step 9.1, check the shape of the fitted circle
  if ( check_circle ( rivet_cloud ) == false )
  {
    rivet_point_new_final ( 0 ) = NAN;
    return;
  }

  Eigen::Vector4f rivet_center_point;
	rivet_center_point << rivet_height_2 + out_distance, center [0], center [1], 1.0;

	// step 10, save the point cloud of the rivet for visualization
  pcl::transformPointCloud ( *rivet_cloud, *rivet_cloud, transform_total_inverse );

	// step 11, transform the central point and in point back the world frame
	rivet_point_new_final = transform_total_inverse * rivet_center_point;
	Eigen::Vector4f rivet_point_in;
	rivet_point_in << -in_distance, rivet_center_point ( 1 ), rivet_center_point ( 2 ), 1.0;
	rivet_point_in_final = transform_total_inverse * rivet_point_in;
}

class RivetLocalizer
{
public:

  int check_theta ( PointCloudT::Ptr cloud_in_ )
  {
    PointCloudT::Ptr cloud_scaled ( new PointCloudT );
    scale_and_color_point_cloud ( cloud_in_, cloud_scaled );
    float plane_theta = calculate_theta ( cloud_scaled );
    std::cout << "$$$ plane_theta = " << plane_theta << std::endl;
    if ( plane_theta < 100.0 )
    {
      rotate_45 = true;
    }
  }

  int find_rivet ( PointCloudT::Ptr cloud_in )
  {
    if ( rotate_45 )
    {
      pcl::transformPointCloud ( *cloud_in, *cloud_in, r_x_45 );
    }

    // step 1, scaling the input point cloud
    PointCloudT::Ptr segment_cloud ( new PointCloudT );
    scale_and_color_point_cloud ( cloud_in, segment_cloud );

    // step 2, transform the input point cloud
    PointCloudT::Ptr segment_cloud_transformed ( new PointCloudT );
    Eigen::Matrix4f transform_1 ( Eigen::Matrix4f::Identity() );
    calculate_transform ( segment_cloud, transform_1 );
    pcl::transformPointCloud ( *segment_cloud, *segment_cloud_transformed, transform_1 );

		// step 3, find the plane
    Eigen::Vector3f surface_normal;
    float d_coef;
    float ransac_thresh = 0.004;
    std::vector < int > inliers;
    get_plane ( segment_cloud_transformed, ransac_thresh, surface_normal, d_coef, inliers );
    if ( inliers.size () == 0 )
    {
      PCL_ERROR ( "Could not estimate a planar model for the given dataset." );
      return ( -1 );
    }
    std::cout << "surface_normal : [" << surface_normal.transpose() << "], d_coef = " << d_coef << std::endl;

    r = 255, g = 0, b = 0;
    rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
    PointCloudT::Ptr plane_cloud	( new PointCloudT );
    for ( size_t i = 0; i < inliers.size (); ++i )
    {
      PointT new_point;
      new_point.x = segment_cloud_transformed->points[ inliers [ i ] ].x;
      new_point.y = segment_cloud_transformed->points[ inliers [ i ] ].y;
      new_point.z = segment_cloud_transformed->points[ inliers [ i ] ].z;
      new_point.rgb = *reinterpret_cast<float*>( &rgb );
      plane_cloud->points.push_back( new_point );
      segment_cloud_transformed->points[ inliers[ i ] ].rgb = new_point.rgb;
    }
    plane_cloud->header.frame_id = reference_frame;

    // step 4, do refined transformation to show the point cloud
    Eigen::Matrix4f transform_1_inverse ( Eigen::Matrix4f::Identity () );
    getInverseMatrix ( transform_1, transform_1_inverse );
    pcl::transformPointCloud ( *plane_cloud, *plane_cloud, transform_1_inverse );
    Eigen::Matrix4f transform_2 ( Eigen::Matrix4f::Identity() );
    calculate_transform ( plane_cloud, transform_2 );

    // step 5, check the transform_total to make sure the x axis is along the direction of the rivet
    Eigen::Matrix4f r_x_180, r_y_180, r_z_180;
    r_x_180 <<  1,  0,  0, 0,
                0, -1,  0, 0,
                0,  0, -1, 0,
                0,  0,  0, 1;
    r_y_180 << -1,  0,  0, 0,
                0,  1,  0, 0,
                0,  0, -1, 0,
                0,  0,  0, 1;
    r_z_180 << -1,  0,  0, 0,
                0, -1,  0, 0,
                0,  0,  1, 0,
                0,  0,  0, 1;
    Eigen::Matrix4f transform_total = transform_2;
    std::cout << "\t transform_total = \n" << transform_total << std::endl;
    int max_idx_2_r, max_idx_2_c;
    transform_total.block ( 1, 0, 1, 3 ).cwiseAbs().maxCoeff( &max_idx_2_r, &max_idx_2_c );
    std::cout << "max_idx_2_r, max_idx_2_c = " << max_idx_2_r << " " << max_idx_2_c << std::endl;
    if ( max_idx_2_c != 0 )
    {
      std::cout << "\t r_x_90" << std::endl;
      // rotate around x for 90 degrees
      Eigen::Matrix4f r_x_90;
      float theta_90 = 90.0 / 180.0 * PI;
      r_x_90 << 1,             0,              0, 0,
                0, cos(theta_90), -sin(theta_90), 0,
                0, sin(theta_90),  cos(theta_90), 0,
                0,             0,              0, 1;
      transform_total = r_x_90 * transform_total;
    }
    if ( transform_total ( 0, 1 ) < 0 && transform_total ( 1, 0 ) < 0 )
    {
      std::cout << "\t r_z_180 " << std::endl;
      transform_total = r_z_180 * transform_total;
    }
    else if ( transform_total ( 0, 1 ) < 0 && transform_total ( 1, 0 ) > 0 )
    {
      std::cout << "\t r_y_180 " << std::endl;
      transform_total = r_y_180 * transform_total;
    }
    else if ( transform_total ( 0, 1 ) > 0 && transform_total ( 1, 0 ) < 0 )
    {
      std::cout << "\t r_x_180 " << std::endl;
      transform_total = r_x_180 * transform_total;
    }
    std::cout << "\t transform_total = \n" << transform_total << std::endl;

    PointCloudT::Ptr cloud_in_transformed	( new PointCloudT );
    scale_and_color_point_cloud ( cloud_in, cloud_in_transformed );
    pcl::transformPointCloud ( *cloud_in_transformed, *cloud_in_transformed, transform_total );
    pcl::transformPointCloud ( *plane_cloud, *plane_cloud, transform_total );

    // step 6 get max and min points
    PointT minPoint, maxPoint;
    getMinMax3D ( *plane_cloud, minPoint, maxPoint );
    std::cout << "minPoint = " << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << std::endl;
    std::cout << "maxPoint = " << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << std::endl;

    // step 7, filter out points belongs to rivets
    r = 0, g = 255, b = 0;
    rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
    PointCloudT::Ptr cloud_rivet ( new PointCloudT );
    for ( PointT temp_point: cloud_in_transformed->points )
    {
      float x = temp_point.x;
      float x_compare = std::abs ( x - rivet_height );
      float y = temp_point.y;
      float z = temp_point.z;
      if ( y >= minPoint.y && y <= maxPoint.y && z >= minPoint.z && z <= maxPoint.z && x_compare <= 0.0015 )
      {
        PointT new_point;
        new_point.x = x;
        new_point.y = y;
        new_point.z = z;
        new_point.rgb = *reinterpret_cast<float*> ( &rgb );
        cloud_rivet->points.push_back ( new_point );
      }
    }
    cloud_rivet->header.frame_id = reference_frame;
    std::cout << "*** cloud_rivet = " << cloud_rivet->points.size() << std::endl;

    // step 8, partition the rivet cloud to seperate rivets
    pcl::KdTreeFLANN < PointT > kdtree;
    kdtree.setInputCloud ( cloud_rivet );
    UF cloud_rivet_uf ( cloud_rivet->points.size () );
    for ( size_t i = 0; i < cloud_rivet->points.size (); ++i )
    {
      PointT searchPoint = cloud_rivet->points [ i ];
      if ( cloud_rivet_uf.find ( i ) == i )
      {
        std::vector < int > pointIdx;
        std::vector < float > pointRadius;
        if ( kdtree.radiusSearch ( searchPoint, rivet_radius, pointIdx, pointRadius ) > 0 )
        {
        	for ( size_t j = 0; j < pointIdx.size (); ++j )
        	{
        		cloud_rivet_uf.merge ( i, pointIdx [ j ] );
        	}
        }
      }
    }
    std::map < int, std::list < int > > rivet_components = cloud_rivet_uf.get_components();

    std::vector < Eigen::Vector4f > rivet_vector;
    int rivet_counter = 0;
    for ( auto const& x : rivet_components )
    {
      // set a new color for the new rivet
      r = rivet_counter % 5 * 50;
      g = rivet_counter % 3 * 80;
      b = rivet_counter % 7 * 30;
      if ( r == 0 && g == 0 && b == 0 )
      {
      	r = 255; g = 125; b = 55;
      }
      rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );

      int component_id = x.first;
      std::list < int > component_element_list = x.second;
      double x_sum = 0, y_sum = 0, z_sum = 0;
      double y_max = -INFINITY, z_max = -INFINITY;

      for ( auto const& point_idx : component_element_list )
      {
      	PointT currentPoint = cloud_rivet->points [ point_idx ];
      	x_sum += currentPoint.x;
      	y_sum += currentPoint.y;
      	z_sum += currentPoint.z;
      	if ( y_max < currentPoint.y )
      	{
      		y_max = currentPoint.y;
      	}
      	if ( z_max < currentPoint.z )
      	{
      		z_max = currentPoint.z;
      	}
      	cloud_rivet->points[point_idx].rgb = *reinterpret_cast<float*> ( &rgb );
      }

    	int rivet_point_counter = component_element_list.size ();
    	double x_avg = x_sum / rivet_point_counter;
    	double y_avg = y_sum / rivet_point_counter;
    	double z_avg = z_sum / rivet_point_counter;
    	double _radius_1 = std::sqrt ( std::pow ( ( y_max - y_avg ) , 2 ) + std::pow ( ( z_max - z_avg ) , 2 ) ) * 2.0;
    	double _radius_2 = std::sqrt ( std::pow ( ( y_max - y_avg ) , 2 ) ) * 2.0;
    	double _radius_3 = std::sqrt ( std::pow ( ( z_max - z_avg ) , 2 ) ) * 2.0;
    	if ( _radius_1 > 0.003 && _radius_2 > 0.003 && _radius_3 > 0.003 )
    	{
    		Eigen::Vector4f rivet_point;
    		rivet_point << (x_avg + 0.01), y_avg, z_avg, 1.0;
    		rivet_vector.push_back ( rivet_point );
    		std::cout << "*** [" << rivet_counter << "] : rivet center = [" << x_avg << ", " << y_avg << ", " << z_avg << "] " << std::endl;
    	}
      rivet_counter ++;
    }
    std::cout << "rivet_vector.size() = " << rivet_vector.size() << std::endl;

    // step 9, generate motion control points
    ofstream point_rivet_fs;
    point_rivet_fs.open ( ros::package::getPath ( "object_localizer" ) + "/config/point_rivet.cfg" );
    rivet_counter = 0;
    PointCloudT::Ptr scene_cloud_total ( new PointCloudT );
    for ( Eigen::Vector4f rivet_point : rivet_vector )
    {
      std::cout << std::endl << std::endl;
      PointT search_point;
      search_point.x = 0.0;
      search_point.y = rivet_point ( 1 );
      search_point.z = rivet_point ( 2 );

      Eigen::Vector4f rivet_point_new_final, rivet_point_in_final;
      PointCloudT::Ptr rivet_support_plane_cloud ( new PointCloudT ), rivet_cloud ( new PointCloudT );
      double roll, pitch, yaw;
      get_rivet_center_orientation ( cloud_in, cloud_in_transformed, search_point, transform_total, rivet_support_plane_cloud, rivet_cloud, roll, pitch, yaw, rivet_point_new_final, rivet_point_in_final );

      if ( std::isnan ( rivet_point_new_final ( 0 ) ) )
      {
        continue;
      }

      *scene_cloud_total += *rivet_support_plane_cloud;
      *scene_cloud_total += *rivet_cloud;

      tf::Matrix3x3 old_rotation;
      old_rotation.setRPY ( roll, pitch, yaw );
      Eigen::Matrix4f old_rotation_matrix;
      old_rotation_matrix <<
              old_rotation[0][0], old_rotation[0][1], old_rotation[0][2], 0,
              old_rotation[1][0], old_rotation[1][1], old_rotation[1][2], 0,
              old_rotation[2][0], old_rotation[2][1], old_rotation[2][2], 0,
              0,  0,  0,  1;
      std::cout << "old_rotation_matrix = " << old_rotation_matrix << std::endl;

      if ( rotate_45 )
      {
        rivet_point_new_final = r_x_45_inv * rivet_point_new_final;
        rivet_point_in_final = r_x_45_inv * rivet_point_in_final;
        Eigen::Matrix4f new_rotation_matrix;
        new_rotation_matrix = r_x_45_inv * old_rotation_matrix;
        get_rpy_from_matrix ( new_rotation_matrix.block< 3, 3 >( 0, 0 ), roll, pitch, yaw );
      }

      std::cout << "*** [" << rivet_counter << "] : " << rivet_point_new_final.head< 3 >().transpose() << std::endl;
      show_frame ( "rivet_" + std::to_string ( rivet_counter ) + "_start", rivet_point_new_final ( 0 ), rivet_point_new_final ( 1 ), rivet_point_new_final ( 2 ), roll, pitch, yaw );
      show_frame ( "rivet_" + std::to_string ( rivet_counter ) + "_end", rivet_point_in_final ( 0 ), rivet_point_in_final ( 1 ), rivet_point_in_final ( 2 ), roll, pitch, yaw );
      point_rivet_fs << rivet_counter << " " << rivet_point_new_final ( 0 ) << " " << rivet_point_new_final ( 1 ) << " " << rivet_point_new_final ( 2 ) << " " << roll << " " << pitch << " " << yaw << std::endl;
      point_rivet_fs << rivet_counter << " " << rivet_point_in_final ( 0 ) << " " << rivet_point_in_final ( 1 ) << " " << rivet_point_in_final ( 2 ) << " " << roll << " " << pitch << " " << yaw << std::endl;
      point_rivet_fs << rivet_counter << " " << rivet_point_new_final ( 0 ) << " " << rivet_point_new_final ( 1 ) << " " << rivet_point_new_final ( 2 ) << " " << roll << " " << pitch << " " << yaw << std::endl;

      std::cout << std::endl << std::endl;
      rivet_counter ++;
    }
    point_rivet_fs.close();

    if ( rotate_45 )
    {
      pcl::transformPointCloud ( *scene_cloud_total, *scene_cloud_total, r_x_45_inv );
    }
    // step 10, show the point cloud
    if ( show_viz )
    {
      Visualize ( scene_cloud_total );
    }
	}

	// show the point cloud in rviz
  void Visualize ( PointCloudT::Ptr scene_cloud_total )
  {
    std::cout << "scene_cloud_total has [" << scene_cloud_total->size() << "] data points" << std::endl;
    if ( scene_cloud_total->width * scene_cloud_total->height > 0 )
    {
      int counter = 0;
      while ( counter < 3 )
      {
        scene_cloud_total->header.frame_id = reference_frame;
        pcl_conversions::toPCL ( ros::Time::now(), scene_cloud_total->header.stamp );
        cloud_pub_.publish ( scene_cloud_total );
        ros::Duration ( 0.01 ) .sleep ();
        counter ++;
      }
      std::cout << "***On Topic [/rivet_localizer/points], published [" << scene_cloud_total->width * scene_cloud_total->height << "] data points***" << std::endl;
    }
  }

  void CfgFileReader ()
  {
    std::string cfgFileName = ros::package::getPath ( "object_localizer" ) + "/config/rivet_localizer.cfg";
    std::cout << "***The path of the rivet_localizer configuration file is: [" << cfgFileName << "]" << std::endl;

    std::ifstream input ( cfgFileName );
    std::string line;
    if ( std::getline ( input, line ) )
    {
      std::istringstream iss ( line );
      iss >> SceneFileName;
      std::cout << "***Profile file name = [" << SceneFileName << "]" << std::endl;
    }
    if ( std::getline ( input, line ) )
    {
      std::istringstream iss ( line );
      iss >> filter_mean_k >> filter_stddev;
      std::cout << "***filter_mean_k = [" << filter_mean_k << "] filter_stddev = [" << filter_stddev << "]" << std::endl;
    }
    if ( std::getline ( input, line ) )
    {
      std::istringstream iss ( line );
      iss >> rivet_height >> rivet_radius;
      std::cout << "***rivet_height = [" << rivet_height << "] rivet_radius = [" << rivet_radius << "]" << std::endl;
    }
    if ( std::getline ( input, line ) )
    {
      if ( boost::starts_with ( line, "Yes" ) )
      {
        show_viz = true;
      }
      std::cout << "***Show Visualization = [" << show_viz << "]" << std::endl;
    }
    if ( std::getline ( input, line ) )
    {
      std::istringstream iss ( line );
      iss >> tool_angle;
      std::cout << "***tool_angle = [" << tool_angle << "]" << std::endl;
    }
    input.close();
  }

  bool start_rivet_localizer ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    // read the configuration file
    CfgFileReader ();

    // load the saved scene point cloud
    std::string SceneFilePath = ros::package::getPath ( "object_localizer" ) + "/data/" + SceneFileName;
    if ( pcl::io::loadPLYFile ( SceneFilePath, *scene_cloud_ ) == -1 )
    {
      ROS_ERROR_STREAM ( "Couldn't read the file: " << SceneFilePath << std::endl );
      return false;
    }
    std::cout << "Loaded " << scene_cloud_->width * scene_cloud_->height << " data points from " << SceneFilePath << std::endl;

    // process the saved scene point cloud
    check_theta ( scene_cloud_ );
    find_rivet ( scene_cloud_ );

    return true;
  }

  RivetLocalizer () : scene_cloud_ ( new pcl::PointCloud< PointT > )
  {
    start_rivet_localizer_ = nh_.advertiseService ( "start_rivet_localizer", &RivetLocalizer::start_rivet_localizer, this );

    std::string cloud_out_name = "/rivet_localizer/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_out_name, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << cloud_out_name );

    r_x_45 << 1,  0,  0,  0,
              0, cos(theta_x_45), -sin(theta_x_45), 0,
              0, sin(theta_x_45),  cos(theta_x_45), 0,
              0,  0,  0,  1;
    getInverseMatrix ( r_x_45, r_x_45_inv );
  }

  ~RivetLocalizer () { }

private:
  ros::NodeHandle nh_;
  pcl::PointCloud<PointT>::Ptr scene_cloud_;
  ros::ServiceServer start_rivet_localizer_;
  ros::Publisher cloud_pub_;

  bool rotate_45 = false;
  float theta_x_45 = 45.0 / 180.0 * PI;
  Eigen::Matrix4f r_x_45, r_x_45_inv;
};

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "rivet_localizer_c_15" );
  ros::AsyncSpinner spinner ( 2 );
  spinner.start ();
  RivetLocalizer rl;
  ros::waitForShutdown ();
  return 0;
}
