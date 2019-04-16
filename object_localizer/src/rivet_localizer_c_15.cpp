#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <cmath>
#include <boost/algorithm/string/predicate.hpp>

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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

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
float tool_angle = -15.0;

// define the union-find data structure
class UF
{
	int cnt, *id, *sz;
public:
	// Create an empty union find data structure with N isolated sets.
	UF ( int N )
	{
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
};

PointCloudT::Ptr scene_cloud_total	( new PointCloudT );
Eigen::Matrix4f transform_inverse ( Eigen::Matrix4f::Identity() );

// show the point cloud
void Visualize ( PointCloudT::Ptr cloud_in_transformed, PointCloudT::Ptr planar_cloud, PointCloudT::Ptr cloud_rivet )
{
	// pcl::visualization::PCLVisualizer viewer ( "Point Cloud Viewer" );
	// int v1 ( 0 );
	// viewer.createViewPort ( 0.0, 0.0, 1.0, 1.0, v1 );
	//
	// // add the point cloud to the viewer, can be updated by [ updatePointCloud () ]
	// pcl::visualization::PointCloudColorHandlerRGBField< PointT > cloud_color_i ( cloud_in_transformed );
	// viewer.addPointCloud ( cloud_in_transformed, cloud_color_i, "scene_cloud", v1 );
	// viewer.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_cloud" );
	// pcl::visualization::PointCloudColorHandlerRGBField< PointT > cloud_color_i_1 ( planar_cloud );
	// viewer.addPointCloud ( planar_cloud, cloud_color_i_1, "planar_cloud", v1 );
	// viewer.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "planar_cloud" );
	// pcl::visualization::PointCloudColorHandlerRGBField< PointT > cloud_color_i_2 ( cloud_rivet );
	// viewer.addPointCloud ( cloud_rivet, cloud_color_i_2, "cloud_rivet", v1 );
	// viewer.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_rivet" );
	//
	// // Set background color
	// viewer.setBackgroundColor ( 0.0, 0.0, 0.0, v1 );
	// viewer.addCoordinateSystem ( 0.1 );
	//
  // // Visualiser window size
  // viewer.setSize ( 1280, 1024 );
	//
	// // Display the viewer and wait for interactive events until the user closes the window.
	// while ( !viewer.wasStopped () )
  // {
  //   viewer.spinOnce ();
	// 	boost::this_thread::sleep ( boost::posix_time::microseconds ( 1000 ) );
	// }

	*scene_cloud_total += *cloud_in_transformed;
	*scene_cloud_total += *planar_cloud;
	*scene_cloud_total += *cloud_rivet;
	pcl::transformPointCloud ( *scene_cloud_total, *scene_cloud_total, transform_inverse );
	scene_cloud_total->header.frame_id = reference_frame;
	scene_cloud_total->width = cloud_in_transformed->size() + planar_cloud->size() + cloud_rivet->size();
	scene_cloud_total->height = 1;
	std::cout << "scene_cloud_total has [" << scene_cloud_total->size() << "] data points" << std::endl;
}

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
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors ();
  eigenVectorsPCA.col ( 2 ) = eigenVectorsPCA.col ( 0 ).cross ( eigenVectorsPCA.col ( 1 ) );

  std::cout << "eigen vector 0: [" << eigenVectorsPCA(0, 0) << ", " << eigenVectorsPCA(1, 0) << ", " << eigenVectorsPCA(2, 0) << "]" << std::endl;
  std::cout << "eigen vector 1: [" << eigenVectorsPCA(0, 1) << ", " << eigenVectorsPCA(1, 1) << ", " << eigenVectorsPCA(2, 1) << "]" << std::endl;
  std::cout << "eigen vector 2: [" << eigenVectorsPCA(0, 2) << ", " << eigenVectorsPCA(1, 2) << ", " << eigenVectorsPCA(2, 2) << "]" << std::endl;

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  projectionTransform.block< 3, 3 >( 0, 0 ) = eigenVectorsPCA.transpose();
  projectionTransform.block< 3, 1 >( 0, 3 ) = -1.0f * ( projectionTransform.block< 3, 3 >( 0, 0 ) * pcaCentroid.head< 3 >() );
}

void scale_and_color_point_cloud ( PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out )
{
	cloud_out->points.clear();
	int cloud_out_counter = 0;
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
    cloud_out_counter++;
  }
  cloud_out->width = cloud_out_counter;
  cloud_out->height = 1;
  cloud_out->header.frame_id = reference_frame;
}

void getMinMax3D (const pcl::PointCloud<PointT> &cloud, PointT &min_pt, PointT &max_pt)
{
  Eigen::Array4f min_p, max_p;
  min_p.setConstant (FLT_MAX);
  max_p.setConstant (-FLT_MAX);

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
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
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = reference_frame;
  transformStamped.child_frame_id = frame_name;
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  ros::Rate loop_rate(10);
  for ( int i = 0; i < 10; i++ )
	{
      br.sendTransform(transformStamped);
      loop_rate.sleep();
  }
  std::cout << "\tFrame " << frame_name << " is added\n";
}

void get_rpy_from_matrix ( Eigen::Matrix3f rotation_matrix, double& roll, double& pitch, double& yaw )
{
  tf::Matrix3x3 object_m ( rotation_matrix (0, 0), rotation_matrix (0, 1), rotation_matrix (0, 2), rotation_matrix (1, 0), rotation_matrix (1, 1), rotation_matrix (1, 2), rotation_matrix (2, 0), rotation_matrix (2, 1), rotation_matrix (2, 2) );
  object_m.getRPY ( roll, pitch, yaw );
}

//##############################################################################################################################################################
PointCloudT::Ptr rivet_cloud_new	( new PointCloudT );

// find new center of a rivet by find a new rivet point cloud
void find_new_rivet_center ( PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_rivet_cycle, Eigen::Matrix4f transform_total, Eigen::Matrix4f transform_1, Eigen::Vector4f& rivet_point_new )
{
	// step 1, prepare the transformed whole point cloud
	PointCloudT::Ptr cloud_in_transformed	( new PointCloudT );
	scale_and_color_point_cloud ( cloud_in, cloud_in_transformed );
	pcl::transformPointCloud( *cloud_in_transformed, *cloud_in_transformed, transform_total );

	// step 2, filter out points belongs to the rivet and save them in cloud_rivet
	pcl::PointXYZRGB minPoint, maxPoint;
  getMinMax3D ( *cloud_rivet_cycle, minPoint, maxPoint );
	std::cout << "minPoint = " << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << std::endl;
	std::cout << "maxPoint = " << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << std::endl;
	uint8_t r = 255, g = 0, b = 0;
	uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
	PointCloudT::Ptr cloud_rivet_p ( new PointCloudT );
	int cloud_rivet_p_counter = 0;
	PointCloudT::Ptr cloud_rivet_m ( new PointCloudT );
	int cloud_rivet_m_counter = 0;
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
				cloud_rivet_p->points.push_back ( new_point );
		    cloud_rivet_p_counter ++;
			}
			else
			{
				cloud_rivet_m->points.push_back ( new_point );
		    cloud_rivet_m_counter ++;
			}
		}
	}
	cloud_rivet_p->width = cloud_rivet_p_counter;
  cloud_rivet_p->height = 1;
  cloud_rivet_p->header.frame_id = reference_frame;
	cloud_rivet_m->width = cloud_rivet_m_counter;
  cloud_rivet_m->height = 1;
  cloud_rivet_m->header.frame_id = reference_frame;

	PointCloudT::Ptr cloud_rivet;
	if ( cloud_rivet_p_counter > cloud_rivet_m_counter )
	{
		cloud_rivet = cloud_rivet_p;
	}
	else
	{
		cloud_rivet = cloud_rivet_m;
	}

	// step 3, calculate the new central point
	double x_sum = 0;
	double y_sum = 0;
	double z_sum = 0;
	int point_counter = 0;
	for ( PointT temp_point: cloud_rivet->points )
	{
		x_sum += temp_point.x;
		y_sum += temp_point.y;
		z_sum += temp_point.z;
		point_counter++;
	}
	double x_avg = x_sum / point_counter;
	double y_avg = y_sum / point_counter;
	double z_avg = z_sum / point_counter;
	rivet_point_new << x_avg + out_distance, y_avg, z_avg, 1.0;

	// step 4, save the point cloud of the rivet for visualization
	Eigen::Matrix4f transform_total_inverse ( Eigen::Matrix4f::Identity() );
	getInverseMatrix ( transform_total, transform_total_inverse );
	pcl::transformPointCloud ( *cloud_rivet, *cloud_rivet, transform_total_inverse );
	pcl::transformPointCloud ( *cloud_rivet, *cloud_rivet, transform_1 );
	*rivet_cloud_new = *rivet_cloud_new + *cloud_rivet;
}

// calculate the orientation and new start point
void calculate_Orientation ( PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_rivet_cycle, Eigen::Matrix4f transform_1, Eigen::Matrix4f transform_2, double& roll, double& pitch, double& yaw, Eigen::Vector4f& rivet_point, Eigen::Vector4f& rivet_point_new_final, Eigen::Vector4f& rivet_point_in_final )
{

	// step 1, calculate the new transformation
	Eigen::Matrix4f r_x_180, r_z_180;
	r_x_180 <<  1,  0,  0, 0,
							0, -1,  0, 0,
							0,  0, -1, 0,
							0,  0,  0, 1;
	r_z_180 << -1,  0,  0, 0,
							0, -1,  0, 0,
							0,  0,  1, 0,
							0,  0,  0, 1;
	Eigen::Matrix4f transform_total = transform_2 * transform_1;
	float min_v = std::min ( std::min ( std::abs ( transform_total (0, 0) ), std::abs ( transform_total (0, 1) ) ), std::abs ( transform_total (0, 2) ) );
	if ( std::abs ( transform_total (0, 0) ) == min_v && transform_total (0, 1) < 0 and transform_total (0, 2) > 0 )
	{
		transform_total = r_z_180 * transform_total;
	}
	if ( transform_total (1, 0) < 0 )
	{
		transform_total = r_x_180 * transform_total;
	}

	// step 2, calculate roll, pitch, yaw from the new transformation
	Eigen::Matrix4f transform_total_inverse ( Eigen::Matrix4f::Identity() );
	getInverseMatrix ( transform_total, transform_total_inverse );
	get_rpy_from_matrix ( transform_total_inverse.block< 3, 3 >( 0, 0 ), roll, pitch, yaw );
	while ( roll < 0.0 )
	{
		roll = 3.1415926 + roll;
	}
	if ( roll > 0 && roll <= 3.1415926/2 )
	{
		roll = roll + 3.1415926;
	}
	if ( roll > 3.1415926/2.0 && roll <= 3.1415926/4.0*3.0 )
	{
		roll = roll + 3.1415926/2;
	}
	if ( roll > 3.1415926 + 0.2 && roll <= 3.1415926 + 3.1415926/2 + 0.5 )
	{
		roll = roll - 3.1415926/2;
	}
	float theta_x = 3.14159265359 - roll;
	Eigen::Matrix4f r_x_theta, r_x_15;
	r_x_theta << 1,   0,  0, 0,
					     0, cos(theta_x), -sin(theta_x), 0,
					     0, sin(theta_x), cos(theta_x), 0,
						   0,  0,  0, 1;
	theta_x = tool_angle/180.0*3.14159265359;
  r_x_15     << 1,   0,  0, 0,
					     0, cos(theta_x), -sin(theta_x), 0,
					     0, sin(theta_x), cos(theta_x), 0,
						   0,  0,  0, 1;
	transform_total = r_x_15 * r_x_theta * transform_total;
	getInverseMatrix ( transform_total, transform_total_inverse );
	get_rpy_from_matrix ( transform_total_inverse.block< 3, 3 >( 0, 0 ), roll, pitch, yaw );
	roll = 3.14159265359 + theta_x;
	std::cout << "\troll, pitch, yaw = " << roll << ", " << pitch << ", " << yaw << std::endl;

	// step 3, calculate the new central point
	Eigen::Vector4f rivet_point_new;
	Eigen::Matrix4f transform_1_inverse ( Eigen::Matrix4f::Identity() );
	getInverseMatrix ( transform_1, transform_1_inverse );
	transform_inverse = transform_1_inverse;
	pcl::transformPointCloud ( *cloud_rivet_cycle, *cloud_rivet_cycle, transform_1_inverse );
	pcl::transformPointCloud ( *cloud_rivet_cycle, *cloud_rivet_cycle, transform_total );
	find_new_rivet_center ( cloud_in, cloud_rivet_cycle, transform_total, transform_1, rivet_point_new );

	// step 4, transform the central point and in point back the world frame
	rivet_point_new_final = transform_total_inverse * rivet_point_new;
	Eigen::Vector4f rivet_point_in;
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	rivet_point_in << -in_distance, rivet_point_new ( 1 ), rivet_point_new ( 2 ), 1.0;
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	rivet_point_in_final = transform_total_inverse * rivet_point_in;
}

// find the points near the search point
void find_near_point ( PointCloudT::Ptr cloud_in_transformed, PointCloudT::Ptr cloud_rivet_cycle, PointT &searchPoint )
{
	pcl::KdTreeFLANN < PointT > kdtree;
  kdtree.setInputCloud ( cloud_in_transformed );
	float search_radius = 0.006;
	std::vector<int> pointIdx;
	std::vector<float> pointRadius;

	int cloud_rivet_cycle_counter = 0;
	uint8_t r = 0, g = 255, b = 0;
	uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );

	if ( kdtree.radiusSearch ( searchPoint, search_radius, pointIdx, pointRadius ) > 0 )
	{
		for ( size_t j = 0; j < pointIdx.size (); ++j )
		{
			PointT new_point;
			new_point.x = cloud_in_transformed->points[ pointIdx[j] ].x;
			new_point.y = cloud_in_transformed->points[ pointIdx[j] ].y;
			new_point.z = cloud_in_transformed->points[ pointIdx[j] ].z;
			new_point.rgb = *reinterpret_cast<float*> ( &rgb );
			cloud_in_transformed->points[ pointIdx[j] ].rgb = *reinterpret_cast<float*> ( &rgb );
			cloud_rivet_cycle->points.push_back ( new_point );
			cloud_rivet_cycle_counter ++;
		}
		cloud_rivet_cycle->header.frame_id = reference_frame;
	}
}

PointCloudT::Ptr cycle_planar_cloud	( new PointCloudT );

// the start function for find orientation and new central point of a rivet
void getRivetOrientation ( PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_in_transformed, PointT &searchPoint, Eigen::Matrix4f transform_1, double& roll, double& pitch, double& yaw, Eigen::Vector4f& rivet_point, Eigen::Vector4f& rivet_point_new_final, Eigen::Vector4f& rivet_point_in_final )
{
	// step 1, search for the points on the support surface within the search_radius
	PointCloudT::Ptr cloud_rivet_cycle ( new PointCloudT );
	PointT searchPoint_new_1;
	searchPoint_new_1.x = 0.0;
	searchPoint_new_1.y = searchPoint.y;
	searchPoint_new_1.z = searchPoint.z - 0.003;
	find_near_point ( cloud_in_transformed, cloud_rivet_cycle, searchPoint_new_1 );
	PointT searchPoint_new_2;
	searchPoint_new_2.x = 0.0;
	searchPoint_new_2.y = searchPoint.y;
	searchPoint_new_2.z = searchPoint.z + 0.003;
	find_near_point ( cloud_in_transformed, cloud_rivet_cycle, searchPoint_new_2 );

	//step 2, fit a plane with cloud_rivet_cycle
	uint8_t r = 0, g = 255, b = 0;
	uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
	pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
  pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
  pcl::SACSegmentation < PointT > seg;
  seg.setOptimizeCoefficients ( true );
  seg.setModelType ( pcl::SACMODEL_PLANE );
  seg.setMethodType ( pcl::SAC_RANSAC );
  seg.setDistanceThreshold ( 0.0002 );
  seg.setInputCloud ( cloud_rivet_cycle );
  seg.segment ( *inliers, *coefficients );
  if ( inliers->indices.size () == 0 )
  {
    PCL_ERROR ( "Could not estimate a planar model for the given dataset." );
		return;
  }
	float planar_coef [3] = { coefficients->values [0], coefficients->values [1], coefficients->values [2] };
	float d_coef = coefficients->values[3];
  std::cerr << "Planar coefficients: [" << planar_coef [0] << ", " << planar_coef [1] << ", " << planar_coef [2] << "], d_coef = " << d_coef << std::endl;
  std::cerr << "Planar inliers: " << inliers->indices.size () << std::endl;

	// step 3 show the cycle cloud fit the plane
	PointCloudT::Ptr cycle_planar_cloud_tmp	( new PointCloudT );
	cycle_planar_cloud_tmp->header.frame_id = reference_frame;
	r = 255, g = 255, b = 0;
	rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
  for ( size_t i = 0; i < inliers->indices.size (); ++i )
	{
		PointT new_point;
    new_point.x = cloud_rivet_cycle->points[ inliers->indices[ i ] ].x;
    new_point.y = cloud_rivet_cycle->points[ inliers->indices[ i ] ].y;
    new_point.z = cloud_rivet_cycle->points[ inliers->indices[ i ] ].z;
    new_point.rgb = *reinterpret_cast<float*>( &rgb );
    cycle_planar_cloud->points.push_back( new_point );
		cycle_planar_cloud_tmp->points.push_back( new_point );
  }

	// step 4, calculate orientation and in point
	Eigen::Matrix4f transform_2 ( Eigen::Matrix4f::Identity() );
	calculate_transform ( cycle_planar_cloud_tmp, transform_2 );
	calculate_Orientation ( cloud_in, cloud_rivet_cycle, transform_1, transform_2, roll, pitch, yaw, rivet_point, rivet_point_new_final, rivet_point_in_final );
}
//##############################################################################################################################################################

int find_rivet ( PointCloudT::Ptr cloud_in )
{
	// step 1, filter, downsampling, and scaling the input point cloud and change the color of each point
	PointCloudT::Ptr cloud_filtered	( new PointCloudT );
	PointCloudT::Ptr segment_cloud ( new PointCloudT );
  // filterOutliner ( cloud_in );
	downSampling ( cloud_in, cloud_filtered );
	scale_and_color_point_cloud ( cloud_filtered, segment_cloud );

	// step 2, transform the input point cloud
	PointCloudT::Ptr segment_cloud_transformed ( new PointCloudT );
	Eigen::Matrix4f transform_1 ( Eigen::Matrix4f::Identity() );
	calculate_transform ( segment_cloud, transform_1 );
	pcl::transformPointCloud( *segment_cloud, *segment_cloud_transformed, transform_1 );

	// step 3, find the planar
	pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
  pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
  pcl::SACSegmentation < PointT > seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType ( pcl::SACMODEL_PLANE );
  seg.setMethodType ( pcl::SAC_RANSAC );
  seg.setDistanceThreshold ( 0.0005 );
  seg.setInputCloud ( segment_cloud_transformed );
  seg.segment ( *inliers, *coefficients );
  if ( inliers->indices.size () == 0 )
  {
    PCL_ERROR ( "Could not estimate a planar model for the given dataset." );
    return ( -1 );
  }
	float planar_coef [3] = { coefficients->values [0], coefficients->values [1], coefficients->values [2] };
	float d_coef = coefficients->values[3];
  std::cerr << "Planar coefficients: " << planar_coef [0] << " " << planar_coef [1] << " " << planar_coef [2] << " " << d_coef << std::endl;
  std::cerr << "Planar inliers: " << inliers->indices.size () << std::endl;
	uint8_t r = 255, g = 0, b = 0;
	uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
	PointCloudT::Ptr planar_cloud	( new PointCloudT );
	int planar_cloud_counter = 0;
  for ( size_t i = 0; i < inliers->indices.size (); ++i )
	{
		segment_cloud_transformed->points[ inliers->indices[ i ] ].rgb = *reinterpret_cast<float*>( &rgb );
		PointT new_point;
    new_point.x = segment_cloud_transformed->points[ inliers->indices[ i ] ].x;
    new_point.y = segment_cloud_transformed->points[ inliers->indices[ i ] ].y;
    new_point.z = segment_cloud_transformed->points[ inliers->indices[ i ] ].z;
    new_point.rgb = *reinterpret_cast<float*>( &rgb );
    planar_cloud->points.push_back( new_point );
    planar_cloud_counter++;
  }
  planar_cloud->width = planar_cloud_counter;
  planar_cloud->height = 1;
  planar_cloud->header.frame_id = reference_frame;

	// step 4, do refined transformation to show the point cloud
	Eigen::Matrix4f transform_2 ( Eigen::Matrix4f::Identity() );
	calculate_transform ( planar_cloud, transform_2 );
	// filterOutliner ( planar_cloud );
	pcl::transformPointCloud ( *planar_cloud, *planar_cloud, transform_2 );

	// step 5, get the total transformation and transform the whole profile scan
	Eigen::Matrix4f r_x_180, r_z_180;
	r_x_180 <<  1,  0,  0, 0,
              0, -1,  0, 0,
              0,  0, -1, 0,
              0,  0,  0, 1;
	r_z_180 << -1,  0,  0, 0,
              0, -1,  0, 0,
              0,  0,  1, 0,
              0,  0,  0, 1;
	std::cout << "r_x_180 = \n" << r_x_180 << "\n r_z_180 = \n" << r_z_180 << std::endl;
	Eigen::Matrix4f transform_total = transform_2 * transform_1;
	std::cout << "transform_2 = \n" << transform_2 << std::endl;
	std::cout << "transform_1 = \n" << transform_1 << std::endl;
	std::cout << "transform_total = \n" << transform_total << std::endl;
	Eigen::Matrix4f transform_total_inverse_temp ( Eigen::Matrix4f::Identity() );
	getInverseMatrix ( transform_total, transform_total_inverse_temp );
	std::cout << "transform_total_inverse_temp = \n" << transform_total_inverse_temp << std::endl;
	pcl::transformPointCloud ( *planar_cloud, *planar_cloud, transform_total_inverse_temp );
	float min_v = std::min ( std::min ( std::abs ( transform_total (0, 0) ), std::abs ( transform_total (0, 1) ) ),
													 std::abs ( transform_total (0, 2) ) );
	if ( std::abs ( transform_total (0, 0) ) == min_v && transform_total (0, 1) < 0 and transform_total (0, 2) > 0 )
	{
		transform_total = r_z_180 * transform_total;
		std::cout << "new transform_total after [r_z_180] = \n" << transform_total << std::endl;
	}
	if ( transform_total (1, 0) < 0 )
	{
		transform_total = r_x_180 * transform_total;
		std::cout << "new transform_total after [r_x_180] = \n" << transform_total << std::endl;
	}
	std::cout << "new transform_total = \n" << transform_total << std::endl;
	PointCloudT::Ptr cloud_in_transformed	( new PointCloudT );
	scale_and_color_point_cloud ( cloud_in, cloud_in_transformed );
	pcl::transformPointCloud( *cloud_in_transformed, *cloud_in_transformed, transform_total );
	pcl::transformPointCloud ( *planar_cloud, *planar_cloud, transform_total );
	pcl::PointXYZRGB minPoint, maxPoint;
  getMinMax3D ( *planar_cloud, minPoint, maxPoint );
	std::cout << "minPoint = " << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << std::endl;
	std::cout << "maxPoint = " << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << std::endl;

	// step 6, check the x axis of the planar after the total transformation
	float planar_coef_abs [3] = { std::abs( planar_coef [0] ), std::abs( planar_coef [1] ), std::abs( planar_coef [2] ) };
	int max_idx = std::distance ( planar_coef_abs, std::max_element ( planar_coef_abs, planar_coef_abs + 3 ) );
	std::cout << "***Max_idx = " << max_idx << std::endl;

	// step 7, filter out points belongs to rivets
	r = 0, g = 255, b = 0;
	rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
	PointCloudT::Ptr cloud_rivet ( new PointCloudT );
	int cloud_rivet_counter = 0;
	// double sum_x = 0.0;
	if ( max_idx == 0 )
	{
		for ( PointT temp_point: cloud_in_transformed->points )
	  {
			float x = temp_point.x;
			float x_compare = std::abs ( std::abs ( x ) - rivet_height );
			float y = temp_point.y;
			float z = temp_point.z;
			if ( y >= minPoint.y && y <= maxPoint.y && z >= minPoint.z && z <= maxPoint.z
					 && x_compare <= 0.0015 && x > 0 )
			{
		    PointT new_point;
				// sum_x += x;
		    new_point.x = x;
		    new_point.y = y;
		    new_point.z = z;
		    new_point.rgb = *reinterpret_cast<float*> ( &rgb );
		    cloud_rivet->points.push_back ( new_point );
		    cloud_rivet_counter ++;
			}
	  }
	}
	cloud_rivet->width = cloud_rivet_counter;
  cloud_rivet->height = 1;
  cloud_rivet->header.frame_id = reference_frame;
	std::cout << "***cloud_rivet has " << cloud_rivet_counter << " data points" << std::endl;
	// std::cout << "***average x = " << sum_x / cloud_rivet_counter << std::endl;

	// step 8, partition the rivet cloud to seperate rivets
	pcl::KdTreeFLANN < PointT > kdtree;
  kdtree.setInputCloud ( cloud_rivet );
	std::vector< Eigen::Vector4f > rivet_vector;

	UF cloud_rivet_uf ( cloud_rivet_counter );
	int rivet_counter = 0;
	for ( size_t i = 0; i < cloud_rivet->points.size (); ++i )
	{
		PointT searchPoint = cloud_rivet->points[i];
		if ( cloud_rivet_uf.find ( i ) == i )
		{
			// search for points within some distance
			std::vector<int> pointIdx;
			std::vector<float> pointRadius;
			double x_sum = searchPoint.x;
			double y_sum = searchPoint.y;
			double z_sum = searchPoint.z;
			double x_max = searchPoint.x;
			double y_max = searchPoint.y;
			double z_max = searchPoint.z;
			if ( kdtree.radiusSearch ( searchPoint, rivet_radius, pointIdx, pointRadius ) > 0 )
			{
				rivet_counter++;
				r = rivet_counter % 5 * 50;
				g = rivet_counter % 3 * 80;
				b = rivet_counter % 7 * 30;
				if ( r == 0 && g == 0 && b == 0 )
				{
					r = 255;
					g = 125;
					b = 55;
				}
				rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
				for ( size_t j = 0; j < pointIdx.size (); ++j )
				{
					cloud_rivet_uf.merge ( i, pointIdx[j] );
					x_sum += cloud_rivet->points[ pointIdx[j] ].x;
					y_sum += cloud_rivet->points[ pointIdx[j] ].y;
					z_sum += cloud_rivet->points[ pointIdx[j] ].z;
					if ( y_max < cloud_rivet->points[ pointIdx[j] ].y )
					{
						y_max = cloud_rivet->points[ pointIdx[j] ].y;
					}
					if ( z_max < cloud_rivet->points[ pointIdx[j] ].z )
					{
						z_max = cloud_rivet->points[ pointIdx[j] ].z;
					}
					cloud_rivet->points[ pointIdx[j] ].rgb = *reinterpret_cast<float*> ( &rgb );
				}
				int rivet_point_counter = pointIdx.size () + 1;
				double x_avg = x_sum / rivet_point_counter;
				double y_avg = y_sum / rivet_point_counter;
				double z_avg = z_sum / rivet_point_counter;
				double real_radius = std::sqrt ( std::pow ( ( y_max - y_avg ) , 2 ) + std::pow ( ( z_max - z_avg ) , 2 ) ) * 2.0;
				double real_radius_2 = std::sqrt ( std::pow ( ( y_max - y_avg ) , 2 ) ) * 2.0;
				double real_radius_3 = std::sqrt ( std::pow ( ( z_max - z_avg ) , 2 ) ) * 2.0;
				if ( real_radius > 0.003 && real_radius_2 > 0.003 && real_radius_3 > 0.003 )
				{
					Eigen::Vector4f rivet_point;
					rivet_point << (x_avg + 0.01), y_avg, z_avg, 1.0;
					rivet_vector.push_back ( rivet_point );
					std::cout << "*** [" << rivet_counter << "] : rivet center = [" << x_avg << ", "
					 					<< y_avg << ", " << z_avg << "]" << std::endl;
					// std::cout << "*** [" << rivet_counter << "] : rivet center = [" << rivet_point << std::endl;
					std::cout << "*** real_radius = " << real_radius << std::endl;
				}
			}
		}
	}
	std::cout << "rivet_vector.size() = " << rivet_vector.size() << std::endl;

	// step 9, get the total inverse transformation matrix
	Eigen::Matrix4f transform_total_inverse ( Eigen::Matrix4f::Identity() );
	getInverseMatrix ( transform_total, transform_total_inverse );

  //##########################################################################################################################################################
	Eigen::Matrix4f r_x_2, r_z_2;
	float theta_x = -0*3.14159/180;
	float theta = -2*3.14159/180;
	r_x_2 << 1,   0,  0, 0,
					 0, cos(theta_x), -sin(theta_x), 0,
					 0, sin(theta_x), cos(theta_x), 0,
						0,  0,  0, 1;
	r_z_2 << cos(theta), -sin(theta),  0, 0,
					 sin(theta),  cos(theta),  0, 0,
						0,  0,  1, 0,
						0,  0,  0, 1;
	std::cout << "r_z_2 = \n" << r_z_2 << std::endl;
	transform_total_inverse =  transform_total_inverse * r_z_2 * r_x_2;
	//##########################################################################################################################################################

	double roll, pitch, yaw;
	get_rpy_from_matrix ( transform_total_inverse.block< 3, 3 >( 0, 0 ), roll, pitch, yaw );
	while ( roll < 0.0 )
	{
		roll = 3.1415926 + roll;
	}
	if ( roll > 0 && roll <= 3.1415926/2 )
	{
		roll = roll + 3.1415926;
	}
	if ( roll > 3.1415926/2.0 && roll <= 3.1415926/4.0*3.0 )
	{
		roll = roll + 3.1415926/2;
	}
	std::cout << "transform_total_inverse = \n" << transform_total_inverse << std::endl;
	std::cout << "roll, pitch, yaw = \n" << roll << ", " << pitch << ", " << yaw << std::endl;

	// step 10, generate motion control points
	ofstream point_rivet_fs;
  std::string cfgFileName = ros::package::getPath ( "object_localizer" ) + "/config/point_rivet.cfg";
  point_rivet_fs.open ( cfgFileName );
	rivet_counter = 0;
	std::vector< Eigen::Vector4f > rivet_vector_final;
	for ( Eigen::Vector4f rivet_point : rivet_vector )
	{
		PointT searchPoint_new;
		searchPoint_new.x = 0.0;
		searchPoint_new.y = rivet_point ( 1 );
		searchPoint_new.z = rivet_point ( 2 );

		Eigen::Vector4f rivet_point_new_final, rivet_point_in_final;
		getRivetOrientation ( cloud_in, cloud_in_transformed, searchPoint_new, transform_total, roll, pitch, yaw, rivet_point, rivet_point_new_final,  rivet_point_in_final);

		if ( std::isnan( rivet_point_new_final ( 0 ) ) )
		{
			continue;
		}

		std::cout << "*** [" << rivet_counter << "] : " << rivet_point_new_final.head< 3 >().transpose() << std::endl;

		show_frame ( "rivet_" + std::to_string ( rivet_counter ) + "_start", rivet_point_new_final ( 0 ), rivet_point_new_final ( 1 ), rivet_point_new_final ( 2 ), roll, pitch, yaw );
		show_frame ( "rivet_" + std::to_string ( rivet_counter ) + "_end", rivet_point_in_final ( 0 ), rivet_point_in_final ( 1 ), rivet_point_in_final ( 2 ), roll, pitch, yaw );

		point_rivet_fs << rivet_counter << " " << rivet_point_new_final ( 0 ) << " " << rivet_point_new_final ( 1 ) << " " << rivet_point_new_final ( 2 ) << " " << roll << " " << pitch << " " << yaw << std::endl;

		point_rivet_fs << rivet_counter << " " << rivet_point_in_final ( 0 ) << " " << rivet_point_in_final ( 1 ) << " " << rivet_point_in_final ( 2 ) << " " << roll << " " << pitch << " " << yaw << std::endl;

		point_rivet_fs << rivet_counter << " " << rivet_point_new_final ( 0 ) << " " << rivet_point_new_final ( 1 ) << " " << rivet_point_new_final ( 2 ) << " " << roll << " " << pitch << " " << yaw << std::endl;

		rivet_counter ++;
	}
	point_rivet_fs.close();

	// step 11, show the point cloud
	if ( show_viz )
	{
		Visualize ( cloud_in_transformed, cycle_planar_cloud, rivet_cloud_new );
	}
}

class RivetLocalizer
{
public:

  void handle_scene_point_cloud ( )
  {
    // load saved scene point cloud
    std::string SceneFilePath = ros::package::getPath ( "object_localizer" ) + "/data/" + SceneFileName;
    if ( pcl::io::loadPLYFile ( SceneFilePath, *scene_cloud_ ) == -1 )
    {
      ROS_ERROR_STREAM ( "Couldn't read file: " << SceneFilePath << std::endl );
      return;
    }
    std::cout << "Loaded " << scene_cloud_->width * scene_cloud_->height << " data points from " << SceneFilePath << std::endl;
    find_rivet ( scene_cloud_ );

		if ( scene_cloud_total->width * scene_cloud_total->height > 0  )
		{
			int counter = 0;
			while ( counter < 15 )
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

	bool start_rivet_localizer ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    handle_scene_point_cloud ();
    return true;
  }

  RivetLocalizer () : scene_cloud_ ( new pcl::PointCloud< PointT > )
	{
		start_rivet_localizer_ = nh_.advertiseService ( "start_rivet_localizer", &RivetLocalizer::start_rivet_localizer, this );

		std::string cloud_out_name = "/rivet_localizer/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_out_name, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << cloud_out_name );
	}

  ~RivetLocalizer () { }

private:
  ros::NodeHandle nh_;
  pcl::PointCloud<PointT>::Ptr scene_cloud_;
	ros::ServiceServer start_rivet_localizer_;
	ros::Publisher cloud_pub_;
};

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
		if ( !boost::starts_with ( line, "#" ) )
		{
	    std::istringstream iss ( line );
	    iss >> filter_mean_k >> filter_stddev;
	    std::cout << "***filter_mean_k = [" << filter_mean_k << "] filter_stddev = [" << filter_stddev << "]" << std::endl;
		}
  }
	if ( std::getline ( input, line ) )
  {
		if ( !boost::starts_with ( line, "#" ) )
		{
			std::istringstream iss ( line );
	    iss >> rivet_height >> rivet_radius;
	    std::cout << "***rivet_height = [" << rivet_height << "] rivet_radius = [" << rivet_radius << "]" << std::endl;
		}
  }
	if ( std::getline ( input, line ) )
  {
		if ( boost::starts_with ( line, "Yes" ) )
		{
			show_viz = true;
		}
		std::cout << "***Show Visualization = [" << show_viz << "]" << std::endl;
  }
  input.close();
}

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "rivet_localizer" );
	ros::AsyncSpinner spinner ( 2 );
  spinner.start ();
	CfgFileReader ();
	RivetLocalizer rl;
	ros::waitForShutdown ();
  return 0;
}
