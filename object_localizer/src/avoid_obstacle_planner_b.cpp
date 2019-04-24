#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <boost/algorithm/string/predicate.hpp>
#include <map>

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
float x_1, x_2, x_3, x_4, y_offset, y_1, y_2, y_3, y_back, z_offset, z_1, z_2, z_3, deg_1, deg_2, deg_3;

// function used to show the point cloud
void Visualize ( PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_cut )
{
	pcl::visualization::PCLVisualizer viewer ( "Point Cloud Viewer" );
	int v1 ( 0 );
	viewer.createViewPort ( 0.0, 0.0, 1.0, 1.0, v1 );

	// add the point cloud to the viewer, can be updated by [ updatePointCloud () ]
	pcl::visualization::PointCloudColorHandlerRGBField < PointT > cloud_color_i ( cloud_in );
	viewer.addPointCloud ( cloud_in, cloud_color_i, "scene_cloud", v1 );
	viewer.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_cloud" );
	pcl::visualization::PointCloudColorHandlerRGBField < PointT > cloud_color_i_1 ( cloud_cut );
	viewer.addPointCloud ( cloud_cut, cloud_color_i_1, "cloud_cut", v1 );
	viewer.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_cut" );

	// Set background color & Visualiser window size
	viewer.setBackgroundColor ( 0.0, 0.0, 0.0, v1 );
	viewer.addCoordinateSystem ( 0.1 );
  viewer.setSize ( 1280, 1024 );
	// Display the viewer and wait for interactive events until the user closes the window.
	while ( !viewer.wasStopped () )
  {
    viewer.spinOnce ();
		boost::this_thread::sleep ( boost::posix_time::microseconds ( 100000 ) );
	}
}

// downsampling an input point cloud
void downSampling ( PointCloudT::Ptr cloud, PointCloudT::Ptr cloud_sampled )
{
  static pcl::VoxelGrid<PointT> grid;
  grid.setInputCloud ( cloud );
  grid.setLeafSize ( 0.005f, 0.005f, 0.005f );
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

class Obstacle
{
public:

  float x_max, x_min, y_max, y_min, z;
	int point_counter;

  Obstacle ( float x_max, float x_min, float y_max, float y_min, float z, int point_counter )
  {
    this->x_max = x_max;
    this->x_min = x_min;
    this->y_max = y_max;
    this->y_min = y_min;
    this->z = z;
    this->point_counter = point_counter;
  }

};

bool obstacleComp ( Obstacle i, Obstacle j )
{
	if ( i.z == j.z)
	{
		return ( i.x_max < j.x_max );
	}
  return ( i.z < j.z );
}

std::vector<std::string> string_buffer_total;
std::vector<std::string> string_start_1_buffer;
std::vector<std::string> string_middle_1_buffer;
std::vector<std::string> string_end_1_buffer;
std::vector<std::string> string_start_2_buffer;
std::vector<std::string> string_middle_2_buffer;
std::vector<std::string> string_end_2_buffer;
std::vector<std::string> string_down_1_buffer;
std::vector<std::string> string_down_2_buffer;
int cmd_counter = 1;
void avoid_beginning_obstacle ( float x_start, float x_min, float y_tmp, float z_tmp, float deg_tmp )
{
	std::string x_start_s = std::to_string( x_start );
	std::string x_min_s = std::to_string( x_min );
	std::string y_tmp_s = std::to_string( y_tmp );
	std::string z_tmp_s = std::to_string( z_tmp );
	std::string deg_tmp_s = std::to_string( deg_tmp );
	std::string str_tmp_1 = std::to_string(cmd_counter) + " " + x_start_s + " " + std::to_string(  y_tmp + 0.04 ) + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
	cmd_counter++;
	std::string str_tmp_2 = std::to_string(cmd_counter) + " " + x_min_s + " " + std::to_string(  y_tmp + 0.04 ) + " " + z_tmp_s + " " + deg_tmp_s + "\n";
	cmd_counter++;
	std::string str_tmp_3 = std::to_string(cmd_counter) + " " + x_min_s + " " + y_tmp_s + " " + z_tmp_s + " " + deg_tmp_s + "\n";
	cmd_counter++;
	if ( x_start == x_1 )
	{
		string_start_1_buffer.push_back ( str_tmp_1 );
		string_start_1_buffer.push_back ( str_tmp_2 );
		string_start_1_buffer.push_back ( str_tmp_3 );
		// std::cout << "xxx str_tmp_1 = " << str_tmp_1 << "xxx str_tmp_2 = " << str_tmp_2 << "xxx str_tmp_3 = " << str_tmp_3;
	}
	if ( x_start == x_3 )
	{
		string_start_2_buffer.push_back ( str_tmp_1 );
		string_start_2_buffer.push_back ( str_tmp_2 );
		string_start_2_buffer.push_back ( str_tmp_3 );
	}
}

void avoid_middle_obstacle ( float x_start, float x_max, float x_min, float y_tmp, float z_tmp, float deg_tmp )
{
	std::string x_start_s = std::to_string( x_start );
	std::string x_max_s = std::to_string( x_max );
	std::string x_min_s = std::to_string( x_min );
	std::string y_tmp_s = std::to_string( y_tmp );
	std::string z_tmp_s = std::to_string( z_tmp );
	std::string deg_tmp_s = std::to_string( deg_tmp );
	std::string str_tmp_1 = std::to_string(cmd_counter) + " " + x_max_s + " " + y_tmp_s + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
	cmd_counter++;
	std::string str_tmp_2 = std::to_string(cmd_counter) + " " + x_max_s + " " + std::to_string(  y_tmp + 0.04 ) + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
	cmd_counter++;
	std::string str_tmp_3 = std::to_string(cmd_counter) + " " + x_min_s + " " + std::to_string(  y_tmp + 0.04 ) + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
	cmd_counter++;
	std::string str_tmp_4 = std::to_string(cmd_counter) + " " + x_min_s + " " + y_tmp_s + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
	cmd_counter++;
	if ( x_start == x_1 )
	{
		string_middle_1_buffer.push_back ( str_tmp_1 );
		string_middle_1_buffer.push_back ( str_tmp_2 );
		string_middle_1_buffer.push_back ( str_tmp_3 );
		string_middle_1_buffer.push_back ( str_tmp_4 );
	}
	if ( x_start == x_3 )
	{
		string_middle_2_buffer.push_back ( str_tmp_1 );
		string_middle_2_buffer.push_back ( str_tmp_2 );
		string_middle_2_buffer.push_back ( str_tmp_3 );
		string_middle_2_buffer.push_back ( str_tmp_4 );
	}
}

void avoid_ending_obstacle ( float x_start, float x_max, float x_end, float y_tmp, float z_tmp, float deg_tmp )
{
	std::string x_start_s = std::to_string( x_start );
	std::string x_max_s = std::to_string( x_max );
	std::string x_end_s = std::to_string( x_end );
	std::string y_tmp_s = std::to_string( y_tmp );
	std::string z_tmp_s = std::to_string( z_tmp );
	std::string deg_tmp_s = std::to_string( deg_tmp );
	std::string str_tmp_1 = std::to_string(cmd_counter) + " " + x_max_s + " " + y_tmp_s + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
	cmd_counter++;
	std::string str_tmp_2 = std::to_string(cmd_counter) + " " + x_max_s + " " + std::to_string( y_tmp + 0.04 ) + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
	cmd_counter++;
	std::string str_tmp_3 = std::to_string(cmd_counter) + " " + x_end_s + " " + std::to_string( y_tmp + 0.04 ) + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
	cmd_counter++;
	if ( x_start == x_1 )
	{
		string_end_1_buffer.push_back ( str_tmp_1 );
		string_end_1_buffer.push_back ( str_tmp_2 );
		string_end_1_buffer.push_back ( str_tmp_3 );
		str_tmp_1 = std::to_string(cmd_counter) + " " + std::to_string(x_2) + " " + std::to_string(y_tmp + y_back) + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
		cmd_counter++;
		str_tmp_2 = std::to_string(cmd_counter) + " " + std::to_string(x_3) + " " + std::to_string(y_tmp + y_back) + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
		cmd_counter++;
		string_end_1_buffer.push_back( str_tmp_1 );
		string_end_1_buffer.push_back( str_tmp_2 );
	}
	if ( x_start == x_3 )
	{
		string_end_2_buffer.push_back ( str_tmp_1 );
		string_end_2_buffer.push_back ( str_tmp_2 );
		string_end_2_buffer.push_back ( str_tmp_3 );
	}
}

void write_string_buffer_total( ofstream &avoid_obstacle_planner_fs, float z_tmp )
{
	std::cout << "string_buffer_total.size() = " << string_buffer_total.size() << std::endl;
	if ( z_tmp == z_2 )
	{
		// std::cout << "Print line path 2" << std::endl;
		while ( !string_buffer_total.empty() )
		{
			// std::cout << "### = "<< string_buffer_total.back();
			avoid_obstacle_planner_fs << string_buffer_total.back();
			string_buffer_total.pop_back();
		}
	} else
	{
		for (int idx = 0; idx < string_buffer_total.size(); idx++)
		{
			avoid_obstacle_planner_fs << string_buffer_total[idx];
		}
	}
}

// write_cfg_file ( avoid_obstacle_planner_fs, obstacle_vector_1, y_1, z_1, deg_1 );
void write_cfg_file ( ofstream &avoid_obstacle_planner_fs, std::vector< Obstacle > &obstacle_vector_tmp, float y_tmp, float z_tmp, float deg_tmp )
{
	bool is_avoid_x1 = false;
	bool is_avoid_x2 = false;
	bool is_avoid_x3 = false;
	bool is_avoid_x4 = false;
	string_start_1_buffer.clear();
	string_start_2_buffer.clear();
	string_middle_1_buffer.clear();
	string_middle_2_buffer.clear();
	string_end_1_buffer.clear();
	string_end_2_buffer.clear();
	if ( obstacle_vector_tmp.size() > 0 )
	{
		while ( !obstacle_vector_tmp.empty() )
		{
			Obstacle obstacle_tmp = obstacle_vector_tmp.back();
			obstacle_vector_tmp.pop_back();
			float x_max = obstacle_tmp.x_max;
			float x_min = obstacle_tmp.x_min;
			if ( x_max > x_1 && x_min < x_1 )
			{
				avoid_beginning_obstacle ( x_1, x_min, y_tmp, z_tmp, deg_tmp );
				is_avoid_x1 = true;
			}
			else if ( x_max < x_1 && x_min > x_2 )
			{
				avoid_middle_obstacle ( x_1, x_max, x_min, y_tmp, z_tmp, deg_tmp );
			}
			else if ( x_max > x_2 && x_min < x_2 )
			{
				avoid_ending_obstacle ( x_1, x_max, x_2, y_tmp, z_tmp, deg_tmp );
				is_avoid_x2 = true;
			}
			else if ( x_max > x_3 && x_min < x_3 )
			{
				avoid_beginning_obstacle ( x_3, x_min, y_tmp, z_tmp, deg_tmp );
				is_avoid_x3 = true;
			}
			else if ( x_max < x_3 && x_min > x_4 )
			{
				avoid_middle_obstacle ( x_3, x_max, x_min, y_tmp, z_tmp, deg_tmp );
			}
			else if ( x_max > x_4 && x_min < x_4 )
			{
				avoid_ending_obstacle ( x_3, x_max, x_4, y_tmp, z_tmp, deg_tmp );
				is_avoid_x4 = true;
			}
		}
	}
	std::cout << "is_avoid_x1 = " << is_avoid_x1	<< " is_avoid_x2 = " <<	is_avoid_x2 << " is_avoid_x3 = " << is_avoid_x3 << " is_avoid_x4 = " << is_avoid_x4 << std::endl;

	// output avoid obstacle path plan
	string_buffer_total.clear();
	std::string x_1_s = std::to_string( x_1 );
	std::string x_2_s = std::to_string( x_2 );
	std::string x_3_s = std::to_string( x_3 );
	std::string x_4_s = std::to_string( x_4 );
	std::string y_tmp_s = std::to_string( y_tmp );
	std::string z_tmp_s = std::to_string( z_tmp );
	std::string deg_tmp_s = std::to_string( deg_tmp );
	if ( !is_avoid_x1 )
	{
		std::string str_tmp = std::to_string(cmd_counter) + " " + x_1_s + " " + y_tmp_s + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
		cmd_counter++;
		string_buffer_total.push_back( str_tmp );
	} else
	{
		string_buffer_total.insert ( std::end ( string_buffer_total ), std::begin (string_start_1_buffer), std::end (string_start_1_buffer) );
	}
	if ( !string_middle_1_buffer.empty() )
	{
		string_buffer_total.insert ( std::end ( string_buffer_total ), std::begin (string_middle_1_buffer), std::end (string_middle_1_buffer) );
	}
	if ( !is_avoid_x2 )
	{
		std::string str_tmp_1 = std::to_string(cmd_counter) + " " + x_2_s + " " + y_tmp_s + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
		cmd_counter++;
		std::string str_tmp_2 = std::to_string(cmd_counter) + " " + x_2_s + " " + std::to_string(y_tmp + y_back) + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
		cmd_counter++;
		std::string str_tmp_3 = std::to_string(cmd_counter) + " " + x_3_s + " " + std::to_string(y_tmp + y_back) + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
		cmd_counter++;
		string_buffer_total.push_back( str_tmp_1 );
		string_buffer_total.push_back( str_tmp_2 );
		string_buffer_total.push_back( str_tmp_3 );
	} else
	{
		string_buffer_total.insert ( std::end ( string_buffer_total ), std::begin (string_end_1_buffer), std::end (string_end_1_buffer) );
	}
	if ( !is_avoid_x3 )
	{
		std::string str_tmp = std::to_string(cmd_counter) + " " + x_3_s + " " + y_tmp_s + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
		cmd_counter++;
		string_buffer_total.push_back( str_tmp );
	} else
	{
		string_buffer_total.insert ( std::end ( string_buffer_total ), std::begin (string_start_2_buffer), std::end (string_start_2_buffer) );
	}
	if ( !string_middle_2_buffer.empty() )
	{
		string_buffer_total.insert ( std::end ( string_buffer_total ), std::begin (string_middle_2_buffer), std::end (string_middle_2_buffer) );
	}
	if ( !is_avoid_x4 )
	{
		std::string str_tmp = std::to_string(cmd_counter) + " " + x_4_s + " " + y_tmp_s + " " +  z_tmp_s + " " + deg_tmp_s + "\n";
		cmd_counter++;
		string_buffer_total.push_back( str_tmp );
	} else
	{
		string_buffer_total.insert ( std::end ( string_buffer_total ), std::begin (string_end_2_buffer), std::end (string_end_2_buffer) );
	}

	// write path plan to configuration file
	write_string_buffer_total ( avoid_obstacle_planner_fs, z_tmp );
}

void write_string_down_1_buffer ( ofstream &avoid_obstacle_planner_fs )
{
	std::cout << "string_down_1_buffer.size() = " << string_down_1_buffer.size() << std::endl;
	for (int idx = 0; idx < string_down_1_buffer.size(); idx++)
	{
		avoid_obstacle_planner_fs << string_down_1_buffer[idx];
		if ( idx == 3 )
		{
			break;
		}
	}
}

void write_string_down_2_buffer ( ofstream &avoid_obstacle_planner_fs )
{
	std::cout << "string_down_2_buffer.size() = " << string_down_2_buffer.size() << std::endl;
	for (int idx = 0; idx < string_down_2_buffer.size(); idx++)
	{
		avoid_obstacle_planner_fs << string_down_2_buffer[idx];
		if ( idx == 3 )
		{
			break;
		}
	}
}

class Avoid_obstacle_planner
{
public:
	// function for finding planars
	int find_path ( PointCloudT::Ptr& cloud_in )
	{
		// step 1, filter, downsampling the input point cloud
		PointCloudT::Ptr cloud_filtered	( new PointCloudT );
		PointCloudT::Ptr cloud_cut ( new PointCloudT );
	  filterOutliner ( cloud_in );
		downSampling ( cloud_in, cloud_filtered );

		// step 2, cut the point cloud
		float x_tmp, y_tmp, z_tmp;
		int cloud_cut_counter = 0;
		for ( size_t i = 0; i < cloud_filtered->points.size (); ++i )
	  {
	    // Check if the point is invalid
	    if ( !pcl_isfinite ( cloud_filtered->points[ i ].x ) ||
	         !pcl_isfinite ( cloud_filtered->points[ i ].y ) ||
	         !pcl_isfinite ( cloud_filtered->points[ i ].z ) )
	        continue;
			x_tmp = cloud_filtered->points[ i ].x;
			y_tmp = cloud_filtered->points[ i ].y;
			z_tmp = cloud_filtered->points[ i ].z;
			// x 0.3 0.12 -0.05 -0.15
			// y -1.34  ( -0.13); -1.46 ( -0.13); -1.57 ( -0.13)
			// z  2.02  (+-0.02);  1.91 (+-0.02);  1.80 (+-0.02)
			if ( ( x_tmp > (x_2 - 0.04) && x_tmp < (x_1 + 0.10) ) || (  x_tmp > (x_4 - 0.08) && x_tmp < (x_3 + 0.025) ) )
			{
				if ( z_tmp > ( z_1 - z_offset*2.5 )  && z_tmp < ( z_1 - z_offset * 0.5 ) )
				{
					if ( y_tmp > ( y_1 - y_offset * ( 0.0 ) ) )
					{
						PointT new_point;
						new_point.x = x_tmp;
						new_point.y = y_tmp;
						new_point.z = z_tmp;
						uint8_t r = 255, g = 0, b = 0;
						uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
						new_point.rgb = *reinterpret_cast<float*>( &rgb );
						cloud_cut->points.push_back( new_point );
						cloud_cut_counter ++;
					}
				}
				else if ( z_tmp > ( z_2 - z_offset*2.5 )  && z_tmp < ( z_2 - z_offset * 0.5 ) )
				{
					if ( y_tmp > ( y_2 - y_offset * ( 0.29 ) ) )
					{
						PointT new_point;
						new_point.x = x_tmp;
						new_point.y = y_tmp;
						new_point.z = z_tmp;
						uint8_t r = 0, g = 255, b = 0;
						uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
						new_point.rgb = *reinterpret_cast<float*>( &rgb );
						cloud_cut->points.push_back( new_point );
						cloud_cut_counter ++;
					}
				}
				else if ( z_tmp > ( z_3 - z_offset*2.5 )  && z_tmp < ( z_3 - z_offset * 0.5 ) )
				{
					if ( y_tmp > ( y_3 - y_offset * ( 0.29 ) ) )
					{
						PointT new_point;
						new_point.x = x_tmp;
						new_point.y = y_tmp;
						new_point.z = z_tmp;
						uint8_t r = 0, g = 0, b = 255;
						uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
						new_point.rgb = *reinterpret_cast<float*>( &rgb );
						cloud_cut->points.push_back( new_point );
						cloud_cut_counter ++;
					}
				}
			}
			if ( x_tmp > ( x_4 - 0.015 ) && x_tmp < ( x_4 + 0.055 ) && z_tmp < ( z_1 - 0.030 ) && z_tmp > ( z_2 + 0.015 ) )
			{
				float k = ( y_1 - y_2 ) / ( z_1 - z_2 );
				float k_2 = ( y_1 - y_tmp ) / ( z_1 - z_tmp );
				if ( k_2 < k )
				{
					PointT new_point;
					new_point.x = x_tmp;
					new_point.y = y_tmp;
					new_point.z = z_tmp;
					uint8_t r = 255, g = 255, b = 0;
					uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
					new_point.rgb = *reinterpret_cast<float*>( &rgb );
					cloud_cut->points.push_back( new_point );
					cloud_cut_counter ++;
				}
			}
			if ( x_tmp > (x_1 - 0.015) && x_tmp < (x_1 + 0.055) && z_tmp < (z_2 - 0.030 ) && z_tmp > ( z_3 + 0.015 ) )
			{
				float k = ( y_2 - y_3 ) / ( z_2 - z_3 );
				float k_2 = ( y_2 - y_tmp ) / ( z_2 - z_tmp );
				if ( k_2 < k )
				{
					PointT new_point;
					new_point.x = x_tmp;
					new_point.y = y_tmp;
					new_point.z = z_tmp;
					uint8_t r = 0, g = 255, b = 255;
					uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
					new_point.rgb = *reinterpret_cast<float*>( &rgb );
					cloud_cut->points.push_back( new_point );
					cloud_cut_counter ++;
				}
			}
	  }

		// step 3, partition the cut off point cloud to seperate obstacles
		pcl::KdTreeFLANN < PointT > kdtree;
	  kdtree.setInputCloud ( cloud_cut );
		UF cloud_obstacle_uf ( cloud_cut_counter );
		for ( size_t i = 0; i < cloud_cut->points.size (); ++i )
		{
			PointT searchPoint = cloud_cut->points[i];
			// search for points within 2 cm
			std::vector<int> pointIdx;
			std::vector<float> pointRadius;
			if ( kdtree.radiusSearch ( searchPoint, 0.01, pointIdx, pointRadius ) > 0 )
			{
				for ( size_t j = 0; j < pointIdx.size (); ++j )
				{
					cloud_obstacle_uf.merge ( i, pointIdx[j] );
				}
			}
		}
		int obstacle_counter = cloud_obstacle_uf.count();
		std::cout << "obstacle_counter = " << obstacle_counter << std::endl;
		// step 3.1, genrate UF_idx_map
		std::map< int, int > UF_idx_map;
		std::map< int, int >::iterator iter;
		for ( size_t i = 0; i < cloud_cut->points.size (); ++i )
		{
			int UF_idx_tmp = cloud_obstacle_uf.find ( i );
			iter = UF_idx_map.find ( UF_idx_tmp );
			if ( iter == UF_idx_map.end() )
			{
				UF_idx_map.insert( std::pair<int, int> ( UF_idx_tmp, UF_idx_map.size() ) );
			}
		}
		// step 3.2, show the UF_idx_map
		std::cout << "UF_idx_map = " << std::endl;
		for ( iter = UF_idx_map.begin(); iter != UF_idx_map.end(); iter++ )
		{
			std::cout << iter->first << ":" << iter->second << std::endl;
		}
		// step 3.3, get obstacle_point_cloud_vector
		std::vector< PointCloudT::Ptr > obstacle_pc_vector;
		for ( int idx = 0; idx < UF_idx_map.size(); idx++ )
		{
			PointCloudT::Ptr cloud_tmp	( new PointCloudT );
			obstacle_pc_vector.push_back ( cloud_tmp );
		}
		// step 3.4, partition point cloud
		for ( size_t i = 0; i < cloud_cut->points.size (); ++i )
	  {
			int UF_idx_tmp = cloud_obstacle_uf.find ( i );
			iter = UF_idx_map.find ( UF_idx_tmp );
			x_tmp = cloud_cut->points[ i ].x;
			y_tmp = cloud_cut->points[ i ].y;
			z_tmp = cloud_cut->points[ i ].z;
			PointT new_point;
			new_point.x = x_tmp;
			new_point.y = y_tmp;
			new_point.z = z_tmp;
			uint8_t r = 255, g = 0, b = 0;
			r = iter->second % 5 * 50;
			g = iter->second % 3 * 80;
			b = iter->second % 7 * 30;
			if ( r == 0 && g == 0 && b == 0 )
			{
				r = 255;
				g = 125;
				b = 55;
			}
			uint32_t rgb = ( static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b) );
			new_point.rgb = *reinterpret_cast<float*>( &rgb );
			obstacle_pc_vector[iter->second]->points.push_back( new_point );
		}

		// step 4, process the point cloud list to get max_x, min_x, max_y, averge_z, point_num;
		PointCloudT::Ptr obstacle_cloud_total	( new PointCloudT );
		PointCloudT::Ptr obstacle_cloud_tmp	( new PointCloudT );
		std::vector< Obstacle > obstacle_vector;
		std::vector< Obstacle > obstacle_vector_1;
		std::vector< Obstacle > obstacle_vector_2;
		std::vector< Obstacle > obstacle_vector_3;
		std::vector< Obstacle > obstacle_vector_4;
		std::vector< Obstacle > obstacle_vector_5;
		float y_start_1 = 0;
		float z_start_1 = 0;
		float y_end_1 = 0;
		float z_end_1 = 10;
		bool is_down_1 = false;
		float y_start_2 = 0;
		float z_start_2 = 0;
		float y_end_2 = 0;
		float z_end_2 = 10;
		bool is_down_2 = false;
		for ( int idx = 0; idx < obstacle_pc_vector.size(); idx++ )
		{
			PointCloudT::Ptr cloud_tmp	= obstacle_pc_vector[idx];
			pcl::PointXYZRGB minPoint, maxPoint;
		  getMinMax3D ( *cloud_tmp, minPoint, maxPoint );
			std::cout << "obstacle cloud [" << idx << "] has [" << cloud_tmp->points.size () << "] points" << std::endl;
			std::cout << "***minPoint = " << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << std::endl;
			std::cout << "***maxPoint = " << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << std::endl;
			float z_tmp = ( maxPoint.z + minPoint.z ) / 2.0;
			float distance = maxPoint.x - minPoint.x;
			std::cout << "###Distance = " << distance << std::endl;
			*obstacle_cloud_tmp += *obstacle_pc_vector[idx];
			if ( distance > 0.10 || distance < 0.0035 || (maxPoint.y - minPoint.y) < 0.005 || (cloud_tmp->points.size () < 20) )
			{
				continue;
			}
			*obstacle_cloud_total += *obstacle_pc_vector[idx];
			float x_max_tmp, x_min_tmp;
			if ( minPoint.x > 0.0 )
			{
				x_min_tmp = minPoint.x - 0.04;
				x_max_tmp = x_min_tmp + 0.095;
			}
			if ( maxPoint.x < 0.0 )
			{
				x_max_tmp = maxPoint.x + 0.02;
				x_min_tmp = x_max_tmp - 0.11;
			}
			if ( std::abs( z_tmp - z_1 ) < 0.03 )
			{
				z_tmp = z_1;
				Obstacle obstacle_tmp_1 ( x_max_tmp, x_min_tmp, maxPoint.y, minPoint.y, z_tmp, cloud_tmp->points.size () );
		    obstacle_vector_1.push_back ( obstacle_tmp_1 );
			} else if ( std::abs( z_tmp - z_2 ) < 0.03 )
			{
				z_tmp = z_2;
				Obstacle obstacle_tmp_2 ( x_max_tmp, x_min_tmp, maxPoint.y, minPoint.y, z_tmp, cloud_tmp->points.size () );
		    obstacle_vector_2.push_back ( obstacle_tmp_2 );
			} else if ( std::abs( z_tmp - z_3 ) < 0.03 )
			{
				z_tmp = z_3;
				Obstacle obstacle_tmp_3 ( x_max_tmp, x_min_tmp, maxPoint.y, minPoint.y, z_tmp, cloud_tmp->points.size () );
		    obstacle_vector_3.push_back ( obstacle_tmp_3 );
			}
			if ( z_tmp < z_1 - 0.03 && z_tmp > z_2 + 0.015 )
			{
				is_down_1 = true;
				float k = ( y_1 - y_2 ) / ( z_1 - z_2 );
				float z_start = maxPoint.z + 0.022;
				if ( z_start >= z_1 - 0.005 )
				{
					z_start = z_1 - 0.005;
				}
				float y_start = y_1 - k * ( z_1 - z_start );
				float z_end = minPoint.z - 0.015;
				float y_end = y_1 - k * ( z_1 - z_end );

				if ( z_start > z_start_1 )
				{
					y_start_1 = y_start;
					z_start_1 = z_start;
				}
				if ( z_end < z_end_1 )
				{
					y_end_1 = y_end;
					z_end_1 = z_end;
				}
			}
			if ( z_tmp < z_2 - 0.03 && z_tmp > z_3 + 0.015 )
			{
				is_down_2 = true;
				float k = ( y_2 - y_3 ) / ( z_2 - z_3 );
				float z_start = maxPoint.z + 0.022;
				if ( z_start >= z_2 - 0.005 )
				{
					z_start = z_2 - 0.005;
				}
				float y_start = y_2 - k * ( z_2 - z_start );
				float z_end = minPoint.z - 0.015;
				float y_end = y_2 - k * ( z_2 - z_end );

				if ( z_start > z_start_2 )
				{
					y_start_2 = y_start;
					z_start_2 = z_start;
				}
				if ( z_end < z_end_2 )
				{
					y_end_2 = y_end;
					z_end_2 = z_end;
				}
			}
			std::cout << "***z_tmp = " << z_tmp << std::endl;
			Obstacle obstacle_tmp ( x_max_tmp, x_min_tmp, maxPoint.y, minPoint.y, z_tmp, cloud_tmp->points.size () );
	    obstacle_vector.push_back ( obstacle_tmp );
		}
		if ( is_down_1 ) {
			std::string x_4_s = std::to_string( x_4 );
			std::string z_start_s = std::to_string( z_start_1 );
			std::string y_start_s = std::to_string( y_start_1 );
			std::string z_end_s = std::to_string( z_end_1 );
			std::string y_end_s = std::to_string( y_end_1 );
			std::string deg_tmp_s = std::to_string( deg_2 );
			std::string str_tmp_1 = std::to_string(cmd_counter) + " " + x_4_s + " " + y_start_s + " " +  z_start_s + " " + deg_tmp_s + "\n";
			cmd_counter++;
			std::string str_tmp_2 = std::to_string(cmd_counter) + " " + x_4_s + " " + std::to_string(  y_start_1 + 0.06 ) + " " +  z_start_s + " " + deg_tmp_s + "\n";
			cmd_counter++;
			std::string str_tmp_3 = std::to_string(cmd_counter) + " " + x_4_s + " " + std::to_string(  y_end_1 + 0.06 ) + " " +  z_end_s + " " + deg_tmp_s + "\n";
			cmd_counter++;
			std::string str_tmp_4 = std::to_string(cmd_counter) + " " + x_4_s + " " + y_end_s + " " +  z_end_s + " " + deg_tmp_s + "\n";
			cmd_counter++;
			string_down_1_buffer.push_back ( str_tmp_1 );
			string_down_1_buffer.push_back ( str_tmp_2 );
			string_down_1_buffer.push_back ( str_tmp_3 );
			string_down_1_buffer.push_back ( str_tmp_4 );
		}
		if ( is_down_2 ) {
			std::string x_1_s = std::to_string( x_1 );
			std::string z_start_s = std::to_string( z_start_2 );
			std::string y_start_s = std::to_string( y_start_2 );
			std::string z_end_s = std::to_string( z_end_2 );
			std::string y_end_s = std::to_string( y_end_2 );
			std::string deg_tmp_s = std::to_string( deg_3 );
			std::string str_tmp_1 = std::to_string(cmd_counter) + " " + x_1_s + " " + y_start_s + " " +  z_start_s + " " + deg_tmp_s + "\n";
			cmd_counter++;
			std::string str_tmp_2 = std::to_string(cmd_counter) + " " + x_1_s + " " + std::to_string(  y_start_2 + 0.06 ) + " " +  z_start_s + " " + deg_tmp_s + "\n";
			cmd_counter++;
			std::string str_tmp_3 = std::to_string(cmd_counter) + " " + x_1_s + " " + std::to_string(  y_end_2 + 0.06 ) + " " +  z_end_s + " " + deg_tmp_s + "\n";
			cmd_counter++;
			std::string str_tmp_4 = std::to_string(cmd_counter) + " " + x_1_s + " " + y_end_s + " " +  z_end_s + " " + deg_tmp_s + "\n";
			cmd_counter++;
			string_down_2_buffer.push_back ( str_tmp_1 );
			string_down_2_buffer.push_back ( str_tmp_2 );
			string_down_2_buffer.push_back ( str_tmp_3 );
			string_down_2_buffer.push_back ( str_tmp_4 );
		}
		std::sort ( obstacle_vector_1.begin(), obstacle_vector_1.end(), obstacleComp );
		std::sort ( obstacle_vector_2.begin(), obstacle_vector_2.end(), obstacleComp );
		std::cout << obstacle_vector_3.size() << std::endl;
		std::sort ( obstacle_vector_3.begin(), obstacle_vector_3.end(), obstacleComp );

		// step 5, generate motion control points
		ofstream avoid_obstacle_planner_fs;
	  std::string cfgFileName = ros::package::getPath ( "motion_control" ) + "/config/avoid_obstacle.cfg";
	  avoid_obstacle_planner_fs.open ( cfgFileName );
		std::cout << obstacle_vector_1.size() << std::endl;
		write_cfg_file ( avoid_obstacle_planner_fs, obstacle_vector_1, y_1, z_1, deg_1 );
		write_string_down_1_buffer ( avoid_obstacle_planner_fs );
		std::cout << obstacle_vector_2.size() << std::endl;
		write_cfg_file ( avoid_obstacle_planner_fs, obstacle_vector_2, y_2, z_2, deg_2 );
		 write_string_down_2_buffer ( avoid_obstacle_planner_fs );
		std::cout << obstacle_vector_3.size() << std::endl;
		write_cfg_file ( avoid_obstacle_planner_fs, obstacle_vector_3, y_3, z_3, deg_3 );
		avoid_obstacle_planner_fs.close();

		// step 6, show the point cloud
		PointCloudT::Ptr scene_cloud_total	( new PointCloudT );
		// *scene_cloud_total += *cloud_filtered;
		// *scene_cloud_total += *cloud_cut;
		*scene_cloud_total += *obstacle_cloud_total;
		scene_cloud_total->header.frame_id = "world";
		cloud_pub_.publish ( scene_cloud_total );
		// Visualize ( cloud_filtered, obstacle_pc_vector[0] );
	}

  void handle_scene_point_cloud ( )
  {
    // load saved scene point cloud
    // std::string SceneFilePath = ros::package::getPath ( "object_localizer" ) + "/data/" + SceneFileName;
    // if ( pcl::io::loadPLYFile ( SceneFilePath, *scene_cloud_ ) == -1 )
    // {
    //   ROS_ERROR_STREAM ( "Couldn't read file: " << SceneFilePath << std::endl );
    //   return;
    // }
    std::cout << "The saved point cloud has [" << scene_cloud_->width * scene_cloud_->height << "] data points from " << std::endl;
    find_path ( scene_cloud_ );
  }

	bool start_avoid_obstacle_planner ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    handle_scene_point_cloud ();
    return true;
  }

	void cloud_cb ( const sensor_msgs::PointCloud2::ConstPtr& cloud )
  {
    // if input cloud is empty, return
    if ( ( cloud->width * cloud->height ) == 0 )
      return;
    // convert input ros cloud to pcl cloud and save it in local variable scene_cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL( *cloud, pcl_pc2 );
    pcl::fromPCLPointCloud2( pcl_pc2, *scene_cloud_ );
  }

  Avoid_obstacle_planner () : scene_cloud_ ( new pcl::PointCloud< PointT > )
	{
		start_avoid_obstacle_planner_ = nh_.advertiseService ( "start_avoid_obstacle_planner", &Avoid_obstacle_planner::start_avoid_obstacle_planner, this );

		std::string cloud_in_name = "/point_cloud_merger/points";
    cloud_sub_ = nh_.subscribe ( cloud_in_name, 1, &Avoid_obstacle_planner::cloud_cb, this );
    ROS_INFO_STREAM ( "Listening for point cloud on topic: " << cloud_in_name );

		std::string cloud_topic_out_ = "/avoid_obstacle/points";
    cloud_pub_ = nh_.advertise < PointCloudT > ( cloud_topic_out_, 30 );
    ROS_INFO_STREAM ( "Publishing point cloud message on topic " << cloud_topic_out_ );
	}

  ~Avoid_obstacle_planner () { }

private:
  ros::NodeHandle nh_;
  pcl::PointCloud<PointT>::Ptr scene_cloud_;
	ros::ServiceServer start_avoid_obstacle_planner_;
	ros::Subscriber cloud_sub_;
	ros::Publisher cloud_pub_;
};

void CfgFileReader ()
{
  std::string cfgFileName = ros::package::getPath ( "object_localizer" ) + "/config/avoid_obstacle_planner.cfg";
  std::cout << "***The path of the avoid_obstacle_planner configuration file is: [" << cfgFileName << "]" << std::endl;

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
		// +0.30 +0.12 -0.05 -0.15 # x_1 x_2 x_3 x_4
    iss >> x_1 >> x_2 >> x_3 >> x_4;
    std::cout << "***[x_1, x_2, x_3, x_4] = [" << x_1 << ", " << x_2 << ", " << x_3 << ", " << x_4 << "]" << std::endl;
  }
	if ( std::getline ( input, line ) )
  {
		std::istringstream iss ( line );
		// +0.02 -1.34 -1.46 -1.57 +0.13 # y_o y_1 y_2 y_3 y_b
    iss >> y_offset >> y_1 >> y_2 >> y_3 >> y_back;
    std::cout << "***[y_offset, y_1, y_2, y_3, y_back] = [" << y_offset << ", " << y_1 << ", " << y_2 << ", " << y_3 << ", " << y_back << "]" << std::endl;
  }
	if ( std::getline ( input, line ) )
  {
		std::istringstream iss ( line );
		// +0.02 +2.02 +1.91 +1.80 # z_b z_1 z_2 z_3
    iss >> z_offset >> z_1 >> z_2 >> z_3;
    std::cout << "***[z_offset, z_1, z_2, z_3] = [" << z_offset << ", " << z_1 << ", " << z_2 << ", " << z_3 << "]" << std::endl;
  }
	if ( std::getline ( input, line ) )
  {
		std::istringstream iss ( line );
		// 0.898 0.834 0.730
    iss >> deg_1 >> deg_2 >> deg_3;
    std::cout << "***[deg_1, deg_2, deg_3] = [" << deg_1 << ", " << deg_2 << ", " << deg_3 << "]" << std::endl;
  }
  input.close();
}

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "avoid_obstacle_planner" );
	ros::AsyncSpinner spinner ( 2 );
  spinner.start ();
	CfgFileReader ();
	Avoid_obstacle_planner AOP;
	ros::waitForShutdown ();
  return 0;
}
