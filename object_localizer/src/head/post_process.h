#include "union_find.h"
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include <pcl/point_types.h>

#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>

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

#ifndef POST_PROCESS_H
#define POST_PROCESS_H

class Target
{
public:
  int id;
  double x, y, z, roll, pitch, yaw;
  Target ( int id_i, double x_i, double y_i, double z_i, double roll_i, double pitch_i, double yaw_i );
};

Target::Target ( int id_i, double x_i, double y_i, double z_i, double roll_i, double pitch_i, double yaw_i )
{
  id = id_i;
  x = x_i;
  y = y_i;
  z = z_i;
  roll = roll_i;
  pitch = pitch_i;
  yaw = yaw_i;
}

void targetFileReader ( pcl::PointCloud< pcl::PointXYZ >::Ptr& target_cloud, std::vector< Target >& target_vector )
{
  std::string cfgFileName = ros::package::getPath ( "object_localizer" ) + "/config/point_rivet.cfg";
  std::cout << "*** Read point_rivet file: [" << cfgFileName << "]" << std::endl;

  int id, current_id = -1;
  double x, y, z, roll, pitch, yaw;
  std::ifstream input ( cfgFileName );
  std::string line;
  while ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> id >> x >> y >> z >> roll >> pitch >> yaw;

    Target target ( id, x, y, z, roll, pitch, yaw );
    target_vector.push_back ( target );
    std::cout << id << ": [x, y, z, roll, pitch, yaw] = [" << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << "]" << std::endl;

    if ( current_id != id )
    {
      pcl::PointXYZ new_point;
			new_point.x = x;
			new_point.y = y;
			new_point.z = z;
			target_cloud->points.push_back ( new_point );
      current_id = id;
    }
  }
  input.close();
}

void target_post_process ()
{
  pcl::PointCloud < pcl::PointXYZ > ::Ptr target_cloud ( new pcl::PointCloud < pcl::PointXYZ > );
  std::vector< Target > target_vector;
  targetFileReader ( target_cloud, target_vector );

  pcl::KdTreeFLANN < pcl::PointXYZ > kdtree;
  kdtree.setInputCloud ( target_cloud );
  float rivet_radius = 0.040;
  UF target_cloud_uf ( target_cloud->points.size () );
  for ( size_t i = 0; i < target_cloud->points.size (); ++i )
  {
    pcl::PointXYZ searchPoint = target_cloud->points [ i ];
    std::vector < int > pointIdx;
    std::vector < float > pointRadius;
    if ( kdtree.radiusSearch ( searchPoint, rivet_radius, pointIdx, pointRadius ) > 0 )
    {
      for ( size_t j = 0; j < pointIdx.size (); ++j )
      {
        target_cloud_uf.merge ( i, pointIdx [ j ] );
      }
    }
  }
  std::map < int, std::list < int > > target_components = target_cloud_uf.get_components();

  ofstream point_rivet_fs;
  point_rivet_fs.open ( ros::package::getPath ( "object_localizer" ) + "/config/point_rivet_2.cfg" );
  int rivet_counter = 0;
  for ( auto const& x : target_components )
  {
    int component_id = x.first;
    std::list < int > component_element_list = x.second;

    for ( auto const& target_idx : component_element_list )
    {
      std::cout << target_idx << " ";
    }

    if ( component_element_list.size() > 6 )
    {
      for ( auto const& target_idx : component_element_list )
      {
        Target target_1 = target_vector [ target_idx * 3 ];
        Target target_2 = target_vector [ target_idx * 3 + 1 ];
        Target target_3 = target_vector [ target_idx * 3 + 2 ];
        point_rivet_fs << rivet_counter << " " << target_1.x << " " << target_1.y << " " << target_1.z << " " << target_1.roll << " " << target_1.pitch << " " << target_1.yaw << std::endl;
        point_rivet_fs << rivet_counter << " " << target_2.x << " " << target_2.y << " " << target_2.z << " " << target_2.roll << " " << target_2.pitch << " " << target_2.yaw << std::endl;
        point_rivet_fs << rivet_counter << " " << target_3.x << " " << target_3.y << " " << target_3.z << " " << target_3.roll << " " << target_3.pitch << " " << target_3.yaw << std::endl;
        rivet_counter ++;
      }
    }
    std::cout << std::endl;
  }
  point_rivet_fs.close();

}

#endif
