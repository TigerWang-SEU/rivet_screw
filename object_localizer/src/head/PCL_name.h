#ifndef PCL_NAME_H
#define PCL_NAME_H

#include <vector>
#include <map>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "union_find.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud< PointT > PointCloudT;

void getMinMax3D ( const pcl::PointCloud<PointT> &cloud, PointT &min_pt, PointT &max_pt )
{
  Eigen::Array4f min_p, max_p;
  min_p.setConstant ( FLT_MAX );
  max_p.setConstant ( -FLT_MAX );

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

// partition a point cloud to seperate part
void partition_pc ( PointCloudT::Ptr cloud_in, std::vector < PointCloudT::Ptr >& cloud_vector, float distance )
{
  pcl::KdTreeFLANN < PointT > kdtree;
  kdtree.setInputCloud ( cloud_in );
  UF cloud_in_uf ( cloud_in->points.size () );
  for ( size_t i = 0; i < cloud_in->points.size (); ++i )
  {
    PointT searchPoint = cloud_in->points [ i ];
    std::vector < int > p_idx;
    std::vector < float > p_rad;
    if ( kdtree.radiusSearch ( searchPoint, distance, p_idx, p_rad ) > 0 )
    {
      for ( size_t j = 0; j < p_idx.size (); ++j )
      {
        cloud_in_uf.merge ( i, p_idx [ j ] );
      }
    }
  }
  std::map < int, std::list < int > > comp_cloud_map = cloud_in_uf.get_components();

  for ( auto const& x : comp_cloud_map )
  {
    int comp_id = x.first;
    std::list < int > comp_element_list = x.second;

    PointCloudT::Ptr comp_cloud ( new PointCloudT );
    for ( auto const& point_idx : comp_element_list )
    {
      PointT current_point = cloud_in->points [ point_idx ];
      comp_cloud->points.push_back ( current_point );
    }
    cloud_vector.push_back ( comp_cloud );
  }

  std::cout << "cloud_vector.size() = " << cloud_vector.size() << std::endl;
}

#endif
