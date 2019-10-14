#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#ifndef PCL_NAME_H
#define PCL_NAME_H

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

#endif
