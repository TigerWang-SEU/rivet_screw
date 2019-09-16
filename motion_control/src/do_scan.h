#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <string>
#include <fstream>

#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

std::string scan_plan_file = ros::package::getPath ( "motion_control" ) + "/config/" + "scan_plan.cfg";
std::string scan_idx_file = ros::package::getPath ( "motion_control" ) + "/config/" + "scan_idx.cfg";

class ScanPlan
{
public:

  float rotation_deg, x_s, y_s, z_s, x_e, y_e, z_e, x_final, y_final, z_final;
  int color_r, color_g, color_b;

  ScanPlan ( float rotation_deg, float x_s, float y_s, float z_s, float x_e, float y_e, float z_e, float x_final, float y_final, float z_final, int color_r, int color_g, int color_b )
  {
    this->rotation_deg = rotation_deg;
    this->x_s = x_s;
    this->y_s = y_s;
    this->z_s = z_s;
    this->x_e = x_e;
    this->y_e = y_e;
    this->z_e = z_e;
    this->x_final = x_final;
    this->y_final = y_final;
    this->z_final = z_final;
    this->color_r = color_r;
    this->color_g = color_g;
    this->color_b = color_b;
  }

};

bool scanPlanComp ( ScanPlan i,ScanPlan j )
{
  return ( i.rotation_deg > j.rotation_deg );
}

void scan_plan_reader ( std::vector< ScanPlan >& scan_plan_vector )
{
  std::cout << "*** Read scan_plan file: [" << scan_plan_file << "]" << std::endl;
  std::ifstream input ( scan_plan_file );
  std::string line;
  double rotation_deg, x_s, y_s, z_s, x_e, y_e, z_e, x_final, y_final, z_final;
  int color_r, color_g, color_b;
  while ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> rotation_deg >> x_s >> y_s >> z_s >> x_e >> y_e >> z_e >> x_final >> y_final >> z_final >> color_r >> color_g >> color_b;
    ScanPlan scan_plan ( rotation_deg, x_s, y_s, z_s, x_e, y_e, z_e, x_final, y_final, z_final, color_r, color_g, color_b );
    scan_plan_vector.push_back ( scan_plan );
  }
  std::sort ( scan_plan_vector.begin(), scan_plan_vector.end(), scanPlanComp );
  input.close();
}

bool scan_plan_writer ( std::string write_string )
{
  std::cout << "*** write scan_plan file: [" << scan_plan_file << "]" << std::endl;
  ofstream scan_plan_fs;
  scan_plan_fs.open ( scan_plan_file );
  scan_plan_fs << write_string << std::endl;
  scan_plan_fs.close();
}

void read_idx ( int& scan_idx )
{
  std::cout << "** Read scan_idx file: [" << scan_idx_file << "]" << std::endl;
  std::ifstream input ( scan_idx_file );
  std::string line;
  int scan_idx = 1;
  if ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> scan_idx;
  }
  input.close();
}

bool read_idx_writer ( int scan_plan_idx )
{
  std::cout << "*** write read_idx file: [" << scan_idx_file << "]" << std::endl;
  ofstream scan_idx_fs;
  scan_idx_fs.open ( scan_idx_file );
  scan_idx_fs << scan_plan_idx << std::endl;
  scan_idx_fs.close();
}
