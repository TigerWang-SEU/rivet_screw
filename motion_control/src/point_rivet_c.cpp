#include <math.h>
#include <cmath>
#include <queue>
#include <iostream>
#include <string>
#include <map>

#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

std::string reference_frame = "world";
ros::ServiceClient new_nut_, stop_new_nut_, start_screwing_, stop_screwing_;

class Adjustment
{
public:
  float h_adjust, v_adjust, tool_distance;
  Adjustment ();
  Adjustment ( float h_adjust, float v_adjust, float tool_distance );
  Adjustment& operator= ( const Adjustment& adjustment_in );
};

Adjustment::Adjustment () {}

Adjustment::Adjustment ( float h_adjust, float v_adjust, float tool_distance )
{
  this->h_adjust = h_adjust;
  this->v_adjust = v_adjust;
  this->tool_distance = tool_distance;
}

Adjustment& Adjustment::operator= ( const Adjustment& adjustment_in )
{
   h_adjust = adjustment_in.h_adjust;
   v_adjust = adjustment_in.v_adjust;
   tool_distance = adjustment_in.tool_distance;
}

double read_tool_angle ( std::map < std::string, Adjustment > & adjustment_map )
{
  std::string tool_angle_file = ros::package::getPath ( "motion_control" ) + "/config/tool_angle.cfg";
  std::cout << "*** Read tool_angle_file is: [" << tool_angle_file << "]" << std::endl;
  std::ifstream input ( tool_angle_file );

  std::string line;
  float tool_angle = 0.0, pitch_degree, h_adjust, v_adjust, tool_distance;
  adjustment_map.clear ();
  while ( std::getline ( input, line ) )
  {
    if ( line.rfind ( "#", 0 ) == 0 || line.length() == 0 )
    {
      continue;
    }
    std::istringstream iss ( line );
    iss >> tool_angle >> pitch_degree >> h_adjust >> v_adjust >> tool_distance;
    std::string adjustment_key = std::to_string ( tool_angle ) + "" + std::to_string ( pitch_degree );
    Adjustment adjustment ( h_adjust, v_adjust, tool_distance );
    adjustment_map [ adjustment_key ] = adjustment;
  }

  input.close();
  return tool_angle;
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

void getInverseMatrix ( Eigen::Matrix4f& original_transform, Eigen::Matrix4f& inverse_transform )
{
	inverse_transform.block< 3, 3 >( 0, 0 ) = original_transform.block< 3, 3 >( 0, 0 ).transpose();
  inverse_transform.block< 3, 1 >( 0, 3 ) = -1.0f * ( inverse_transform.block< 3, 3 >( 0, 0 ) * original_transform.block< 3, 1 >( 0, 3 ) );
}

void get_rpy_from_matrix ( Eigen::Matrix4f rotation_matrix, double& roll, double& pitch, double& yaw )
{
  tf::Matrix3x3 object_m ( rotation_matrix (0, 0), rotation_matrix (0, 1), rotation_matrix (0, 2), rotation_matrix (1, 0), rotation_matrix (1, 1), rotation_matrix (1, 2), rotation_matrix (2, 0), rotation_matrix (2, 1), rotation_matrix (2, 2) );
  object_m.getRPY ( roll, pitch, yaw );
}

void get_matrix_from_rpy ( Eigen::Matrix4f& rotation_matrix, double roll, double pitch, double yaw )
{
  rotation_matrix << cos(yaw)*cos(pitch),
                     cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll),  cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll), 0,
                     sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), 0,
                    -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), 0,
                              0,                    0,                    0, 1;
}

class Target
{
public:
  int id;
  double x, y, z, roll, pitch, yaw;
  Target ( int id_i, double x_i, double y_i, double z_i, double roll_i, double pitch_i, double yaw_i);
};

Target::Target ( int id_i, double x_i, double y_i, double z_i, double roll_i, double pitch_i, double yaw_i)
{
  id = id_i;
  x = x_i;
  y = y_i;
  z = z_i;
  roll = roll_i;
  pitch = pitch_i;
  yaw = yaw_i;
}

void targetFileReader ( std::queue< Target >& target_queue )
{
  std::map < std::string, Adjustment > adjustment_map;
  float tool_angle = read_tool_angle ( adjustment_map );
  Eigen::Matrix4f h_v_adjust, r_x_theta;
  // get a rotation matrix
  float theta_x = tool_angle / 180.0 * M_PI;
  get_matrix_from_rpy ( r_x_theta, theta_x, 0, 0 );

  std::string cfgFileName = ros::package::getPath ( "object_localizer" ) + "/config/point_rivet.cfg";
  std::cout << "***The path of the point_rivet configuration file is: [" << cfgFileName << "]" << std::endl;

  int id, current_id = -1;
  double x, y, z, roll, pitch, yaw;
  std::ifstream input ( cfgFileName );
  std::string line;
  while ( std::getline ( input, line ) )
  {
    std::istringstream iss ( line );
    iss >> id >> x >> y >> z >> roll >> pitch >> yaw;

    float pitch_degree = pitch / M_PI * 180.0;
    std::cout << "$$$ pitch_degree = " << pitch_degree << std::endl;
    float b = (int ) ( pitch_degree / 10 ) * 10 + 5;
    if ( pitch_degree < 0 )
    {
        b -= 10;
    }
    std::string adjustment_key = std::to_string ( tool_angle ) + "" + std::to_string ( b );
    if ( adjustment_map.find ( adjustment_key ) == adjustment_map.end () )
    {
      continue;
    }

    Adjustment adjustment = adjustment_map [ adjustment_key ];
    h_v_adjust << 1, 0, 0, adjustment.tool_distance,
                  0, 1, 0, adjustment.h_adjust,
                  0, 0, 1, -adjustment.v_adjust,
                  0, 0, 0, 1;

    Eigen::Matrix4f origin_trans;
    get_matrix_from_rpy ( origin_trans, roll, pitch, yaw );
    origin_trans ( 0, 3 ) = x;
    origin_trans ( 1, 3 ) = y;
    origin_trans ( 2, 3 ) = z;
    Eigen::Matrix4f total_transform =  origin_trans * h_v_adjust * r_x_theta;
    Eigen::Vector4f point_origin;
    point_origin << 0.0, 0.0, 0.0, 1.0;
    point_origin = total_transform * point_origin;
    x = point_origin ( 0 );
    y = point_origin ( 1 );
    z = point_origin ( 2 );
    get_rpy_from_matrix ( total_transform, roll, pitch, yaw );

    if ( current_id != id )
    {
      show_frame ( "rivet_" + std::to_string( id ), x, y, z, roll, pitch, yaw );
      current_id = id;
    }

    Target target ( id, x, y, z, roll, pitch, yaw );
    target_queue.push ( target );
    std::cout << id << ": [x, y, z, roll, pitch, yaw] = [" << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
  }
  input.close();
}

void move_trajectory ( geometry_msgs::Pose& target_pose1, geometry_msgs::Pose& target_pose2, moveit::planning_interface::MoveGroupInterface& move_group )
{
  std::vector < geometry_msgs::Pose> waypoints;
  waypoints.push_back ( target_pose1 );
  waypoints.push_back ( target_pose2 );

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath ( waypoints, eef_step, jump_threshold, trajectory );
  ROS_INFO_NAMED ( "point_rivet", "trajectory plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0 );

  if ( fraction > 0.98 )
  {
    // scale the velocity and acceleration of the trajectory
    const double scale_factor = 0.5;
    int point_size = trajectory.joint_trajectory.points.size();
    for ( int point_idx = 0; point_idx < point_size; point_idx++ )
    {
      trajectory_msgs::JointTrajectoryPoint point_tmp = trajectory.joint_trajectory.points[point_idx];
      int size_tmp = point_tmp.velocities.size();
      for ( int i = 0; i <= size_tmp; i++ )
      {
        float velocity_tmp = point_tmp.velocities[i];
        trajectory.joint_trajectory.points[point_idx].velocities[i] = velocity_tmp * scale_factor;
        float acceleration_tmp = point_tmp.accelerations[i];
        trajectory.joint_trajectory.points[point_idx].accelerations[i] = acceleration_tmp * scale_factor;
      }
      ros::Duration time_from_start_tmp = point_tmp.time_from_start;
      trajectory.joint_trajectory.points[point_idx].time_from_start.fromSec ( time_from_start_tmp.toSec() / scale_factor );
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;
    move_group.execute ( my_plan );
  }
}

void set_target_pose_1 ( Target& target, geometry_msgs::Pose& target_pose )
{
  target_pose.position.x = target.x + 0.04;
  target_pose.position.y = target.y + 0.05;
  target_pose.position.z = target.z - 0.05;
  float rollt  = target.roll;
  float pitcht = target.pitch;
  float yawt   = target.yaw;
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
}

void set_target_pose ( Target& target, geometry_msgs::Pose& target_pose )
{
  target_pose.position.x = target.x;
  target_pose.position.y = target.y;
  target_pose.position.z = target.z;
  float rollt  = target.roll;
  float pitcht = target.pitch;
  float yawt   = target.yaw;
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( rollt, pitcht, yawt );
}

void do_point_rivet ()
{
  // step 1, read in all target rivets
  std_srvs::Empty msg;
  std::queue< Target > target_queue;
  targetFileReader ( target_queue );

  // step 2, create interface for motion planning
  static const std::string PLANNING_GROUP = "rivet_tool";
  moveit::planning_interface::MoveGroupInterface move_group ( PLANNING_GROUP );
  ROS_INFO_NAMED( "point_rivet", "Reference frame: %s", move_group.getPlanningFrame ().c_str () );
  ROS_INFO_NAMED( "point_rivet", "End effector link: %s", move_group.getEndEffectorLink ().c_str () );

  // step 3, move to each rivet
  if ( !target_queue.empty () )
  {
    Target target = target_queue.front ();
    target_queue.pop ();
    geometry_msgs::Pose target_pose1;
    set_target_pose_1 ( target, target_pose1 );
    move_group.setPoseTarget ( target_pose1 );
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO_STREAM ( "Move to start pose:" );
    bool success = ( move_group.plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
    ROS_INFO_NAMED ( "point_rivet", "planning for goal pose is %s", success ? "success" : "FAILED" );

    if ( success )
    {
      move_group.setMaxVelocityScalingFactor ( 0.3 );
      move_group.setMaxAccelerationScalingFactor ( 0.3 );
      move_group.move ();
      ros::Duration ( 0.5 ) .sleep ();

      set_target_pose ( target, target_pose1 );
      move_group.setPoseTarget ( target_pose1 );
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      ROS_INFO_STREAM ( "Move to start pose:" );
      bool success = ( move_group.plan ( my_plan ) == moveit::planning_interface::MoveItErrorCode::SUCCESS );
      ROS_INFO_NAMED ( "point_rivet", "planning for goal pose is %s", success ? "success" : "FAILED" );

      if ( success )
      {
        move_group.setMaxVelocityScalingFactor ( 0.3 );
        move_group.setMaxAccelerationScalingFactor ( 0.3 );
        move_group.move ();
        ros::Duration ( 0.5 ) .sleep ();

        new_nut_.call ( msg );

        // start the screwing part
        while ( !target_queue.empty () )
        {
          // pump out a new nut
          // new_nut_.call ( msg );
          // ros::Duration ( 0.3 ) .sleep ();

          // get the in pose
          Target target = target_queue.front ();
          target_queue.pop ();
          geometry_msgs::Pose target_pose2;
          set_target_pose ( target, target_pose2 );

          // start screwing the rivet and move into the rivet
          start_screwing_.call ( msg );
          move_trajectory ( target_pose1, target_pose2, move_group );
          ros::Duration ( 4.5 ) .sleep ();
          stop_screwing_.call ( msg );
          target_pose1 = target_pose2;

          // move back the rivet_tool and stop pump the nut
          target = target_queue.front ();
          target_queue.pop ();
          set_target_pose ( target, target_pose2 );
          move_trajectory ( target_pose1, target_pose2, move_group );

          // pump out the broken part of a nut
          // stop_new_nut_.call ( msg );
          // for ( int pump_counter = 0; pump_counter < 2; pump_counter++ )
          // {
          //   ros::Duration ( 0.2 ) .sleep ();
          //   new_nut_.call ( msg );
          //   ros::Duration ( 0.3 ) .sleep ();
          //   stop_new_nut_.call ( msg );
          // }

          // move to the next rivet if there exist the next rivet
          target_pose1 = target_pose2;
          if ( !target_queue.empty () )
          {
            target = target_queue.front();
            target_queue.pop();
            set_target_pose ( target, target_pose2 );
            move_trajectory ( target_pose1, target_pose2, move_group );
            target_pose1 = target_pose2;
          }
        }
        stop_new_nut_.call ( msg );
      }
    }
  }
}

bool start_point_rivet ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  do_point_rivet ();
  return true;
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "point_rivet_c" );
  ros::NodeHandle nh_;

  new_nut_ = nh_.serviceClient < std_srvs::Empty > ( "new_nut" );
  stop_new_nut_ = nh_.serviceClient < std_srvs::Empty > ( "stop_new_nut" );
  start_screwing_ = nh_.serviceClient < std_srvs::Empty > ( "start_screwing" );
  stop_screwing_ = nh_.serviceClient < std_srvs::Empty > ( "stop_screwing" );

  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  ros::ServiceServer start_point_rivet_;
  start_point_rivet_ = nh_.advertiseService ( "start_point_rivet", &start_point_rivet );
  ros::waitForShutdown ();
  return 0;
}
