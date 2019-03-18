#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

#include <std_srvs/Empty.h>
#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <geometric_shapes/shape_messages.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

class AddAndRemoveObject {

  ros::NodeHandle nh_;
  tf2_ros::StaticTransformBroadcaster br;
  ros::ServiceClient planning_scene_diff_client;
  ros::ServiceServer add_aircraft_frame_, remove_aircraft_frame_;
  bool is_add = false;

public:

  AddAndRemoveObject ()
  {
    ROS_INFO ( "Starting add and remove aircraft frame service" );
    add_aircraft_frame_ = nh_.advertiseService ( "add_aircraft_frame", &AddAndRemoveObject::add_aircraft_frame, this );
    remove_aircraft_frame_ = nh_.advertiseService ( "remove_aircraft_frame", &AddAndRemoveObject::remove_aircraft_frame, this );
    planning_scene_diff_client = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
  }

  ~AddAndRemoveObject ()
  {}

  bool add_aircraft_frame ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    is_add = true;
    CfgFileReader ();
    return true;
  }

  bool remove_aircraft_frame ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    is_add = false;
    CfgFileReader ();
    return true;
  }

  void getObjectShapeAndPosition ( int object_num, moveit_msgs::CollisionObject& object, float x, float y, float z, float roll, float pitch, float yaw )
  {

  	shape_msgs::SolidPrimitive primitive;
  	geometry_msgs::Pose pose;

    switch( object_num )
    {
      case 0:
      {
        static const Eigen::Vector3d scale(0.001, 0.001, 0.001);
		    shapes::Mesh* m = shapes::createMeshFromResource("package://model_loader/model/M1-2-010-0-001-A_Jig-Aussenhaut-FAL.stl", scale);
		    shapes::ShapeMsg mesh_msg;
		    shapes::constructMsgFromShape(m, mesh_msg);
		    shape_msgs::Mesh mesh;
		    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
		    object.meshes.push_back(mesh);

		    pose.position.x = x;
		    pose.position.y = y;
		    pose.position.z = z;
		    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
		    object.mesh_poses.push_back(pose);
		    break;
	    }
	    case 1:
	    {
        static const Eigen::Vector3d scale(0.001, 0.001, 0.001);
		    shapes::Mesh* m = shapes::createMeshFromResource("package://model_loader/model/platte_mit_schrauben.stl", scale);
		    shapes::ShapeMsg mesh_msg;
		    shapes::constructMsgFromShape(m, mesh_msg);
		    shape_msgs::Mesh mesh;
		    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
		    object.meshes.push_back(mesh);

		    pose.position.x = x;
		    pose.position.y = y;
		    pose.position.z = z;
		    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
		    object.mesh_poses.push_back(pose);
		    break;
      }
    }
  }

  moveit_msgs::CollisionObject spawnObject ( std::string object_id, int object_num, float x, float y, float z, float roll, float pitch, float yaw )
  {
      moveit_msgs::ApplyPlanningScene srv;
      moveit_msgs::PlanningScene planning_scene;
      planning_scene.is_diff = true;
      planning_scene.robot_state.is_diff = true;

      moveit_msgs::CollisionObject object;
      object.id = object_id;
      object.header.frame_id = "world";
      getObjectShapeAndPosition(object_num, object, x, y, z, roll, pitch, yaw);

      // add object to scene
      object.operation = object.ADD;
      planning_scene.world.collision_objects.push_back(object);

      srv.request.scene = planning_scene;
      planning_scene_diff_client.call(srv);

      return object;
  }

  moveit_msgs::CollisionObject removeObject ( std::string object_id )
  {
      moveit_msgs::ApplyPlanningScene srv;
      moveit_msgs::PlanningScene planning_scene;
      planning_scene.is_diff = true;
      planning_scene.robot_state.is_diff = true;

      moveit_msgs::CollisionObject object;
      object.id = object_id;
      object.header.frame_id = "world";

      // remove objects from the scene
      object.operation = object.REMOVE;
      planning_scene.world.collision_objects.push_back(object);

      // remove attached object in case it is attached
      moveit_msgs::AttachedCollisionObject aco;
      object.operation = object.REMOVE;
      aco.object = object;
      planning_scene.robot_state.attached_collision_objects.push_back(aco);

      srv.request.scene = planning_scene;
      planning_scene_diff_client.call(srv);

      return object;
  }

  void CfgFileReader ()
  {
    std::string cfgFileName;
    cfgFileName = ros::package::getPath("model_loader") + "/config/aircraft_frame_setup.cfg";
    std::cout << "\tThe path of the configuration file is: [" << cfgFileName << "]" << std::endl;

    // read the configuration file
    std::ifstream input ( cfgFileName );
    std::string line;
    geometry_msgs::TransformStamped transformStamped;
    // read the first line in the configuration file
    if ( std::getline ( input, line ) )
    {
      std::istringstream iss ( line );
      std::string cmd;
      int objectNum;
      iss >> cmd >> objectNum;
      std::cout << "\t[" << cmd << "]\t[" << objectNum << "]\n";

      for ( int idx = 0; idx < objectNum; idx++ )
      {
        std::getline(input, line);
        std::istringstream iss(line);
        std::string objectID;
        int objectType;
        double x, y, z, roll, pitch, yaw, rollD, pitchD, yawD;
        iss >> objectID >> objectType >> x >> y >> z >> rollD >> pitchD >> yawD;
        roll = rollD * M_PI / 180.0; // rollD degrees around X
        pitch = pitchD * M_PI / 180.0; // pitchD degrees around Y
        yaw = yawD * M_PI / 180.0; // yawD degrees around Z
        std::cout << "\t[" << objectID << "]\t[" << objectType << "]\t[" << x << "]\t[" << y << "]\t[" << z
          << "]\t[" << roll << "]\t[" << pitch << "]\t[" << yaw << "]\n";
        if ( !is_add )
        {
          removeObject ( objectID );
        }
        else
        {
          spawnObject ( objectID, objectType, x, y, z, roll, pitch, yaw );

          // create a frame for each object
          transformStamped.header.stamp = ros::Time::now();
          transformStamped.header.frame_id = "world";
          transformStamped.child_frame_id = objectID;
          transformStamped.transform.translation.x = x;
          transformStamped.transform.translation.y = y;
          transformStamped.transform.translation.z = z;
          tf2::Quaternion q;
          q.setRPY ( roll, pitch, yaw );
          transformStamped.transform.rotation.x = q.x();
          transformStamped.transform.rotation.y = q.y();
          transformStamped.transform.rotation.z = q.z();
          transformStamped.transform.rotation.w = q.w();
          ros::Rate loop_rate ( 10 );
          for (int i = 0; i < 10; i++)
          {
            br.sendTransform ( transformStamped );
            loop_rate.sleep ();
          }
          std::cout << "\tFrame " << objectID << " is added\n";
        }
      }
    }
    input.close();
  }

};

int main ( int argc, char** argv )
{
  ros::init(argc, argv, "AAR_aircraft_frame");
  ros::AsyncSpinner spinner ( 2 );
  spinner.start ();
  AddAndRemoveObject AARObject;
  ros::waitForShutdown ();
  return 0;
}
