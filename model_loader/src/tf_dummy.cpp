#include <iostream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main ( int argc, char** argv )
{

  ros::init ( argc, argv, "tf_fake_broadcaster" );
  ros::NodeHandle node;
  static tf::TransformBroadcaster br;
  ros::Rate loop_rate ( 30 );

  std::cout << "Starting to publish fake tf transform from table to floor" << std::endl;

  // int count = 0;
  while ( ros::ok() )
  {
    tf::Transform transform;
    transform.setOrigin ( tf::Vector3 ( 0.0, 0.0, 0.0 ) );
    tf::Quaternion q;
    q.setRPY ( 0.0, 0.0, 0.0 );
    transform.setRotation ( q );
    br.sendTransform ( tf::StampedTransform ( transform, ros::Time::now(), "floor", "table") );
    ros::spinOnce ();
    loop_rate.sleep ();
    // ++count;
    // count = count % 10;
  }

  return 0;
};
