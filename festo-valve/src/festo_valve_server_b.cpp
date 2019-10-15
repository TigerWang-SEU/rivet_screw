#include "ros/ros.h"
#include <iostream>
#include <vector>
#include "modbus.h"
#include <boost/utility/binary.hpp>
#include <bitset>

class FestoValveController
{
private:
  boost::shared_ptr < modbus > mb_ptr;
  std::string ip_address;
  uint16_t current_state = 0b0000000000000000;
  ros::NodeHandle nh_;
  ros::ServiceServer start_rough_localizer_, stop_rough_localizer_;
public:
  FestoValveController ();
  ~FestoValveController ();
  void open_valve ( int valve_no );
  void close_valve ( int valve_no );

  // bool start_new_nut ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  // {
  //   open_valve ( 1 );
  //   return true;
  // }
  //
  // bool stop_new_nut ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  // {
  //   close_valve ( 1 );
  //   return true;
  // }
  //
  // bool start_lift_table ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  // {
  //   open_valve ( 2 );
  //   return true;
  // }
  //
  // bool stop_lift_table ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  // {
  //   close_valve ( 2 );
  //   return true;
  // }

};

FestoValveController::FestoValveController ()
{
  ros::NodeHandle nh_p_ ( "~" );
  nh_p_.getParam ( "ip_address", ip_address );
  mb_ptr.reset ( new modbus ( ip_address, 502 ) );
  mb_ptr->modbus_set_slave_id ( 1 );
  mb_ptr->modbus_connect ();

  // start_rough_localizer_ = nh_.advertiseService ( "start_rough_localizer", &RoughLocalizer::start_rough_localizer, this );
  // stop_rough_localizer_ = nh_.advertiseService ( "stop_rough_localizer", &RoughLocalizer::stop_rough_localizer, this );
  // start_lift_table = nh_.advertiseService ( 'start_lift_table', Empty, start_lift_table )
  // stop_lift_table = nh_.advertiseService ( 'stop_lift_table', Empty, stop_lift_table )
  // start_new_nut = nh_.advertiseService ( 'start_new_nut', Empty, start_new_nut )
  // stop_new_nut = nh_.advertiseService ( 'stop_new_nut', Empty, stop_new_nut )
}

FestoValveController::~FestoValveController ()
{
  mb_ptr->modbus_close ();
}

void FestoValveController::open_valve ( int valve_no )
{
  uint16_t target_state = current_state;
  std::cout << "current_state = " << std::bitset < 16 > ( current_state ) << std::endl;
  if ( valve_no == 1 )
  {
    uint16_t valve_mask = 0b0000000000000001;
    target_state = current_state | valve_mask;
  }
  if ( valve_no == 2 )
  {
    uint16_t valve_mask = 0b0000000000000010;
    target_state = current_state | valve_mask;
  }
  mb_ptr->modbus_write_register ( 40003, target_state );
  current_state = target_state;
}

void FestoValveController::close_valve ( int valve_no)
{
  uint16_t target_state = current_state;
  std::cout << "current_state = " << std::bitset < 16 > ( current_state ) << std::endl;
  if ( valve_no == 1 )
  {
    uint16_t valve_mask = 0b1111111111111110;
    target_state = current_state & valve_mask;
  }
  if ( valve_no == 2 )
  {
    uint16_t valve_mask = 0b1111111111111101;
    target_state = current_state & valve_mask;
  }
  mb_ptr->modbus_write_register ( 40003, target_state );
  current_state = target_state;
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "festo_valve_server_b" );
  ros::NodeHandle nh_p_ ( "~" );
  nh_p_.setParam ( "ip_address", "192.168.2.183" );
  ros::AsyncSpinner spinner ( 4 );
  spinner.start ();
  // to do somehting
  FestoValveController fv_con;
  fv_con.open_valve ( 1 );
  ros::Duration ( 2 ).sleep ();
  fv_con.open_valve ( 2 );
  ros::Duration ( 2 ).sleep ();
  fv_con.close_valve ( 2 );
  ros::Duration ( 2 ).sleep ();
  fv_con.close_valve ( 1 );
  ros::Duration ( 2 ).sleep ();
  ros::waitForShutdown ();
  return 0;
}
