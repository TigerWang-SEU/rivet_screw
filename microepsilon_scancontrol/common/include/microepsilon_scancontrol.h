#ifndef _MICROEPSILON_SCANCONTROL_ROS_H_
#define _MICROEPSILON_SCANCONTROL_ROS_H_

#include "libllt.h"
#include <vector>
#include <queue>
#include <iostream>
#include <boost/shared_ptr.hpp>

namespace microepsilon_scancontrol
{
  const int SCANNER_RESOLUTION = 1280;

  struct ScanProfileConverted
  {
    std::vector < double > x;
    std::vector < double > z;
    unsigned int profile_counter;
    double shutter_open;
    double shutter_close;
  };

  typedef boost::shared_ptr < ScanProfileConverted > ScanProfileConvertedPtr;

  class Notifyee
  {
  public:
    virtual void notify ( ScanProfileConvertedPtr ) = 0;
  };

  class Scanner
  {
  private:
    bool scanning_;
    bool connected_;

    unsigned int idle_time_;
    unsigned int shutter_time_;
    std::string serial_number_;
    std::string path_to_device_properties_;

    LLT llt_;
    Notifyee* notifyee_;

    bool connect();
    bool disconnect();
    bool initialise();

    static void control_lost_callback_wrapper ( ArvGvDevice* gv_device, gpointer user_data );
    void control_lost_callback ( ArvGvDevice* gv_device );
    static void new_profile_callback_wrapper ( const void* data, size_t data_size, gpointer user_data );
    void new_profile_callback ( const void* data, size_t data_size );

  public:
    Scanner ( Notifyee* notifyee, unsigned int shutter_time, unsigned int idle_time,
              std::string serial_number, std::string path_to_device_properties );
    ~Scanner ();
    bool reconnect ();
    bool startScanning ();
    bool stopScanning ();
  };

}  // namespace microepsilon_scancontrol
#endif
