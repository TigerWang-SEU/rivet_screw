#ifndef PROFILESCALLBACK_H
#define PROFILESCALLBACK_H

#include <ros/package.h>
#include <iostream>
#include <vector>
#include "libllt.h"

#define MAX_INTERFACE_COUNT 5
#define MAX_RESOLUTION 6

guint32 profile_counter = 0;
double shutter_closed = 0, shutter_opened = 0;
ros::Time profile_time;
std::vector < double > value_x, value_z;
void save_profile_pc ();

// laser scanner information
class Scanner
{
private:
  std::string serial_number = "218020023";
  const int SCANNER_RESOLUTION = 1280;
  guint32 idle_time = 800;
  guint32 shutter_time = 200;
  double lag_compensation = 0.001;
  std::string device_properties_path = ros::package::getPath ( "microepsilon_scancontrol" ) + "/scanCONTROL_Linux_SDK_0.1.0/device_properties.dat";
  LLT hLLT;
  TScannerType llt_type;
  std::vector < guint8 > profile_buffer;

public:

  guint32 resolution;

  Scanner ();
  ~Scanner ();
  // define functions for connecting and disconnecting laser scanner
  void clean_up ( void );
  bool connect_scanner ();
  bool disconnect_scanner ( void );

  bool start_transfer_profiles ()
  {
    gint32 ret = 0;
    if ( ( ret = hLLT.TransferProfiles ( NORMAL_TRANSFER, true ) ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error in profile transfer! - [" << ret << "]" << std::endl;
      return false;
    }
    return true;
  }

  bool stop_transfer_profiles ()
  {
    gint32 ret = 0;
    if ( ( ret = hLLT.TransferProfiles ( NORMAL_TRANSFER, false ) ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while stopping transmission! - [" << ret << "]" << std::endl;
      return false;
    }
    return true;
  }

  void NewProfile ( const void *data, size_t data_size, gpointer user_data );
  void ControlLostCallback ( gpointer user_data );
};

Scanner::Scanner () {}

Scanner::~Scanner() {}

// clean up used hLLT and event
void Scanner::clean_up ()
{
  // delete hLLT;
}

// connect laser scanner
bool Scanner::connect_scanner ()
{
  // 1. new LLT instance
  // hLLT = new LLT ();

  gint32 ret = 0;
  char *interfaces [ MAX_INTERFACE_COUNT ];
  guint32 resolutions [ MAX_RESOLUTION ];
  guint32 interface_count = 0;

  // 1. searching for scanCONTROL devices
  if ( ( ret = GetDeviceInterfaces ( &interfaces[0], MAX_INTERFACE_COUNT ) ) == ERROR_GETDEVINTERFACE_REQUEST_COUNT )
  {
    std::cout << "There are more than " << MAX_INTERFACE_COUNT << " scanCONTROL connected" << std::endl;
    interface_count = MAX_INTERFACE_COUNT;
  }
  else if ( ret < 1 )
  {
    std::cout << "A error occured during searching for connected scanCONTROL" << std::endl;
    interface_count = 0;
  }
  else
  {
    interface_count = ret;
  }

  // 2. show the number of found scanCONTROLs
  if ( interface_count == 0 )
  {
    std::cout << "There is no scanCONTROL connected - Exiting" << std::endl;
    clean_up ();
    return false;
  }
  else if ( interface_count == 1 )
  {
    std::cout << "There is 1 scanCONTROL connected " << std::endl;
  }
  else
  {
    std::cout << "There are " << interface_count << " scanCONTROL connected" << std::endl;
  }

  // 3. show there laser scanners
  //    and find the target laser scanner with a serial_number
  bool foundSN = false;
  int activeDevice = 0;
  for ( guint32 i = 0; i < interface_count; i++ )
  {
    std::cout << interfaces [ i ] << "" << std::endl;
    std::string tempStr = interfaces [ i ];
    if ( serial_number.size () != 0 &&
         tempStr.compare ( tempStr.size () - serial_number.size (), serial_number.size (), serial_number ) == 0 )
    {
      std::cout << "Found Device with serial number: " << serial_number << std::endl;
      foundSN = true;
      activeDevice = i;
      break;
    }
  }
  // if not find the target laser scanner
  if( !foundSN )
  {
    clean_up ();
    return false;
  }

  if ( ( ret = SetPathtoDeviceProperties ( device_properties_path.c_str() ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error setting device properties path:\n\t[" << device_properties_path << "]\n";
    clean_up ();
    return false;
  }

  // 5. set device id
  std::cout << "Connecting to " << interfaces[ activeDevice ] << std::endl;
  if ( ( ret = hLLT.SetDeviceInterface ( interfaces[ activeDevice ] ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting dev id - Error " << ret << "!" << std::endl;
    clean_up ();
    return false;
  }

  // 6. connect to the laser scanner
  if ( ( ret = hLLT.Connect() ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while connecting to the laser scanner - Error " << ret << "!" << std::endl;
    clean_up ();
    return false;
  }

  std::cout << "### I am here ###" << std::endl;

  // 7. get the laser scanner information
  if ( ( ret = hLLT.GetLLTType ( &llt_type ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while GetLLTType! - Error " << ret << "!" << std::endl;
    clean_up ();
    return false;
  }

  if ( ret == GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED )
  {
    std::cout << "Can't decode scanCONTROL type. Please contact Micro Epsilon for a newer version of the library." << std::endl;
    clean_up ();
    return false;
  }
  // 7.1. print the laser scanner information
  if ( llt_type == scanCONTROL27xx_xxx )
  {
    std::cout << "The scanCONTROL is a scanCONTROL27xx" << std::endl;
  }
  else if ( llt_type == scanCONTROL26xx_xxx )
  {
    std::cout << "The scanCONTROL is a scanCONTROL26xx" << std::endl;
  }
  else if ( llt_type == scanCONTROL29xx_xxx )
  {
    std::cout << "The scanCONTROL is a scanCONTROL29xx" << std::endl;
  }
  else
  {
    std::cout << "The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK" << std::endl;
    clean_up ();
    return false;
  }

  // 8. get all possible resolutions of the selected laser scanner
  std::cout << "Get all possible resolutions" << std::endl;
  if ( ( ret = hLLT.GetResolutions ( &resolutions [ 0 ], MAX_RESOLUTION ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error GetResolutions!" << std::endl;
    clean_up ();
    return false;
  }

  // 8.1. set resolution to the max possible resolution
  resolution = resolutions [ 0 ];
  std::cout << "Set resolution [" << resolution << "]" << std::endl;
  if ( hLLT.SetResolution ( resolution ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting resolution!" << std::endl;
    clean_up ();
    return false;
  }

  // 9. set profile
  if ( hLLT.SetProfileConfig ( PROFILE ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting SetProfileConfig!" << std::endl;
    clean_up ();
    return false;
  }

  // 10. set idle_time
  if ( hLLT.SetFeature ( FEATURE_FUNCTION_IDLETIME, idle_time ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_IDLETIME!" << std::endl;
    clean_up ();
    return false;
  }

  // 10. set shutter_time
  if ( hLLT.SetFeature ( FEATURE_FUNCTION_SHUTTERTIME, shutter_time ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_SHUTTERTIME!" << std::endl;
    clean_up ();
    return false;
  }

  // 10. set TRIG_INTERNAL
  if ( hLLT.SetFeature ( FEATURE_FUNCTION_TRIGGER, 0x00000000 ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_TRIGGER!" << std::endl;
    clean_up ();
    return false;
  }

  // 11. register Callbacks for program handling
  std::cout << "Register callbacks" << std::endl;
  if ( hLLT.RegisterBufferCallback ( (gpointer)&Scanner::NewProfile, NULL ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while registering buffer callback!" << std::endl;
    clean_up ();
    return false;
  }

  if ( hLLT.RegisterControlLostCallback( (gpointer)&Scanner::ControlLostCallback, NULL ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while registering control lost callback!" << std::endl;
    clean_up ();
    return false;
  }

  profile_buffer.resize ( resolution * 64 );
  value_x.resize ( resolution );
  value_z.resize ( resolution );

  return true;
}

// disconnect laser scanner
bool Scanner::disconnect_scanner ()
{
  gint32 ret = 0;
  std::cout << "Disconnecting..." << std::endl;
  if ( ( ret = hLLT.Disconnect() ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while disconnecting - Error " << ret << "!" << std::endl;
  }
  clean_up();
  return true;
}

double average ( double a, double b )
{
	return ( a + b ) / 100000.0 / 2.0;
}

// save new profile into profile_buffer
void Scanner::NewProfile ( const void *data, size_t data_size, gpointer user_data )
{
  if ( data != NULL && data_size == profile_buffer.size() )
  {
    // 1. copy data to profile_buffer
    memcpy ( &profile_buffer[0], data, data_size );

    // 2. get x and z arrays
  	ConvertProfile2Values ( &profile_buffer [ 0 ], profile_buffer.size(), & ( hLLT.appData ), resolution, 0, NULL, NULL, NULL, &value_x [ 0 ], &value_z [ 0 ], NULL, NULL );

  	// 3. get profile_counter and time information
  	Timestamp2TimeAndCount ( &profile_buffer[0], &shutter_opened, &shutter_closed, &profile_counter );
  	std::cout << "Profile: [" << profile_counter << "] has time [shutter_closed, shutter_opened] = [" << shutter_closed << "," << shutter_opened << "]" << std::endl;
    profile_time = ros::Time::now() - ros::Duration ( average ( shutter_opened, shutter_closed ) + lag_compensation );

    // 4. create a profile point cloud
    save_profile_pc ();
  }
}

// control of the device is lost. Display a message and reconnect!
void Scanner::ControlLostCallback ( gpointer user_data )
{
  std::cout << "Control lost" << std::endl;
  disconnect_scanner ();
  connect_scanner ();
}

#endif // PROFILESCALLBACK_H
