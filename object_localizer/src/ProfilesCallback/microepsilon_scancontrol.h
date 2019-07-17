#ifndef _MICROEPSILON_SCANCONTROL_ROS_H_
#define _MICROEPSILON_SCANCONTROL_ROS_H_

#include "libllt.h"
#include <vector>
#include <queue>
#include <iostream>

const int SCANNER_RESOLUTION = 1280;
std::vector < unsigned char > profile_buffer_;
std::vector < double > value_x, value_z;
unsigned int profile_counter;
double shutter_open;
double shutter_close;
double lag_compensation = 0.001;
ros::Time profile_time;
void save_profile_pc ();

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

  bool connect();
  bool disconnect();
  bool initialise();

  static void control_lost_callback_wrapper ( ArvGvDevice* gv_device, gpointer user_data );
  void control_lost_callback ( ArvGvDevice* gv_device );
  static void new_profile_callback_wrapper ( const void* data, size_t data_size, gpointer user_data );
  void new_profile_callback ( const void* data, size_t data_size );

public:
  Scanner ( std::string serial_number, unsigned int shutter_time, unsigned int idle_time, std::string device_properties_path );
  ~Scanner ();
  bool reconnect ();
  bool startScanning ();
  bool stopScanning ();
};

// connect to the laser scanner
bool Scanner::connect()
{
  if ( connected_ )
  {
    return true;
  }

  // list all connected microepsilon laser scanners, the maximum number is 5.
  unsigned int uiInterfaceCount = 0;
  int activeDevice = 0;
  std::vector <char *> vcInterfaces ( 5 );
  int iRetValue = GetDeviceInterfaces ( &vcInterfaces[0], vcInterfaces.size() );
  if ( iRetValue == ERROR_GETDEVINTERFACE_REQUEST_COUNT )
  {
    std::cout << "There are more than " << vcInterfaces.size() << " scanCONTROL connected \n";
    uiInterfaceCount = vcInterfaces.size();
  }
  else if ( iRetValue < 1 )
  {
    std::cout << "A error occured during searching for connected scanCONTROL \n";
    uiInterfaceCount = 0;
    return false;
  }
  else
  {
    uiInterfaceCount = iRetValue;
    if ( uiInterfaceCount == 0 )
      std::cout << "There is no scanCONTROL connected \n";
    else if ( uiInterfaceCount == 1 )
      std::cout << "There is 1 scanCONTROL connected \n";
    else
      std::cout << "There are " << uiInterfaceCount << " scanCONTROL connected \n";
    bool foundSN = false;
    for ( int i = 0; i < uiInterfaceCount; ++i )
    {
      std::cout << "***" << i << ":" << vcInterfaces[i] << std::endl;
      std::string tempStr = vcInterfaces[i];
      if ( serial_number_.size () != 0 &&
           tempStr.compare ( tempStr.size () - serial_number_.size (), serial_number_.size (), serial_number_ ) == 0 )
      {
        std::cout << "Found Device with serial number: " << serial_number_ << std::endl;
        foundSN = true;
        activeDevice = i;
        break;
      }
    }
    if ( !foundSN && serial_number_.size() != 0 )
    {
      std::cout << "Could not find device with S/N: " << serial_number_ << ". Using first device in list." << std::endl;
    }
  }

  if ( ( iRetValue = SetPathtoDeviceProperties ( path_to_device_properties_.c_str() ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error setting device properties path:\n\t[" << path_to_device_properties_ << "]\n";
    return false;
  }

  std::cout << "Connecting to " << vcInterfaces[ activeDevice ] << std::endl;
  if ( ( llt_.SetDeviceInterface( vcInterfaces[ activeDevice ] ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting dev id " << iRetValue << "!\n";
    return false;
  }

  // Connect to sensor
  if ( ( iRetValue = llt_.Connect() ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while connecting to camera - Error " << iRetValue << "!\n";
    return false;
  }

  // print the type of microepsilon laser scanner
  TScannerType m_tscanCONTROLType;
  if ( ( iRetValue = llt_.GetLLTType( &m_tscanCONTROLType ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while GetLLTType!\n";
    return false;
  }
  if (iRetValue == GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED)
  {
    std::cout << "Can't decode scanCONTROL type. Please contact Micro Epsilon for a newer version of the library.\n";
  }
  if ( m_tscanCONTROLType == scanCONTROL27xx_xxx )
  {
    std::cout << "The scanCONTROL is a scanCONTROL27xx\n";
  }
  else if ( m_tscanCONTROLType == scanCONTROL26xx_xxx )
  {
    std::cout << "The scanCONTROL is a scanCONTROL26xx\n";
  }
  else if ( m_tscanCONTROLType == scanCONTROL29xx_xxx )
  {
    std::cout << "The scanCONTROL is a scanCONTROL29xx\n";
  }
  else
  {
    std::cout << "The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK\n";
  }

  connected_ = true;
  return true;
}

// disconnect the laser scanner
bool Scanner::disconnect ()
{
  if ( !connected_ )
  {
    return true;
  }
  llt_.Disconnect ();
  connected_ = false;
  return true;
}

// setup the parameters for me laser scanner
bool Scanner::initialise()
{
  if ( !connected_ )
  {
    return false;
  }

  if ( llt_.SetResolution ( SCANNER_RESOLUTION ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting resolution!\n";
    return false;
  }

  if ( llt_.SetProfileConfig ( PROFILE ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting PROFILE Mode!\n";
    return false;
  }

  if ( llt_.SetFeature ( FEATURE_FUNCTION_IDLETIME, idle_time_ ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting uiIdleTime!\n";
    return false;
  }

  if ( llt_.SetFeature ( FEATURE_FUNCTION_SHUTTERTIME, shutter_time_ ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting uiShutterTime!\n";
    return false;
  }

  if ( llt_.SetFeature ( FEATURE_FUNCTION_TRIGGER, 0x00000000 ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting trigger!\n";
    return false;
  }

  // Register Callbacks for Profiles
  if ( ( llt_.RegisterBufferCallback ( ( gpointer ) &Scanner::new_profile_callback_wrapper, this ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while registering buffer callback!\n";
    return false;
  }

  if ( ( llt_.RegisterControlLostCallback ( ( gpointer ) &Scanner::control_lost_callback_wrapper, this ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while registering control lost callback!\n";
    return false;
  }

  profile_buffer_.resize ( SCANNER_RESOLUTION * 64 );
  value_x.resize ( SCANNER_RESOLUTION );
  value_z.resize ( SCANNER_RESOLUTION );;
  return true;
}

void Scanner::control_lost_callback_wrapper ( ArvGvDevice *gv_device, gpointer user_data )
{
  ( (Scanner *) user_data )->control_lost_callback ( gv_device );
}

void Scanner::control_lost_callback ( ArvGvDevice *gv_device )
{
  connected_ = false;
  bool was_scanning = scanning_;
  std::cout << " Connection to scanner lost! Trying to reconnect! " << std::endl;
  reconnect ();
  if ( was_scanning )
  {
    startScanning ();
  }
}

void Scanner::new_profile_callback_wrapper ( const void *data, size_t data_size, gpointer user_data )
{
  ( (Scanner *) user_data )->new_profile_callback ( data, data_size );
}

double average ( double a, double b )
{
	return ( a + b ) / 100000.0 / 2.0;
}

void Scanner::new_profile_callback ( const void *data, size_t data_size )
{
  if ( data != NULL && data_size == profile_buffer_.size() )
  {
    memcpy ( &profile_buffer_[0], data, data_size );
    Timestamp2TimeAndCount ( &profile_buffer_[0], &shutter_open, &shutter_close, &profile_counter );
    ConvertProfile2Values ( &profile_buffer_[0], profile_buffer_.size(), &llt_.appData, SCANNER_RESOLUTION, 0, NULL, NULL, NULL, &value_x [ 0 ], &value_z [ 0 ], NULL, NULL );
    if ( profile_counter != 0 )
    {
      std::cout << "Profile: [" << profile_counter << "] has time [shutter_open, shutter_close] = [" << shutter_open << "," << shutter_close << "]" << std::endl;
      profile_time = ros::Time::now() - ros::Duration ( average ( shutter_open, shutter_close ) + lag_compensation );
      save_profile_pc ();
    }
  }
}

Scanner::Scanner ( std::string serial_number, unsigned int shutter_time, unsigned int idle_time, std::string device_properties_path )
  : serial_number_ ( serial_number ), shutter_time_ ( shutter_time ), idle_time_ ( idle_time )
{
  connected_ = false;
  scanning_ = false;
  path_to_device_properties_ = device_properties_path;

  connect();
  if ( connected_ )
  {
    if ( !initialise() )
    {
      disconnect();
      return;
    }
  }
}

Scanner::~Scanner()
{
  if ( connected_ )
  {
    if ( scanning_ )
    {
      stopScanning ();
    }
    disconnect ();
  }
}

bool Scanner::startScanning ()
{
  if ( !connected_ )
  {
    return false;
  }
  if ( scanning_ )
  {
    return true;
  }

  int iRetValue;
  if ( ( iRetValue = llt_.TransferProfiles ( NORMAL_TRANSFER, true ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error in profile transfer!\n";
    return false;
  }
  scanning_ = true;
  return true;
}

bool Scanner::stopScanning ()
{
  if ( !connected_ )
  {
    scanning_ = false;
    return true;
  }
  if ( !scanning_ )
  {
    return true;
  }
  if ( ( llt_.TransferProfiles ( NORMAL_TRANSFER, false ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while stopping transmission!\n";
    return false;
  }
  scanning_ = false;
  return true;
}

bool Scanner::reconnect()
{
  if ( connected_ )
  {
    if ( scanning_ )
    {
      stopScanning ();
    }
    disconnect ();
  }
  connect ();
  if ( connected_ )
  {
    if ( !initialise() )
    {
      return false;
    }
  }
  else
  {
    return false;
  }
  return true;
}

#endif
