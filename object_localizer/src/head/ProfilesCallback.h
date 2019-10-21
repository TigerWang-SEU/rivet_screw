#ifndef PROFILESCALLBACK_H
#define PROFILESCALLBACK_H

#include <iostream>
#include <llt.h>
#include <vector>

#define MAX_INTERFACE_COUNT 5
#define MAX_RESOLUTION 6

// define functions for connecting and disconnecting laser scanner
void clean_up ( void );
bool connect_scanner ( std::string serial_number_ );
bool disconnect_scanner ( void );

void NewProfile ( const void *data, size_t data_size, gpointer user_data );
void ControlLostCallback ( gpointer user_data );

// laser scanner information
std::string  serial_number = "218020023";
CInterfaceLLT *hLLT;
guint32 resolution;
std::vector<guint8> profile_buffer;
TScannerType llt_type;
bool is_transfer = false;
// event handle
EHANDLE *event;

// clean up used hLLT and event
void clean_up ()
{
  delete hLLT;
  CInterfaceLLT::FreeEvent ( event );
}

// connect laser scanner
bool connect_scanner ( std::string serial_number_ )
{
  // 1. new LLT instance
  hLLT = new CInterfaceLLT ();
  event = CInterfaceLLT::CreateEvent ();

  gint32 ret = 0;

  char *interfaces [ MAX_INTERFACE_COUNT ];
  guint32 resolutions [ MAX_RESOLUTION ];
  guint32 interface_count = 0;

  guint32 idle_time = 700;
  guint32 shutter_time = 300;

  // 1. searching for scanCONTROL devices
  if ( ( ret = CInterfaceLLT::GetDeviceInterfaces ( &interfaces[0], MAX_INTERFACE_COUNT ) ) == ERROR_GETDEVINTERFACE_REQUEST_COUNT )
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
    if ( serial_number_.size () != 0 &&
         tempStr.compare ( tempStr.size () - serial_number_.size (), serial_number_.size (), serial_number_ ) == 0 )
    {
      std::cout << "Found Device with serial number: " << serial_number_ << std::endl;
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

  // 5. set device id
  if ( ( ret = hLLT->SetDeviceInterface ( interfaces[ activeDevice ] ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting dev id - Error " << ret << "!" << std::endl;
    clean_up ();
    return false;
  }

  // 6. connect to the laser scanner
  if ( ( ret = hLLT->Connect() ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while connecting to the laser scanner - Error " << ret << "!" << std::endl;
    clean_up ();
    return false;
  }

  // 7. get the laser scanner information
  if ( ( ret = hLLT->GetLLTType ( &llt_type ) ) < GENERAL_FUNCTION_OK )
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
  if ( llt_type >= scanCONTROL27xx_25 && llt_type <= scanCONTROL27xx_xxx )
  {
    std::cout << "The scanCONTROL is a scanCONTROL27xx" << std::endl;
  }
  else if ( llt_type >= scanCONTROL26xx_25 && llt_type <= scanCONTROL26xx_xxx )
  {
    std::cout << "The scanCONTROL is a scanCONTROL26xx" << std::endl;
  }
  else if ( llt_type >= scanCONTROL29xx_25 && llt_type <= scanCONTROL29xx_xxx )
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
  if ( ( ret = hLLT->GetResolutions ( &resolutions [ 0 ], MAX_RESOLUTION ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error GetResolutions!" << std::endl;
    clean_up ();
    return false;
  }

  // 8.1. set resolution to the max possible resolution
  resolution = resolutions [ 0 ];
  if ( hLLT->SetResolution ( resolution ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting resolution!" << std::endl;
    clean_up ();
    return false;
  }

  // 9. set profile
  if ( hLLT->SetProfileConfig ( PROFILE ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting SetProfileConfig!" << std::endl;
    clean_up ();
    return false;
  }

  // 10. set idle_time
  if ( hLLT->SetFeature ( FEATURE_FUNCTION_IDLETIME, idle_time ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_IDLETIME!" << std::endl;
    clean_up ();
    return false;
  }

  // 10. set shutter_time
  if ( hLLT->SetFeature ( FEATURE_FUNCTION_SHUTTERTIME, shutter_time ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_SHUTTERTIME!" << std::endl;
    clean_up ();
    return false;
  }

  // 10. set TRIG_INTERNAL
  if ( hLLT->SetFeature ( FEATURE_FUNCTION_TRIGGER, TRIG_INTERNAL ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_TRIGGER!" << std::endl;
    clean_up ();
    return false;
  }

  // 11. register Callbacks for program handling
  std::cout << "Register callbacks" << std::endl;
  if ( hLLT->RegisterBufferCallback ( (gpointer)&NewProfile, NULL ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while registering buffer callback!" << std::endl;
    clean_up ();
    return false;
  }

  if ( hLLT->RegisterControlLostCallback( (gpointer)&ControlLostCallback, NULL ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while registering control lost callback!" << std::endl;
    clean_up ();
    return false;
  }
  return true;
}

bool power_on ()
{
  // power on the laser scanner
  unsigned int value = 2;
  if ( hLLT->SetFeature ( FEATURE_FUNCTION_LASERPOWER, value ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_LASERPOWER!" << std::endl;
    return false;
  }
  // start profile transfer
  gint32 ret = 0;
  if ( !is_transfer )
  {
    if ( ( ret = hLLT->TransferProfiles ( NORMAL_TRANSFER, true ) ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error in profile transfer! - [" << ret << "]" << std::endl;
      return false;
    }
    is_transfer = true;
  }
  std::cout << "LASERPOWER ON!" << std::endl;
  return true;
}

bool power_off ()
{
  // stop profile transfer
  gint32 ret = 0;
  if ( is_transfer )
  {
    if ( ( ret = hLLT->TransferProfiles ( NORMAL_TRANSFER, false ) ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while stopping transmission! - [" << ret << "]" << std::endl;
      return false;
    }
    is_transfer = false;
  }
  // power off the laser scanner
  unsigned int value = 0;
  if ( hLLT->SetFeature ( FEATURE_FUNCTION_LASERPOWER, value ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_LASERPOWER!" << std::endl;
    return false;
  }
  std::cout << "LASERPOWER OFF!" << std::endl;
  return true;
}

// disconnect laser scanner
bool disconnect_scanner ()
{
  gint32 ret = 0;
  std::cout << "Disconnecting..." << std::endl;
  if ( ( ret = hLLT->Disconnect() ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while disconnecting - Error " << ret << "!" << std::endl;
  }
  clean_up();
  return true;
}

// save new profile into profile_buffer
void NewProfile ( const void *data, size_t data_size, gpointer user_data )
{
  if ( data_size == profile_buffer.size() )
  {
    memcpy ( &profile_buffer[0], data, data_size );
    set_event ( event );
  }
}

// control of the device is lost. Display a message and reconnect!
void ControlLostCallback ( gpointer user_data )
{
  std::cout << "Control lost" << std::endl;
  disconnect_scanner ();
  connect_scanner ( serial_number );
}

#endif // PROFILESCALLBACK_H
