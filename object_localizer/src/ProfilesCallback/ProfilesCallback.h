#ifndef PROFILESCALLBACK_H
#define PROFILESCALLBACK_H

#include <iostream>
#include <llt.h>
#include <vector>

#define MAX_INTERFACE_COUNT 5
#define MAX_RESOLUTION 6

gint32 GetProfilesCallback(void);
void DisplayProfiles(double *, double *, guint32);

void NewProfile(const void *data, size_t data_size, gpointer user_data);
void ControlLostCallback(gpointer user_data);

CInterfaceLLT *hLLT;
guint32 resolution;
std::vector<guint8> profile_buffer;
TScannerType llt_type;
guint32 profile_count, needed_profile_count;
guint32 profile_data_size;

// for time information
guint32 profile_counter = 0;
double shutter_closed = 0, shutter_opened = 0;

// event handle
EHANDLE *event;

bool connect_scanner ( std::string serial_number_ )
{
  gint32 ret = 0;

  char *interfaces[MAX_INTERFACE_COUNT];
  guint32 resolutions[MAX_RESOLUTION];
  guint32 interface_count = 0;

  guint32 idle_time = 900;
  guint32 shutter_time = 100;

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

  if ( interface_count == 0 )
  {
    std::cout << "There is no scanCONTROL connected - Exiting" << std::endl;
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
  // show there laser scanners and find the target laser scanner with a serial_number
  bool foundSN = false;
  int activeDevice = 0;
  for ( guint32 i = 0; i < interface_count; i++ )
  {
    std::cout << interfaces[i] << "" << std::endl;
    std::string tempStr = interfaces[i];
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
    return false;
  }

  // new LLT instance
  hLLT = new CInterfaceLLT();
  event = CInterfaceLLT::CreateEvent();

  if ((ret = hLLT->SetDeviceInterface(interfaces[ activeDevice ])) < GENERAL_FUNCTION_OK)
  {
    std::cout << "Error while setting dev id " << ret << "!" << std::endl;
    clean_up ();
    return false;
  }

  // connect to sensor
  if ( ( ret = hLLT->Connect() ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while connecting to camera - Error " << ret << "!" << std::endl;
    clean_up ();
    return false;
  }

  if ( ( ret = hLLT->GetLLTType ( &llt_type ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while GetLLTType!" << std::endl;
    clean_up ();
    return false;
  }

  if ( ret == GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED )
  {
    std::cout << "Can't decode scanCONTROL type. Please contact Micro Epsilon for a newer version of the library." << std::endl;
    clean_up ();
    return false;
  }

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

  std::cout << "Get all possible resolutions" << std::endl;
  if ( ( ret = hLLT->GetResolutions ( &resolutions[0], MAX_RESOLUTION ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error GetResolutions!" << std::endl;
    clean_up ();
    return false;
  }

  // set Resolution to max
  resolution = resolutions[0];
  if ( hLLT->SetResolution ( resolution ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting resolution!" << std::endl;
    clean_up ();
    return false;
  }

  if ( hLLT->SetProfileConfig ( PROFILE ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting SetProfileConfig!" << std::endl;
    clean_up ();
    return false;
  }

  if ( hLLT->SetFeature ( FEATURE_FUNCTION_IDLETIME, idle_time ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_IDLETIME!" << std::endl;
    clean_up ();
    return false;
  }

  if ( hLLT->SetFeature ( FEATURE_FUNCTION_SHUTTERTIME, shutter_time ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_SHUTTERTIME!" << std::endl;
    clean_up ();
    return false;
  }

  if ( hLLT->SetFeature ( FEATURE_FUNCTION_TRIGGER, TRIG_INTERNAL ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while setting FEATURE_FUNCTION_TRIGGER!" << std::endl;
    clean_up ();
    return false;
  }

  std::cout << "Register callbacks" << std::endl;
  // register Callbacks for program handling
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

bool Scanner::disconnect()
{
  std::cout << "Disconnecting..." << std::endl;
  if ( ( ret = hLLT->Disconnect() ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while disconnecting - Error " << ret << "!" << std::endl;
  }
  clean_up();
  return true;
}

void clean_up ()
{
  delete hLLT;
  CInterfaceLLT::FreeEvent(event);
}

gint32 save_profile ()
{
  gint32 ret = 0;

  std::vector<double> value_x, value_z;
  profile_buffer.resize ( resolution * 64 );
  value_x.resize ( resolution );
  value_z.resize ( resolution );

  CInterfaceLLT::ResetEvent ( event );
  // start transfer
  if ( ( ret = hLLT->TransferProfiles(NORMAL_TRANSFER, true ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error in profile transfer! " << ret << "" << std::endl;
    return ret;
  }

  while ()
  {
    std::cout << "Start acquisition of profiles" << std::endl;
    if ( CInterfaceLLT::WaitForSingleObject ( event, 2000 ) != WAIT_OBJECT_0 )
    {
      std::cout << "Timeout!" << std::endl;
    }
  }

  // stop transfer
  if ( ( ret = hLLT->TransferProfiles ( NORMAL_TRANSFER, false ) ) < GENERAL_FUNCTION_OK )
  {
    std::cout << "Error while stopping transmission!" << std::endl;
    return ret;
  }

  // display example points
  if ( ( ret = CInterfaceLLT::ConvertProfile2Values ( &profile_buffer[0], profile_buffer.size(), resolution, PROFILE, llt_type, 0, NULL, NULL, NULL, &value_x[0], &value_z[0], NULL, NULL ) ) != ( CONVERT_X | CONVERT_Z ) )
  {
    std::cout << "Error while extracting profiles" << std::endl;
    return ret;
  }

  return GENERAL_FUNCTION_OK;
}

// save new profile into profile_buffer
void NewProfile ( const void *data, size_t data_size, gpointer user_data )
{
  if ( data_size == profile_buffer.size() )
  {
    profile_data_size = data_size;
    memcpy ( &profile_buffer[0], data, data_size );
    CInterfaceLLT::Timestamp2TimeAndCount ( &profile_buffer[ (resolution * 64) - 16 ], &shutter_closed, &shutter_opened, &profile_counter, NULL);
  }
  set_event ( event );
}

void ControlLostCallback(gpointer user_data)
{
  // control of the device is lost. Display a message and exit!
  std::cout << "Control lost" << std::endl;
  exit(0);
}

void DisplayProfiles(double *x, double *z, guint32 resolution)
{
  for ( guint32 i = 0; i < resolution; i++ )
  {
    std::cout << "\rX: " << x[i] << "  Z: " << z[i];
    usleep(1250);
  }
  std::cout << std::endl;
}

#endif // PROFILESCALLBACK_H
