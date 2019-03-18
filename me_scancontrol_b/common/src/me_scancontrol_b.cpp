#include "me_scancontrol_b.h"

namespace microepsilon_scancontrol
{
  // connect to the me laser scanner
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
    int iRetValue = CInterfaceLLT::GetDeviceInterfaces ( &vcInterfaces[0], vcInterfaces.size() );
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

    std::shared_ptr < CInterfaceLLT > hLLT ( new CInterfaceLLT() );

    // Set Path to device_properties.dat (optional)
    if ( ( iRetValue = hLLT->SetPathDeviceProperties ( path_to_device_properties_.c_str() ) ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error setting device properties path:\n\t[" << path_to_device_properties_ << "]\n";
      return false;
    }
    // Connect to sensor
    std::cout << "Connecting to " << vcInterfaces[ activeDevice ] << std::endl;
    if ( ( iRetValue = hLLT->SetDeviceInterface ( vcInterfaces[ activeDevice ] ) ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while setting dev id " << iRetValue << "!\n";
      return false;
    }
    if ( ( iRetValue = hLLT->Connect () ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while connecting to camera - Error " << iRetValue << "!\n";
      return false;
    }

    // print the type of microepsilon laser scanner
    if ( ( iRetValue = hLLT->GetLLTType ( &m_tscanCONTROLType ) ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while GetLLTType!\n";
      return false;
    }
    if ( iRetValue == GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED )
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

  bool Scanner::disconnect()
  {
    if ( !connected_ )
    {
      return true;
    }
    hLLT->Disconnect ();
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

    if ( hLLT->SetResolution ( SCANNER_RESOLUTION ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while setting resolution!\n";
      return false;
    }

    if ( ( hLLT->SetProfileConfig ( PROFILE ) ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while setting PROFILE Mode!\n";
      return false;
    }

    // if ( ( hLLT->SetBufferCount ( 4 ) ) < GENERAL_FUNCTION_OK)
    // {
    //   std::cout << "Error while setting BufferCount!\n";
    //   return false;
    // }

    if ( hLLT->SetFeature ( FEATURE_FUNCTION_IDLETIME, idle_time_ ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while setting uiIdleTime!\n";
      return false;
    }

    if ( hLLT->SetFeature ( FEATURE_FUNCTION_SHUTTERTIME, shutter_time_) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while setting uiShutterTime!\n";
      return false;
    }

    if ( hLLT->SetFeature ( FEATURE_FUNCTION_TRIGGER, 0x00000000 ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while setting trigger!\n";
      return false;
    }

    // Register Callbacks for Profiles
    if ( ( hLLT->RegisterBufferCallback ( ( gpointer ) &Scanner::new_profile_callback_wrapper, this ) ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while registering buffer callback!\n";
      return false;
    }

    if ( ( hLLT->RegisterControlLostCallback ( ( gpointer ) &Scanner::control_lost_callback_wrapper, this ) ) < GENERAL_FUNCTION_OK )
    {
      std::cout << "Error while registering control lost callback!\n";
      return false;
    }

    return true;
  }

  void Scanner::control_lost_callback_wrapper ( ArvGvDevice *gv_device, gpointer user_data )
  {
    ( (Scanner *) user_data )->control_lost_callback ( gv_device );
  }

  void Scanner::control_lost_callback ( ArvGvDevice *gv_device )
  {
    connected_ = false;
    // get the old scanning setting
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

  void Scanner::new_profile_callback ( const void *data, size_t data_size )
  {
    // save the input data
    std::vector < unsigned char > profile_buffer_;
    profile_buffer_.resize ( SCANNER_RESOLUTION * 64 );
    if ( data != NULL && data_size == profile_buffer_.size() )
    {
      memcpy ( &profile_buffer_[0], data, data_size );
    }

    ScanProfileConvertedPtr profile ( new ScanProfileConverted );
    CInterfaceLLT::Timestamp2TimeAndCount ( &profile_buffer_[0], &( profile->shutter_open ), &( profile->shutter_close ), &( profile->profile_counter ), NULL );
    // show the time information of the input profile
    // std::cout <<"[profile_counter, shutter_open, shutter_close] = [" << profile->profile_counter << ", " << profile->shutter_open << ", " << profile->shutter_close << "]" << std::endl;
    profile->x.resize ( SCANNER_RESOLUTION );
    profile->z.resize ( SCANNER_RESOLUTION );
    // CInterfaceLLT::ConvertProfile2Values ( &profile_buffer_[0], profile_buffer_.size(), SCANNER_RESOLUTION, 0, NULL, NULL, NULL, &( profile->x[0] ), &( profile->z[0] ), NULL, NULL );

    gint32 ret = 0;
    if ( ( ret = CInterfaceLLT::ConvertProfile2Values ( &profile_buffer_[0], profile_buffer_.size(), SCANNER_RESOLUTION, PROFILE, m_tscanCONTROLType, 0,
                                                        NULL, NULL, NULL, &( profile->x[0] ), &( profile->z[0] ), NULL, NULL ) ) != ( CONVERT_X | CONVERT_Z ) )
    {
        std::cout << "Error while extracting profiles" << std::endl;
    }

    // publish the new profile
    notifyee_->notify( profile );
  }

/////////////////////////////////////////////////////////////////////////////////////////////////////////

  Scanner::Scanner ( Notifyee *notifyee, unsigned int shutter_time, unsigned int idle_time,
                     std::string serial_number, std::string path_to_device_properties )
    : notifyee_ ( notifyee ), shutter_time_ ( shutter_time ), idle_time_ ( idle_time ), serial_number_ ( serial_number )
  {
    connected_ = false;
    scanning_ = false;
    path_to_device_properties_ = path_to_device_properties + "/device_properties.dat";

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
    if ( ( iRetValue = hLLT->TransferProfiles ( NORMAL_TRANSFER, true ) ) < GENERAL_FUNCTION_OK )
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
    if ( ( hLLT->TransferProfiles ( NORMAL_TRANSFER, false ) ) < GENERAL_FUNCTION_OK )
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

}  // namespace microepsilon_scancontrol
