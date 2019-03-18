#include "ros/ros.h"
#include "microepsilon_scancontrol.h"

#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>

#include <libllt.h>
#include <iostream>
#include <vector>

#define MAX_INTERFACE_COUNT 	5
std::vector<char *> vcInterfaces ( MAX_INTERFACE_COUNT );

void ControlLostCallback ( ArvGvDevice *gv_device, gpointer ptr );
void DisplayProfiles ( double * , double * , guint32 );
void NewProfile ( const void * , size_t , gpointer );

LLT * m_pLLT;
guint32 m_uiResolution;
#define MAX_RESOLUTION 		6
std::vector<guint32> vuiResolutions ( MAX_RESOLUTION );

std::vector<unsigned char> m_uiProfileBuffer;

using namespace std;

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "me_test" );

  guint32 uiInterfaceCount = 0;
  guint32 uiIdleTime = 700;
  guint32 uiShutterTime = 300;

  int iRetValue = GetDeviceInterfaces ( &vcInterfaces[0], vcInterfaces.size() );
  if ( iRetValue == ERROR_GETDEVINTERFACE_REQUEST_COUNT )
  {
      cout << "There are more than " << vcInterfaces.size() << " scanCONTROL connected \n";
      uiInterfaceCount = vcInterfaces.size();
  }
  else if (iRetValue < 1)
  {
      cout << "A error occured during searching for connected scanCONTROL \n";
      uiInterfaceCount = 0;
  }
  else
      uiInterfaceCount = iRetValue;

  if (uiInterfaceCount == 0)
      cout << "no interface found" << endl;

  for (guint32 i = 0; i < uiInterfaceCount; i++)
  {
      cout << "Interface: " << vcInterfaces[i] << "\n";
  }

  /* Make an instance for each LLT */
  m_pLLT = new LLT();

  if ( ( iRetValue = SetPathtoDeviceProperties( "/home/syn/ros_ws/src/microepsilon_scancontrol/misc/device_properties.dat" ) ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error setting device ID path\nExit program...\n";
      exit(0);
  }

  if ( ( m_pLLT->SetDeviceInterface( vcInterfaces[0] ) ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while setting dev id " << iRetValue << "!\n";
      exit(0);
  }

  // Connect to sensor
  if ( ( iRetValue = m_pLLT->Connect() ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while connecting to camera - Error " << iRetValue << "!\n";
      exit(0);
  }

  TScannerType m_tscanCONTROLType;
  if ( ( iRetValue = m_pLLT->GetLLTType( &m_tscanCONTROLType ) ) < GENERAL_FUNCTION_OK)
  {
      cout << "Error while GetLLTType!\n";
      exit(0);
  }

  if (iRetValue == GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED)
  {
      cout << "Can't decode scanCONTROL type. Please contact Micro Epsilon for a newer version of the library.\n";
  }

  if (m_tscanCONTROLType == scanCONTROL27xx_xxx)
  {
      cout << "The scanCONTROL is a scanCONTROL27xx\n";
  }
  else if (m_tscanCONTROLType == scanCONTROL26xx_xxx)
  {
      cout << "The scanCONTROL is a scanCONTROL26xx\n";
  }
  else if (m_tscanCONTROLType == scanCONTROL29xx_xxx)
  {
      cout << "The scanCONTROL is a scanCONTROL29xx\n";
  }
  else
  {
      cout << "The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK\n";
  }

  cout << "Get all possible resolutions\n";
  if ( ( iRetValue = m_pLLT->GetResolutions( &vuiResolutions[0], vuiResolutions.size() ) ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error GetResolutions!\n";
      exit(0);
  }
  int counter = 0;
  for ( m_uiResolution : vuiResolutions )
  {
    std::cout << "Resolution [" << counter << "] :" <<  vuiResolutions[ counter ] << std::endl;
    counter ++;
  }
  // using the first resolution
  std::cout << vuiResolutions[0] << std::endl;
  m_uiResolution = vuiResolutions[0];

  if ( m_pLLT->SetResolution ( m_uiResolution ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while setting transmission mode!\n";
      exit(0);
  }

  if ( m_pLLT->SetProfileConfig ( PROFILE ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while setting SetProfileConfig!\n";
      exit(0);
  }

  if ( m_pLLT->SetFeature ( FEATURE_FUNCTION_IDLETIME, uiIdleTime ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while setting FEATURE_FUNCTION_IDLETIME!\n";
      exit(0);
  }

  if ( m_pLLT->SetFeature ( FEATURE_FUNCTION_SHUTTERTIME, uiShutterTime ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while setting FEATURE_FUNCTION_SHUTTERTIME!\n";
      exit(0);
  }

  if ( m_pLLT->SetFeature ( FEATURE_FUNCTION_TRIGGER, 0x00000000 ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while setting FEATURE_FUNCTION_TRIGGER!\n";
      exit(0);
  }

  // Register Callbacks for program handling
  cout << "Register callbacks\n";
  int llt_id = 1;
  if ( ( m_pLLT->RegisterBufferCallback ( (gpointer) &NewProfile, &llt_id ) ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while registering buffer callback!\n";
      exit(0);
  }

  if ( ( m_pLLT->RegisterControlLostCallback ( (gpointer) &ControlLostCallback, NULL ) ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while registering control lost callback!\n";
      exit(0);
  }

  // Start profile transmission
  m_uiProfileBuffer.resize ( m_uiResolution * 64 );
  std::vector<double> ValueX, ValueZ;
  ValueX.resize ( m_uiResolution );
  ValueZ.resize ( m_uiResolution );
  if ( ( iRetValue = m_pLLT->TransferProfiles ( NORMAL_TRANSFER, true ) ) < GENERAL_FUNCTION_OK )
  {
      return iRetValue;
  }

  // start to handle input data
  int msg_count = 0;
  while ( ros::ok() )
  {
    ConvertProfile2Values ( &m_uiProfileBuffer[0], m_uiProfileBuffer.size(), &m_pLLT->appData, m_uiResolution, 0, NULL, NULL, NULL, &ValueX[0], &ValueZ[0], NULL, NULL );
    DisplayProfiles ( &ValueX[0], &ValueZ[0], m_uiResolution );
    ros::spinOnce ();
    msg_count++;
  }

  // Stop transfer
  if ( ( iRetValue = m_pLLT->TransferProfiles( NORMAL_TRANSFER, false ) ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while stopping transmission!\n";
      return iRetValue;
  }

  // Disconnect to sensor
  cout << "Disconnect\n";
  if ( ( iRetValue = m_pLLT->Disconnect() ) < GENERAL_FUNCTION_OK )
  {
      cout << "Error while connecting to camera - Error " << iRetValue << "!\n";
      exit(0);
  }

  return 0;
}

void NewProfile ( const void *pucData, size_t uiSize, gpointer user_data )
{
  if ( pucData != NULL && uiSize == m_uiProfileBuffer.size() && *(int *)user_data == 1 )
  {
    memcpy( &m_uiProfileBuffer[0], pucData, uiSize );
  }
}

void ControlLostCallback( ArvGvDevice *mydevice, gpointer user_data )
{
  // Control of the device is lost. Display a message and force application exit
  if ( mydevice )
  {
      cout << "Control lost\n";
      exit(0);
  }
}

void DisplayProfiles(double *adValueX, double *adValueZ, guint32 uiResolution)
{
  static int msg_counter = 0;
  for ( guint32 i = 1; i < uiResolution; i++ )
  {
      double x = adValueX[i];
      double y = 0;
      double z = adValueZ[i];
  }
  msg_counter++;
  std::cout << msg_counter << ": point_size = " << uiResolution << std::endl;
}
