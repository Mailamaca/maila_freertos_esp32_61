#include <driver/spi_master.h>
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include <math.h>


////////////////////////////////////////////////
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};              // vector to hold integral error for Mahony method
float deltat = 0.0f;                             // integration interval for both filter schemes

float accelScale;
float gyroScale;
float magScaleX, magScaleY, magScaleZ;


#define Kp 7.50f
// #define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 1.7f
uint8_t txData[2] = { };
uint8_t rxData[21] = { };


void setupIMU(int pinCLK,
              int pinMISO,
              int pinMOSI,
              int pinCS);

float _ax, _ay, _az;

void updateIMU(TickType_t TimePast);
  
///////////////////////////////////////////
void fReadMPU9250 ( uint8_t byteReadSize, uint8_t addressToRead );
///////////////////////////////////////////////////////////////////////
void fWriteSPIdata8bits( uint8_t address, uint8_t DataToSend);
///////////////////////////////////////////////////////////////////////
void fReadAK8963( uint8_t subAddress, uint8_t count );
////////////////////////////////////////////////////////////////////////////////////////////
void fWrite_AK8963 ( uint8_t subAddress, uint8_t dataAK8963 );
/////////////////////////////////////////////////////////////////////////////////////////////
void fwriteMUP9250register ( uint8_t addr, uint8_t sendData );
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
//////////////////////////////////////////////
