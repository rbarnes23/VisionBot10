/**
 * @ Author: Richard Barnes
 * @ Create Time: 2022-01-29 01:59:44
 * @ Modified by: Richard Barnes
 * @ Modified time: 2022-05-14 14:19:22
 * @ Modified time: 2022-06-28 03:52:13
 */

#include <SPI.h>
#include <common/mavlink.h>
#include "math.h"
#include <vector>
//#include <elapsedMillis.h>
#include <FreeRTOS_TEENSY4.h>
#define USE_TEENSY_HW_SERIAL
//#include <WiFi.h>
//#include <WiFiUdp.h>

#define BTS_7960
// setting PWM properties
const int frequency = 24000;
const int resolution = 8; //Resolution 8(255), 10(1024), 12(4096), 15(32768)
#ifdef W400
#define MOTOR_BRAKE_PIN_L 12
#define MOTOR_DIRECTION_PIN_L 13
#define MOTOR_PWM_PIN_L 2
#define ENCODER_PIN_L 19

#define MOTOR_BRAKE_PIN_R 27
#define MOTOR_DIRECTION_PIN_R 14
#define MOTOR_PWM_PIN_R 23
#define ENCODER_PIN_R 18
#endif

#ifdef BTS_7960
//LEFT MOTOR
#define L_EN_REVERSE 0 //15  GREEN/WHITE
#define L_EN_FORWARD 2//YELLOW/WHITE  SET TO 2 NORMALLY

#define L_PWM_REVERSE 18  //GREEN
#define L_PWM_FORWARD 19  //YELLOW

//RIGHT MOTOR
#define R_EN_REVERSE 13  // Orange/White
#define R_EN_FORWARD 12  // Black/White

#define R_PWM_REVERSE 27 // Orange
#define R_PWM_FORWARD 14  //Black
#endif


//#define DEBUG true
//#define cout Serial
#define GPSSerial Serial1
#define FSYSerial Serial2
#define telem Serial3
const uint8_t SDA_PIN =21;
const uint8_t SCL_PIN =22;


elapsedMillis bootTime;
const int GPSBAUD = 9600;

// MAGNETIC DECLINATION
float MAGDEC = -6.05f;
float VEHICLE_ORIENTATION = -180;
float wp_radius = 1.0f;
float max_speed = 2.0f;

// Helper Fields
int DEADZONE = 0;
uint8_t motor_control = 0;
float val_spd, val_turn;

// Move this to motor control
struct LR
{
  float L;
  float R;
};
struct LR lr;

/**********************************************
Map floating point data
 **********************************************/
float fmap(float sensorValue, float sensorMin, float sensorMax, float outMin, float outMax)
{
  return (sensorValue - sensorMin) * (outMax - outMin) / (sensorMax - sensorMin) + outMin;
}

QueueHandle_t ntripQueue;

// WiFi network name and password:
const char *networkName = "Frontier4720";
const char *networkPswd = "6253020202";

// IP address to send UDP data to:
//  either use the ip address of the server or
//  a network broadcast address
const char *udpAddress = "192.168.254.255";
const int udpPortGC = 14551;
SemaphoreHandle_t udpMutex;

// Are we currently connected?
boolean connected = false;


// The udp library class
//WiFiUDP udpGC;
typedef std::pair<double,double> Point2d;
std::vector<Point2d> _points;
//#include "Geo.h"
//Geo geo(10,1,true);
//#include "SPIFFS.h"
//#include "FS.h"
#include "mavlinkMessaging.h"
#include "gps.h"
//#include "IMUThread.h"
//#include "NtripTask.h"
//#include "FlySky.h"
//#include "Config.h"
//#include "PurePursuit.h"
//#include "stanley.h"
//#include "autopilot.h"



