#define VERSION 1.01
//Modified by Desmartins Daniel 31/08/2024
// Single antenna, IMU code for AgOpenGPS
//
// connection plan:
// Teensy Serial 7 RX (28) to F9P Position receiver TX1 (Position data)
// Teensy Serial 7 TX (29) to F9P Position receiver RX1 (RTCM data for RTK)
//
// Configuration of receiver
// Position F9P
// CFG-RATE-MEAS - 100 ms -> 10 Hz
// CFG-UART1-BAUDRATE 460800
// Serial 1 In - RTCM (Correction Data from AOG)
// Serial 1 Out - NMEA GGA
//
// lansalot's attempt at Keya integration
// (he apologizes in advance)

/************************* User Settings *************************/
// Serial Ports
#define SerialAOG Serial                //AgIO USB conection
#define SerialRTK Serial8               //RTK radio
#define SerialGPS Serial3               //Main postion receiver (GGA) (Serial2 must be used here with T4.0 / Basic Panda boards - Should auto swap)

const int32_t baudAOG = 115200; 
const int32_t baudGPS = 460800;
const int32_t baudRTK = 9600;     // most are using Xbee radios with default of 115200

int8_t KeyaCurrentSensorReading = 0;

#define ImuWire Wire        //SCL=19:A5 SDA=18:A4
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

//Swap BNO08x roll & pitch?
//const bool swapRollPitch = false;

const bool invertRoll= true;  //Used for IMU with dual antenna
#define baseLineLimit 5       //Max CM differance in baseline

#define REPORT_INTERVAL 20    //BNO report time, we want to keep reading it quick & offen. Its not timmed to anything just give constant data.
uint32_t READ_BNO_TIME = 0;   //Used stop BNO data pile up (This version is without resetting BNO everytime)

//Status LED's
//#define GGAReceivedLED 13         //Teensy onboard LED
//#define Power_on_LED 5            //Red
//#define Ethernet_Active_LED 6     //Green
//#define GPSRED_LED 9              //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
//#define GPSGREEN_LED 10           //Green (Flashing = Dual bad, ON = Dual good)
//#define AUTOSTEER_STANDBY_LED 11  //Red
//#define AUTOSTEER_ACTIVE_LED 12   //Green
uint32_t gpsReadyTime = 0;        //Used for GGA timeout

/*****************************************************************/

// Ethernet Options (Teensy 4.1 Only)
#ifdef ARDUINO_TEENSY41
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 1;
};  ConfigIP networkAddress;   //3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = { 0, 0, 0, 0}; //This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy = 5120;             // port of this module
unsigned int AOGNtripPort = 2233;       // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;   // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;    // Port of AOG that listens
char Eth_NTRIP_packetBuffer[512];       // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;     //Out port 5544
EthernetUDP Eth_udpNtrip;     //In port 2233
EthernetUDP Eth_udpAutoSteer; //In & Out Port 8888

IPAddress Eth_ipDestination;
#endif // ARDUINO_TEENSY41

//Speed pulse output
elapsedMillis speedPulseUpdateTimer = 0;
byte velocityPWM_Pin = 36;      // Velocity (MPH speed) PWM pin

#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"

#include <FlexCAN_T4.h>
// CRX2/CTX2 on Teensy are CAN2 on board
// CRX3/CTX3 on Teensy are CAN1 on board
// Seems to work for CAN2, not sure why it didn't for CAN1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_256> K_Bus;    //Tractor / Control Bus
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Keya_Bus;

//Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

// booleans to see if we are using BNO08x
bool useBNO08x = false;

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];    //Extra serial rx buffer

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

bool isTriggered = false;
bool blink = false;

bool Autosteer_running = true; //Auto set off in autosteer setup
bool Ethernet_running = false; //Auto set on in ethernet setup
bool GGA_Available = false;    //Do we have GGA on correct port?

float roll = 0;
float pitch = 0;
float yaw = 0;

// Setup procedure ------------------------
void setup()
{
  delay(500);                         //Small delay so serial can monitor start up
  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);

  delay(10);
  Serial.begin(baudAOG);
  delay(10);
  Serial.println("Firmware : AutoSteer GPS Teensy Engage CAN And Keya for ECU PCB !");
  Serial.print("Version : ");
  Serial.println(VERSION);
  Serial.println("Start setup");

  SerialGPS.begin(baudGPS);
  SerialGPS.addMemoryForRead(GPSrxbuffer, serial_buffer_size);
  SerialGPS.addMemoryForWrite(GPStxbuffer, serial_buffer_size);

  delay(10);
  SerialRTK.begin(baudRTK);
  //SerialRTK->addMemoryForRead(RTKrxbuffer, serial_buffer_size);

  Serial.println("SerialAOG, SerialRTK, SerialGPS initialized");

  Serial.println("\r\nStarting AutoSteer...");
  autosteerSetup();
  
  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();

  Serial.println("\r\nStarting IMU...");
  //test if CMPS working
  uint8_t error;

  ImuWire.begin();
  
  for (int16_t i = 0; i < nrBNO08xAdresses; i++)
  {
      bno08xAddress = bno08xAddresses[i];

      //Serial.print("\r\nChecking for BNO08X on ");
      //Serial.println(bno08xAddress, HEX);
      ImuWire.beginTransmission(bno08xAddress);
      error = ImuWire.endTransmission();

      if (error == 0)
      {
          //Serial.println("Error = 0");
          Serial.print("0x");
          Serial.print(bno08xAddress, HEX);
          Serial.println(" BNO08X Ok.");

          // Initialize BNO080 lib
          if (bno08x.begin(bno08xAddress, ImuWire)) //??? Passing NULL to non pointer argument, remove maybe ???
          {
              //Increase I2C data rate to 400kHz
              ImuWire.setClock(400000); 

              delay(300);

              // Use gameRotationVector and set REPORT_INTERVAL
              bno08x.enableGameRotationVector(REPORT_INTERVAL);
              useBNO08x = true;
          }
          else
          {
              Serial.println("BNO080 not detected at given I2C address.");
          }
      }
      else
      {
          //Serial.println("Error = 4");
          Serial.print("0x");
          Serial.print(bno08xAddress, HEX);
          Serial.println(" BNO08X not Connected or Found");
      }
      if (useBNO08x) break;
  }

  delay(100);
  Serial.print("useBNO08x = ");
  Serial.println(useBNO08x);

  Serial.println("Right... time for some CANBUS! And, we're dedicated to Keya here");
  CAN_Setup();

  Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
}

void loop()
{
    KeyaBus_Receive();

    // Read incoming nmea from GPS
    if (SerialGPS.available())
    {
            parser << SerialGPS.read();
    }

    udpNtrip();

    // Check for RTK Radio
    if (SerialRTK.available())
    {
        SerialGPS.write(SerialRTK.read());
    }

    //GGA timeout, turn off GPS LED's etc
    if((systick_millis_count - gpsReadyTime) > 10000) //GGA age over 10sec
    {
      //digitalWrite(GPSRED_LED, LOW);
      //digitalWrite(GPSGREEN_LED, LOW);
    }

    //Read BNO
    if((systick_millis_count - READ_BNO_TIME) > REPORT_INTERVAL && useBNO08x)
    {
      READ_BNO_TIME = systick_millis_count;
      readBNO();
    }
    
    if (Autosteer_running) autosteerLoop();
    else ReceiveUdp();
    
  if (Ethernet.linkStatus() == LinkOFF) 
  {
    //digitalWrite(Power_on_LED, 1);
    //digitalWrite(Ethernet_Active_LED, 0);
  }
  if (Ethernet.linkStatus() == LinkON) 
  {
    //digitalWrite(Power_on_LED, 0);
    //digitalWrite(Ethernet_Active_LED, 1);
  }
}//End Loop
//**************************************************************************
