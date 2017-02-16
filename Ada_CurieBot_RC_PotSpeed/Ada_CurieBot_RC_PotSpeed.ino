/*********************************************************************
This is CurieBot RC -- the Arduino 101 based robot using Bluetooth LE
Potentiometer Speed Control Variant
Pot connected to GND, 3.3V, and pin A0

Modified to drive a 3-wheeled BLE Robot Rover! by http://james.devi.to

 Pick one up today in the Adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

  #include <CurieBLE.h>
  #include <BLEPeripheral.h>
  #include "BLESerial.h"

  BLESerial ble = BLESerial();

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M1 & M2 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(1);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(2);

//not used, testing acceleration
// int accelTime = 200;


#define BLUETOOTH_NAME                 "CurieBot" //Name your RC here
#define BLE_READPACKET_TIMEOUT         500   // Timeout in ms waiting to read a response

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Stream *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

char buf[60];

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

int robotSpeed = 100;

void setup(void)
{
  //while (!Serial);
  AFMS.begin();  // create with the default frequency 1.6KHz

  // turn off both motors
  L_MOTOR->setSpeed(0);
  R_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);
    
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Robot Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialize the module */
  BLEsetup();
  
  L_MOTOR->setSpeed(robotSpeed); 
  R_MOTOR->setSpeed(robotSpeed);
}

int velocity = 0;

float x, y;

int L_restrict = 0;
int R_restrict = 0;

unsigned long lastAccelPacket = 0;

bool modeToggle = false;

void loop(void)
{
  //Set your motor speed
  int reading  = analogRead(A0);
  L_MOTOR->setSpeed(map(reading, 0, 1023, 0, 255)); 
  R_MOTOR->setSpeed(map(reading, 0, 1023, 0, 255));

  // read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;
 
  buttonMode();
}

bool isMoving = false;
unsigned long lastPress = 0;

bool buttonMode(){

   // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';

    Serial.print("Button #"); Serial.print(buttnum);
    if (pressed) 
      Serial.println(" pressed");
    else
      Serial.println(" released");
    //Serial.println(isMoving);
    
    if (pressed) {
      isMoving = true;
      if(buttnum == 1){
        if(robotSpeed <= 245){
          robotSpeed = robotSpeed + 10;
        }

      }
      
      if(buttnum == 2){
        
      }

      if(buttnum == 3){
        if(robotSpeed >=10){
          robotSpeed = robotSpeed - 10;
        }
      }

      if(buttnum == 4){
        
      }
      if(buttnum == 5){
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(FORWARD);
      }
      if(buttnum == 6){
        L_MOTOR->run(BACKWARD);
        R_MOTOR->run(BACKWARD);
      }
      if(buttnum == 7){
        L_MOTOR->run(RELEASE);
        R_MOTOR->run(FORWARD);
      }
      if(buttnum == 8){
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(RELEASE);        
      }

      lastPress = millis();

    } else {
      isMoving = false;

      L_MOTOR->run(RELEASE);
      R_MOTOR->run(RELEASE);
    }
     return true; 
  }

  return false;

}

void BLEsetup() {
  // using Curie
  ble.setLocalName("CurieBot");
  ble.begin();
 
}
