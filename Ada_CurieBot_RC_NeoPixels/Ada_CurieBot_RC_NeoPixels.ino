
/*********************************************************************
 This is CurieBot RC -- the Arduino 101 based robot using Bluetooth LE
 NeoPixel Ground Effects Variant
 NeoPixel 24 ring mounted under the robot, connected to pin 6
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
#include <Adafruit_NeoPixel.h>


#define NEOPIN 6
#define NUMPIXELS 24
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, NEOPIN, NEO_GRBW + NEO_KHZ800 );


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

//NeoPixel setup
#define NEOPIN 6
#define NUMPIXELS 24

void setup(void) {  
  Serial.begin(9600);
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

  //setup the neopixel ring
  pixel.begin();
  pixel.setBrightness(128); //medium brightness
  pixel.show();

}

int velocity = 0;

float x, y;

int L_restrict = 0;
int R_restrict = 0;

unsigned long lastAccelPacket = 0;

bool modeToggle = false;

void loop(void) {



  // read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

//color picker for NeoPixel ring
  if(packetbuffer[1]== 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    for(int i=0; i<NUMPIXELS;i++){
      pixel.setPixelColor(i, pixel.Color(red,green,blue));
    }
    pixel.show();
  }

  // Read from Accelerometer input
  if( accelMode() ) {
    lastAccelPacket = millis();
    modeToggle = true;
    return;
  }

  // Stop motors if accelerometer data is turned off (100ms timeout)
  if(((millis() - lastAccelPacket) > 100) & modeToggle) {
    L_MOTOR->run(RELEASE);
    R_MOTOR->run(RELEASE);
    modeToggle = false;
    return;
  }

  //if no accelerometer, use control pad
  if( !modeToggle ) {
    buttonMode();
  } 
}


bool accelMode(){
  if (packetbuffer[1] == 'A') {
          x = parsefloat( packetbuffer + 2 );
          y = parsefloat( packetbuffer + 6 );

        if( x <= -0.55 ){
          x += 0.55;
          x *= -100.0;
          L_MOTOR->run( BACKWARD );
          R_MOTOR->run( BACKWARD );
          if( x >= 45 ) x = 45;
          if( x <= 0 ) x = 0;
          velocity = map( x, 0, 45, 0 ,255 );
        }
        else if( x >= -0.25 ){
          x+= 0.25;
          x *= 100;
          L_MOTOR->run( FORWARD );
          R_MOTOR->run( FORWARD );
          if( x>= 45 ) x = 45;
          if( x<= 0 ) x = 0;
          velocity = map( x, 0, 45, 0, 255 );
        }
        else{
          L_MOTOR->run( RELEASE );
          R_MOTOR->run( RELEASE );
          velocity = 0;
        }

        //account for L / R accel

        if( y >= 0.1 ){
            y -= 0.1;
            y *= 100;
            if( y >= 50 ) y = 50;
            if( y <= 0 ) y = 0;

            L_restrict = fscale( y, 0.0, 50.0, 0.0, 100.0, -4.0 );
        }
        else if( y <= -0.1 ){
            y += 0.1;
            y *= -100;
            if( y>= 50 ) y = 50;
            if( y<= 0 ) y = 0;

            R_restrict = fscale( y, 0.0, 50.0, 0.0, 100.0, -4.0 );
        }
        else{
            L_restrict = 0;
            R_restrict = 0;
        }

          float Lpercent = ( 100.0 - L_restrict ) / 100.00 ;
          float Rpercent = ( 100.0 - R_restrict ) / 100.00 ;

          // Serial.print( x ); 
          // Serial.print( "\t" ); 
          // Serial.print( Lpercent ); 
          // Serial.print( "\t" ); 
          // Serial.print( velocity ); 
          // Serial.print( "\t" ); 
          // Serial.println( Rpercent ); 

          L_MOTOR->setSpeed( velocity * Lpercent ); 
          R_MOTOR->setSpeed( velocity * Rpercent ); 

          return true;
    }
    return false;
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


      if (buttnum ==3){
        

        
      }
       if (buttnum ==4){
        

        
      }
      
      lastPress = millis();
      
      // speed up the motors
      for (int speed=0; speed < 155; speed+=5) {//255
        L_MOTOR->setSpeed(speed);
        R_MOTOR->setSpeed(speed);
        delay(5); // 250ms total to speed up
      }
    } else {
      isMoving = false;
      
      // slow down the motors
      for (int speed=155; speed >= 0; speed-=5) {//255
        L_MOTOR->setSpeed(speed);
        R_MOTOR->setSpeed(speed);
        delay(5); // 50ms total to slow down
      }
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

//Logarithmic mapping function from http://playground.arduino.cc/Main/Fscale
float fscale( float inputValue,  float originalMin, float originalMax, float newBegin, float newEnd, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println(); 
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){ 
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd; 
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);  
   Serial.print("   ");  
   Serial.print(NewRange, DEC);  
   Serial.print("   ");  
   Serial.println(zeroRefCurVal, DEC);  
   Serial.println();  
   */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {   
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
  }

  return rangedValue;
}
