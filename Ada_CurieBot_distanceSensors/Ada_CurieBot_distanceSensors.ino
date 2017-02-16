/*********************************************************************
 This is CurieBot -- the Arduino 101 based robot
 Autonomous Distance Sensor Variant
 Two IR digital distance sensors connected to pins A0 & A1

 Pick one up today in the Adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M1 & M2 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(1);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(2);

// And connect the Sharp distance sensors
int leftSensor = A0;
int rightSensor = A1;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  pinMode(leftSensor, INPUT); // set up distance sensor pins
  pinMode(rightSensor, INPUT);

  AFMS.begin();  // create with the default frequency 1.6KHz

}

void loop() {
    int readLeft = digitalRead(leftSensor);
  int readRight = digitalRead(rightSensor);

 Serial.print("left: ");
 Serial.print(readLeft);
  Serial.print("           right: ");
 Serial.println(readRight);
 
  L_MOTOR->setSpeed(160); 
  R_MOTOR->setSpeed(160);
  L_MOTOR->run(FORWARD);
  R_MOTOR->run(FORWARD);

  while (digitalRead(rightSensor) == LOW){
    

    L_MOTOR->setSpeed(100); 
    R_MOTOR->setSpeed(100);
    L_MOTOR->run(BACKWARD);
    R_MOTOR->run(RELEASE);
  }

  while (digitalRead(leftSensor) == LOW){
     

    L_MOTOR->setSpeed(100); 
    R_MOTOR->setSpeed(100);
    L_MOTOR->run(RELEASE);
    R_MOTOR->run(BACKWARD);
  }
}
