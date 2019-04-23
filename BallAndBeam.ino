/*
 * Benen Adsitt, Bruce Baker, Matthew Skipworth 
 * TCES 455
 * PID Controller for the Ball and Beam Lab
 * December 13, 2018
 */

#include <Wire.h>

#include "SparkFun_VL53L1X_Arduino_Library.h"
VL53L1X distanceSensor;

int servoPin = 7;     // Control pin for servo motor

int pulseWidth = 0;    // Amount to pulse the servo
long lastPulse = 0;    // the time in millisecs of the last pulse
int refreshTime = 20;  // the time in millisecs needed in between pulses

int minPulse = 600;   // minimum pulse width

unsigned int currentTime = 0;
unsigned int pastTime = 0;

//!!! IMPORTANT !!! R NEEDS TO BE IN TERMS OF METERS ALL THE TIME
double r = 0;  //This is the initial position of the ball
double prevR = 0;  //last position of the ball
int balanceD = 250;  //This is the distance at which the ball is considered balanced.
double deltaT = 50;  //Time between steps.  50ms between steps is 20 times per second.


double Kp = 17.1146;

double KpTerm = 0;

double Ki = 1.3044;
double sum = 0;
double maxSum =7;  //Find this one with simulink or something
double KiTerm = 0;

double Kd = 56.1378;
double alpha = 0.2;  //Find an alpha between 0 and 1
double yLast = 0;
double KdTerm = 0;
void setup() {
 Wire.begin();
  pinMode(servoPin, OUTPUT);  // Set servo pin as an output pin
  pulseWidth = minPulse;      // Set the motor position to the minimum
  Serial.begin(9600);         // connect to the serial port
  Serial.println("Servo Serial Better ready");
  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() == false)
    Serial.println("Sensor offline!");
}


void loop() {
  // put your main code here, to run repeatedly:
  pastTime = millis();
  while (distanceSensor.newDataReady() == false)
    delay(1);
  double distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  r = (distance - balanceD)/1000; //GET THIS BUNK INTO MILLIMETERS
//  Serial.print("Kp Term: ");
//  Serial.print(KpTerm);
//  Serial.print("  Ki Term: ");
//  Serial.print(KiTerm, 10);
//  Serial.print("  Kd Term: ");
//  Serial.print(KdTerm);
//  Serial.println();
  PIDtheFool();
  Serial.print("Controller Output Term: ");
  double contTerm = KpTerm + KiTerm + KdTerm;
  double total = r + contTerm;

  
  //gotta calculate a pulsewidth now for the servo.
  pulseWidth = -174 * total + 1470;
  Serial.print(pulseWidth);
  Serial.println();
  if(pulseWidth < 600) {
    pulseWidth = 600;
  } else if(pulseWidth > 2400) {
    pulseWidth = 2400;
  }
  updateServo();
  currentTime = millis();
  
  while(currentTime - pastTime < 50){
    currentTime = millis();
    updateServo();
  }
}
void PIDtheFool() {
  //kpTerm
  KpTerm = r * Kp;

    //Integral term
  double tempTime = deltaT/1000; // mS to Second conversion
  sum = sum + r * tempTime;
  sum = min( max(sum, -maxSum), maxSum); //limits the size of the sum
  KiTerm = sum * Ki;
  
  //Derivative term
  
  double yNow = alpha * r + (1 - alpha) * yLast;
  KdTerm = (yNow - yLast)/(tempTime) * Kd;
  yLast = yNow; 
}

void updateServo() {
  // pulse the servo again if rhe refresh time (20 ms) have passed:
  if (millis() - lastPulse >= refreshTime) {
    digitalWrite(servoPin, HIGH);   // Turn the motor on
    delayMicroseconds(pulseWidth);  // Length of the pulse sets the motor position
    digitalWrite(servoPin, LOW);    // Turn the motor off
    lastPulse = millis();           // save the time of the last pulse
  }
}
