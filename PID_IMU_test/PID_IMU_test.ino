/*
   AutoPID BasicTempControl Example Sketch

   This program reads a dallas temperature probe as input, potentiometer as setpoint, drives an analog output.
   It lights an LED when the temperature has reached the setpoint.
*/
#include <AutoPID.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
float angle;
long timer = 0;
//pid settings and gains
#define OUTPUT_MIN -10
#define OUTPUT_MAX 10
#define KP .12
#define KI .0003
#define KD 0

double gyro, setPoint, outputVal;

//input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&gyro, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  
  myPID.setTimeStep(10);

}//void setup


void loop() {
  mpu.update();
  setPoint = 0;
  myPID.run(); //call every loop, updates automatically at certain time interval
  gyro = mpu.getAngleZ();
  Serial.print("Gyro: "); Serial.print(gyro); Serial.print("\t");
  Serial.print("output PID:"); Serial.println(outputVal); 
}//void loop
