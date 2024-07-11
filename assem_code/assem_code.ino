 /*the closed-loop feedback control can be achieved with MPU6050 
-----------  A5 - SCL ;  A4 - SDA; interupt 2;
*/
#include <AccelStepper.h>
#include "Wire.h"
#include <MPU6050_light.h>

#include <AutoPID.h>

#define Stepper1Pulse 7
#define Stepper1Direction 6
#define Stepper2Pulse 8
#define Stepper2Direction 9

#define OUTPUT_MIN -2
#define OUTPUT_MAX 2
#define KP .12
#define KI .0003
#define KD 0

//khai báo cho PID
  double gyro, setPoint, outputVal;
  AutoPID myPID(&gyro, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
//khai báo cho MPU
  MPU6050 mpu(Wire);
  float angle;
  long timer = 0;
//khai báo cho hàm motor
  int highspeed=255;
  int lowspeed=10;
  int mySpeed = 200;   // speed
  
  int corrSpeedL;       //Left Motor
  int corrSpeedR;       //Right Motor
  unsigned long t1 = 0;
  unsigned long t2 = 0;
// khai báo cho stepmotor
  int Motor1speed = 4000;
  int Motor2speed = 4000;
  int speedmin = 0; //pulses per second
  int speedmax = 4000;  //pulses per second
  AccelStepper step1(1, Stepper1Pulse, Stepper1Direction);
  AccelStepper step2(1, Stepper2Pulse, Stepper2Direction);
  
void setup() {
  Serial.begin(9600);
  Wire.begin();
  myPID.setTimeStep(10);
//khai báo step
  step1.setMaxSpeed (speedmax);  
  step1.setSpeed(0);
  step1.setAcceleration(1000);
  step2.setMaxSpeed (speedmax);  
  step2.setSpeed(0);
  step2.setAcceleration(1000);
  pinMode(Stepper1Pulse, OUTPUT);
  pinMode(Stepper1Direction, OUTPUT);
  pinMode(Stepper2Pulse, OUTPUT);
  pinMode(Stepper2Direction, OUTPUT);
  digitalWrite(Stepper1Pulse, LOW);
  digitalWrite(Stepper1Direction, LOW);
  digitalWrite(Stepper2Pulse, LOW);
  digitalWrite(Stepper2Direction, LOW);
//khai báo và cali MPU
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ }
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}
void loop() {
  //safety timer
  if (millis() - timer > 10000){
    digitalWrite(driverEnable_A, HIGH);
    digitalWrite(driverEnable_B, HIGH);};
  
  //============
  mpu.update();
  setPoint = 0;
  myPID.run();
  gyro = mpu.getAngleZ();
  
  if (millis() - timer2 > 1000){}
  Serial.print("Gyro: "); Serial.print(gyro); Serial.print("\t");
  Serial.print("output PID:"); Serial.println(outputVal);
  
  }
}
void driveStraight(int speed){// thêm khoản thời gian để gyro đọc 
  static unsigned long onTime;
   //to compute corrSpeed(A/B) The yaw data is acquired at the first startup, and the data is updated every ten millisecundes.
  if (millis() - onTime > 10){
  corrSpeed(speed);
  onTime = millis();  
 }  
}  
void corrSpeed(int myspeed){// calculate each motor speed
  calcYaw();
  int  kp= 15; ////Add proportional constant - p( ???)
  //if drive direktion changes: corrSpeedA = mySpeedA - (yawOld - yaw) * kp;
  corrSpeedR = mySpeed + (yaw-yawOld)*kp; //maintain speed by speeding up right motor
  corrSpeedL = mySpeed - (yaw-yawOld)*kp; //if error is positive, slow down left motor
  //======
  if (corrSpeedR > highspeed)corrSpeedR = highspeed;
    else if (corrSpeedR < lowspeed)corrSpeedR = lowspeed;
  //if drive direktion changes:corrSpeedB = mySpeedB + (yawOld - yaw) * kp;
  if (corrSpeedL > highspeed)corrSpeedL = highspeed;
    else if (corrSpeedL < lowspeed)corrSpeedL = lowspeed;
 }


  