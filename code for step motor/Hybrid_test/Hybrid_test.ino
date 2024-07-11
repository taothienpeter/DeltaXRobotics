#include <AccelStepper.h>
#define driverPUL_A 7    // PUL- Apin
#define driverDIR_A 6    // DIR- Apin
#define driverPUL_B 8    // PUL- Bpin
#define driverDIR_B 9    // DIR- Bpin
#define driverEnable_A 10 
#define driverEnable_B 11

AccelStepper LeftStepper(1, driverPUL_A, driverDIR_A); // (using driver mode, step pin, dir pin)
AccelStepper RightStepper(1, driverPUL_B, driverDIR_B); // đã có khai báo cổng 
unsigned long timer = millis();
void setup() {
  // put your setup code here, to run once:
  LeftStepper.setMaxSpeed(10000);
  LeftStepper.setSpeed(0);
  LeftStepper.setAcceleration(2000);
  //==============
  RightStepper.setMaxSpeed(10000);
  RightStepper.setSpeed(0);
  RightStepper.setAcceleration(2000);
  //======================
  pinMode (driverEnable_A,OUTPUT);
  pinMode (driverEnable_B,OUTPUT);
  digitalWrite(driverPUL_A, LOW);
  digitalWrite(driverDIR_A, LOW);
  digitalWrite(driverPUL_B, LOW);
  digitalWrite(driverDIR_B, LOW);
  digitalWrite(driverEnable_A, LOW);
  digitalWrite(driverEnable_B, LOW);

}

void loop() {
  // timer
  
  if (millis() - timer > 10000){
  digitalWrite(driverEnable_A, HIGH);
  digitalWrite(driverEnable_B, HIGH);};
  
  //==============
  /*
  LeftStepper.setSpeed(4000);
  LeftStepper.runSpeed();
  RightStepper.setSpeed(4000);
  RightStepper.runSpeed();*/

  LeftStepper.move(20000);
  LeftStepper.setSpeed(4000);
  LeftStepper.runSpeedToPosition();
  RightStepper.move(20000);
  RightStepper.setSpeed(4000);
  RightStepper.runSpeedToPosition();
  Serial.println("$")
}
