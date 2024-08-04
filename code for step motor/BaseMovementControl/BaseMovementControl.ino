#include <util/atomic.h>
#include <AccelStepper.h>
   // Push button for reverse on pin 2, pin 3
#define driverPUL_A 7    // PUL- Apin
#define driverDIR_A 6    // DIR- Apin
#define driverPUL_B 8    // PUL- Bpin
#define driverDIR_B 9    // DIR- Bpin
#define driverEnable_A 10 
#define driverEnable_B 11

AccelStepper LeftStepper(1, driverPUL_A, driverDIR_A); // (using driver mode, step pin, dir pin)
AccelStepper RightStepper(1, driverPUL_B, driverDIR_B);
// Variables
float speed_linear, speed_angular; // tốc độ trung bình cho tịnh tiến, và rẽ
int speed_L, speed_R; // tốc độ động cơ bước (tính bằng step/vòng)
//some variable:
float maxSpeed = 20000;
float accceleration = 2000;
void step_init(){
  LeftStepper.setMaxSpeed(maxSpeed);
  LeftStepper.setSpeed(0);
  LeftStepper.setAcceleration(accceleration);
  RightStepper.setMaxSpeed(maxSpeed);
  RightStepper.setSpeed(0);
  RightStepper.setAcceleration(accceleration);
  //================
  pinMode (driverPUL_A, OUTPUT);
  pinMode (driverDIR_A, OUTPUT);
  pinMode (driverPUL_B, OUTPUT);
  pinMode (driverDIR_B, OUTPUT);
  pinMode (driverEnable_A,OUTPUT);
  pinMode (driverEnable_B,OUTPUT);
  digitalWrite(driverEnable_A, LOW);
  digitalWrite(driverEnable_B, LOW);
  digitalWrite(driverDIR_A, LOW);
  digitalWrite(driverDIR_B, LOW);
}

void setup() {
  Serial.begin(9600);
  step_init();
  
}
void loop() {  
  serial_Read();
  motor_Speed_Set();
}

void serial_Read(){ // đọc thông tin gửi qua serial port
   if (Serial.available() > 0) {// read input
    String command = Serial.readStringUntil('\n');
    speed_L = command.substring(0, command.indexOf(',')).toFloat();
    speed_R = command.substring(command.indexOf(',') + 1).toFloat();}
}
void motor_Speed_Set (){ //pulse/micro second
  RightStepper.setSpeed(speed_L);
  RightStepper.run();
  LeftStepper.setSpeed(speed_R);
  LeftStepper.run();
}


