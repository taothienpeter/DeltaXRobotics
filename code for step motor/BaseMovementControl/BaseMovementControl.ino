#include <util/atomic.h>
#include <AccelStepper.h>
   // Push button for reverse on pin 2, pin 3
#define driverPUL_A 53    // PUL- Apin
#define driverDIR_A 51    // DIR- Apin
#define driverPUL_B 52    // PUL- Bpin
#define driverDIR_B 50    // DIR- Bpin
#define driverEnable_A 49 
#define driverEnable_B 48

AccelStepper LeftStepper(1, driverPUL_A, driverDIR_A); // (using driver mode, step pin, dir pin)
AccelStepper RightStepper(1, driverPUL_B, driverDIR_B);
// Variables
char argv1[16];
char argv2[16];
short index;

float speed_L = 1600, speed_R = 1600; // tốc độ động cơ bước (tính bằng step/vòng)
float* pSpeed_L = &speed_L;
float* pSpeed_R = &speed_R;
//some variable:
float maxSpeed = 1600;
float accceleration = 0;
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
  Serial.begin(115200);
  step_init();
  
}
void loop() {  
  serial_Read();
  //test();
}

void serial_Read(){ // đọc thông tin gửi qua serial port

  if (Serial.available() > 0) {// read input
    
    speed_L = Serial.readBytesUntil(',', char);
    speed_R = Serial.parseInt();
    Serial.println(speed_L);
    Serial.println(speed_R);
  }
  else{
    RightStepper.setSpeed(speed_L);
    //Serial.print("ok");
    RightStepper.run();
    LeftStepper.setSpeed(speed_R);
    LeftStepper.run();}
}
void test(){
    RightStepper.setSpeed(speed_L);
    RightStepper.run();
    LeftStepper.setSpeed(speed_R);
    LeftStepper.run();
}
void feedback(){
  Serial.write("OK");
}


