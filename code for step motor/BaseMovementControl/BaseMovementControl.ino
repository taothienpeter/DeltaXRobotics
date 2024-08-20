#include <util/atomic.h>
#include <AccelStepper.h>
   // Push button for reverse on pin 2, pin 3
#define driverPUL_A 53    // PUL- Apin
#define driverDIR_A 51    // DIR- Apin
#define driverPUL_B 52    // PUL- Bpin
#define driverDIR_B 50    // DIR- Bpin
#define driverEnable_A 49 
#define driverEnable_B 48

#define BAUDRATE 115200

AccelStepper LeftStepper(1, driverPUL_A, driverDIR_A); // (using driver mode, step pin, dir pin)
AccelStepper RightStepper(1, driverPUL_B, driverDIR_B);
// Variables

char chr, cmd;
short arg = 0, arg1, arg2 , index = 0;
int speed_L, speed_R;
char argv1[16], argv2[16];

 // tốc độ động cơ bước (tính bằng step/vòng)
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
  Serial.begin(BAUDRATE);
  step_init();
  
}
void loop() {  
  serial_Read();
  //test();
}

void serial_Read(){
  while (Serial.available() > 0) {
    chr = Serial.read();
    //Serial.println(chr);
    if (chr == 13 || chr == ',' || chr == '\n' || chr == '\r') { 
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      //chạy chương trình 
      runCommand();
      //reset cmd
      resetCommand();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
  }}}
  
    RightStepper.setSpeed(speed_L);
    //Serial.print("ok");
    RightStepper.run();
    LeftStepper.setSpeed(speed_R);
    LeftStepper.run();
}

void resetCommand() {
  
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case 'b':
    Serial.println(BAUDRATE);
    break;
  case 'r':
    speed_L = arg1;
    speed_R = arg2;
    Serial.println("OK"); 
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}