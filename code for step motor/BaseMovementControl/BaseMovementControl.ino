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

class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
    // Constructor
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

    // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
    // derivative
    float dedt = (e-eprev)/(deltaT);
    // integral
    eintegral = eintegral + e*deltaT;
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
    // store previous error
    eprev = e;
  }
};
//some variable:
float maxSpeed = 20000;
float accceleration = 2000;

String receivedData, stringPart, intPart1, intPart2;
int pulse, time;

int periodDelay = 500;       // Pulse Delay period (50-2000)
boolean setdir = LOW; // Set Direction

// Khai báo thời gian delay cho các xung PWM (điều chỉnh tốc độ)
unsigned long delayTime1 = 500; // Thời gian delay cho động cơ 1 (micro giây)
unsigned long delayTime2 = 300; // Thời gian delay cho động cơ 2 (micro giây)

// Khai báo biến thời gian để theo dõi thời gian tạo xung PWM
unsigned long previousMicros1 = 0;
unsigned long previousMicros2 = 0;
bool pulState1 = LOW;
bool pulState2 = LOW;

unsigned long looptime;
void setup() {
  Serial.begin(9600);

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
  /*
  while (!Serial) {
    ; // Đợi cho đến khi Serial kết nối
  }
  Serial.println("Serial communication started.");
  Serial.println("Commands: <cmd> <pulse> <time>");
  /*
    cmd:forw, back, right, left
    pulse: pulse/micro second
    time: second 
  */
}

void loop() {    
  serialRead();
  //Serial.println("$");
  /*
//====================================
      unsigned long startTime = millis();
      unsigned long Rtime;
    if(stringPart == "forw"){
      while(Rtime < time){
       Rtime = millis() - startTime;
       setMotorSpeed(pulse, pulse);
       setMotorDir(1);//1 = forward
      }
    }
    if(stringPart == "back"){
      while(Rtime < time){
       Rtime = millis() - startTime;
       setMotorSpeed(pulse, pulse);
       setMotorDir(-1);//-1 = backward
      }
    }

    
    if(stringPart == "right"){
      while(Rtime < time){
       Rtime = millis() - startTime;
       setMotorSpeed(pulse, pulse);
       setMotorDir(2);//2 = right  <=============sửa thành right
      }
    }

    if(stringPart == "left"){
      while(Rtime < time){
       Rtime = millis() - startTime;
       setMotorSpeed(pulse, pulse);
       setMotorDir(-2);//-2 = left  <=============sửa thành left
      }
    }    
    */
}

//=============================================
void serialRead(){
    // read input
    
  if (Serial.available() > 0) {
    receivedData = Serial.readStringUntil('\n'); // Đọc đến khi gặp ký tự xuống dòng
    Serial.print("Received: ");
    Serial.println(receivedData);

    int firstSpaceIndex = receivedData.indexOf(' ');
    int secondSpaceIndex = receivedData.indexOf(' ', firstSpaceIndex + 1);

    if (firstSpaceIndex != -1 && secondSpaceIndex != -1) {
      stringPart = receivedData.substring(0, firstSpaceIndex);
      intPart1 = receivedData.substring(firstSpaceIndex + 1, secondSpaceIndex);
      intPart2 = receivedData.substring(secondSpaceIndex + 1);

      pulse = intPart1.toInt();
      time = intPart2.toInt();
      Serial.print(pulse); Serial.println(time);
    }
  }
}
void setMotorSpeed (int speed_A, int speed_B){ //pulse/micro second
  RightStepper.setSpeed(speed_A);
  RightStepper.run();
  LeftStepper.setSpeed(speed_B);
  LeftStepper.run();
}
