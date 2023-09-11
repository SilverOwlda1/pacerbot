#include <Wire.h>



float miles = 1; // miles
float trackTime = 8.0; // minutes

//in DeviceDriverSet_xxx0.h
//in DeviceDriverSet_xxx0.h
/*ITR20001 Detection*/
#define MPU 0x68
#define lineTracker_L A2
#define lineTracker_M A1
#define lineTracker_R A0

int servoPin = 10;
int ena = 5;
int enb = 11;
int in1 = 8;
int in2 = 2;
int in3 = 7;
int in4 = 0;


float AccelXBias = 0.0;
float AccelYBias = 0.0;
float meanLineBrightness = 700;
int motorInput = 0;
int servoInput = map(90, 0, 180, 0, 255);
float desiredVelocity = 0.0;

void setup() {

  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);  // put your setup code here, to run once:
  float rpm = (26.8224 * miles / trackTime * 1000.0 / 29.0) / (2 * 3.14159) * 60.0;
  desiredVelocity = rpm * 2 * 3.14159 / 60.0 * 29.0;

  motorInput = map(rpm, 0, 15540, 0, 255);

  pinMode(servoPin, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(lineTracker_L, INPUT);
  pinMode(lineTracker_M, INPUT);
  pinMode(lineTracker_R, INPUT);

  calibrateCar();
  delay(5000);
}

float t = millis();
float velocityX = 0.0;
float velocityY = 0.0;
float velocity = 0.0;
float distance = 0.0;
int L = 0.0;
int M = 0.0;
int R = 0.0;
int direction = 0;

void loop() {

  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  float AccX = 9.81 * (Wire.read() << 8 | Wire.read()) / 16384.0 - AccelXBias; // X-axis value
  float AccY = 9.81 * (Wire.read() << 8 | Wire.read()) / 16384.0 - AccelYBias; // Y-axis value// Z-axis value

  float acceleration = pow((pow(AccX, 2) + pow(AccY, 2)), 0.5);
  float dt = (millis() - t) / 1000.0;
  velocityX += AccX * dt;
  velocityY += AccY * dt;
  distance += velocity * dt;
  velocity = pow(pow(velocityX, 2) + pow(velocityY, 2), 0.5);
  correctMotorInput();
  

  L = readLineTracker_L()*0.8 + L*0.2;
  M = readLineTracker_M()*0.8 + M*0.2;
  R = readLineTracker_R()*0.8 + R*0.2;

  Serial.print(L);
  Serial.print(" ");
  Serial.print(M);

  Serial.print(" ");
  Serial.print(R);
  Serial.println();

  correctSteering(L, M, R, 3);

  // Serial.println(angle);


  t = millis();

  //Serial.print("Left = ");
  //Serial.print(readLineTracker_L());
  //Serial.print(", Middle = ");
  //.print(readLineTracker_M());
  //Serial.print(", Right = ");
  //Serial.print(readLineTracker_R());
  //Serial.println();
}

// runs in setup function to calculate biases of line tracker and accelerometer sensors
void calibrateCar() {
  int sampleSize = 0;
  int ltLBias = 0;
  int ltMBias = 0;
  int ltRBias = 0;
  int sum = 0;

  while (sampleSize < 5) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccelXBias += 9.81 * (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    AccelYBias += 9.81 * (Wire.read() << 8 | Wire.read()) / 16384.0;

    sum += readLineTracker_L();
    sum += readLineTracker_M();
    sum += readLineTracker_R();
    Serial.println(sum);
    delay(1);

    sampleSize += 3;
  }

  AccelXBias /= sampleSize;
  AccelYBias /= sampleSize;
  meanLineBrightness = (sum+0.0)/sampleSize;

}

void correctMotorInput(float velocity) {
  motorInput += (int) 5 * (desiredVelocity - velocity);
}

int contrast = 300;

void correctSteering(int L, int M, int R, float velocity) {
  if (abs(L - meanLineBrightness) > contrast && abs(R - meanLineBrightness) < contrast) {
    int angle = map(velocity, 0, 5.364, 0, 70);
    //angle += 90;
    Serial.println("turn right");
    //return angle;
    servoInput = map(angle, 0, 180, 0, 255);
    
  }
  if (abs(R - meanLineBrightness) > contrast && abs(L - meanLineBrightness) < contrast) {
    int angle = map(velocity, 0, 5.364, 0, 70);
   // angle += 90;
    servoInput = map(angle+90, 0, 180, 0, 255);
    Serial.println("turn left");
    
    //return angle;
  }
  if (abs(L - meanLineBrightness) < contrast && abs(R - meanLineBrightness) < contrast && abs(M - meanLineBrightness) < contrast) {
    servoInput = map(90, 0, 180, 0, 255);
    //return 0;
    Serial.println("don't turn");
  } 
  
  analogWrite(servoPin, servoInput);
  //Serial.println("bruh");
  Serial.print("brightness = ");
  Serial.println(meanLineBrightness);

}

//in DeviceDriverSet_xxx0.cpp
float readLineTracker_L() {
  return analogRead(lineTracker_L);
}

float readLineTracker_M() {
  return analogRead(lineTracker_M);
}

float readLineTracker_R() {
  return analogRead(lineTracker_R);
}
