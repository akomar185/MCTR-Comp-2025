#include <MPU6050_tockn.h>

#include <Wire.h>


#include <Arduino.h>
#include <IRremote.hpp>
#include <Arduino_JSON.h>

#include <Servo.h> // servo dependencies


using namespace std;

//IR reciever pinout and key-setup
const int RECV_PIN = A0;
JSONVar irmap;

// Motor driver pinout
const int ena = 11;
const int enb = 10;

const int in1 = 30; // try switching these to see if error persists? is so, hardware fs
const int in2 = 32;
const int in3 = 34;
const int in4 = 36;

//GYRO Setup and Pinout
// const int SDA1 = 20;
// const int SCL1 = 21;
MPU6050 mpu(Wire);

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// movement variables
int prev_key = 0;
int pause = 0;
int key;
int turn_counter;

int challenge_num = 0;

//SERVO SETUP
Servo claw;
Servo lift;

//ULTRASONIC SENSOR SETUP

// Define Trig and Echo pins.
const int TRIG_PIN = 50;
const int ECHO_PIN = 48;

void setup() {
  Serial.begin(9600);
  while (!Serial){
    delay(20);
  }

  Serial.println("Setup started!");

  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  Serial.println("Made it after mpu.begin");


  
  // ir reciever pinout setup
  pinMode(RECV_PIN, INPUT);
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);
  irmap["3910598400"] = 0;
  irmap["4077715200"] = 1;
  irmap["3877175040"] = 2;
  irmap["2707357440"] = 3;
  irmap["4144561920"] = 4;
  irmap["3810328320"] = 5;
  irmap["2774204160"] = 6;
  irmap["3175284480"] = 7;
  irmap["2907897600"] = 8;
  irmap["3041591040"] = 9;
  irmap["0"] = 99;

  // motor driver pinout setup
  pinMode(ena, OUTPUT); // pwm
  pinMode(enb, OUTPUT); // pwm

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.println("Setup finsihed!");

  // servo motor setup
  claw.attach(22)
  lift.attach(24)

  //ULTRASONIC SENSOR SETUP
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // catch IR signal and decide what to do

  mpu.update();
  Serial.print("GyroX: "); Serial.print(mpu.getGyroX());
  Serial.print(" | GyroY: "); Serial.print(mpu.getGyroY());
  Serial.print(" | GyroZ: "); Serial.println(mpu.getGyroZ());
  delay(500);


  if (pause >= 4) {
    key = -1;
  }
  pause ++;
  if (IrReceiver.decode()) {
    String str = String(IrReceiver.decodedIRData.decodedRawData);
    key = irmap[str];
    pause = 0;
    
    IrReceiver.resume(); // Enable receiving of the next value
  } else {
    delay(50);
  }
  // Serial.println(key);
  move(key);

  
  // challenge specific code
  switch (challenge_num) {
    case 1: challengeOne(); break;
    case 2: challengeTwo(); break;
    case 3: challengeThree(); break;
  }

  int distanceCentimeters = getDistanceCentimeters();


  // Print the distance on the Serial Monitor.
  Serial.print("Distance = ");
  Serial.print(distanceCentimeters);
  Serial.println(" cm");


  delay(50);
}

// movement functions
void move(int key) {
  if (key == 99 && (prev_key == 7 || prev_key == 9)) {
    turn_counter ++;
    if (turn_counter >= 1) {
      return;
    }
  } else if (key == 99) {
    key = prev_key;
  }
  turn_counter = 0;
  prev_key = key;
  switch (key) {
    case 5: forwards(255); break;
    case 7: left(255); break;
    case 8: backwards(255); break;
    case 9: right(255); break;
    default: stop(); break;
  }
}

void stop() {
  analogWrite(ena, 0);
  analogWrite(enb, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void forwards(int speed) {
  Serial.println("Going forwards?!");
  analogWrite(ena, speed);
  analogWrite(enb, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backwards(int speed) {
  Serial.println("Going back?!");

  analogWrite(ena, speed);
  analogWrite(enb, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void left(int speed) {
  Serial.println("Going left?!");

  analogWrite(ena, speed);
  analogWrite(enb, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
}

void right(int speed) {
  Serial.println("Going roight?!");

  analogWrite(ena, speed);
  analogWrite(enb, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
}

//function to convert the signal from the ultrasonic sensor to a distance value
int getDistanceCentimeters() {
  // Clear the trigPin by setting it LOW.
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);


  // Trigger the sensor by setting the trigPin high for 10 microseconds.
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);


  // Read the echoPin, pulseIn() returns the duration (length of the pulse) in microseconds.
  long durationMicroseconds = pulseIn(ECHO_PIN, HIGH);
  // Calculate the distance in centimeters. Note that at 20 degrees Celcius, the speed of sound
  // is roughly 0.034 cm / us.
  int distanceCentimeters = durationMicroseconds * 0.034 / 2;


  return distanceCentimeters;
}

// challenge operating stuff
void challengeOne() {
  //Get gyro data

}

void challengeTwo() {
  return;
}

void challengeThree() {
  return;
}