#include <MPU6050.h>

#include <Servo.h> // servo dependencies


#include <Wire.h>

#include <Arduino.h>
#include <IRremote.hpp>
#include <Arduino_JSON.h>

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
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t theta;


//SERVO SETUP
Servo claw;
Servo lift;

  

//ULTRASONIC SENSOR SETUP

// Define Trig and Echo pins.
const int TRIG_PIN = 50;
const int ECHO_PIN = 48;

// movement variables
int prev_key = 0;
int pause = 0;
int key;
int turn_counter;

int challenge_num = 0;

void setup() {

  Serial.begin(9600);
  Wire.begin();
  //0 is closed
  claw.attach(3);
  claw.write(0);
  delay(1000);
  claw.write(45);
  delay(1000);
  claw.write(0);
  delay(1000);


  Serial.println("started");
  mpu.initialize();
  if(mpu.testConnection()){
    Serial.println("connected");
  } else {
    Serial.println("not successfull");
  }
  
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

  // servo motor setup
  claw.attach(22);
  lift.attach(24);

  //ULTRASONIC SENSOR SETUP
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  // mpu offsets
  Serial.println("Updating internal sensor offsets...\n");
  mpu.setXAccelOffset(0); //Set your accelerometer offset for axis X
  mpu.setYAccelOffset(0); //Set your accelerometer offset for axis Y
  mpu.setZAccelOffset(0); //Set your accelerometer offset for axis Z
  mpu.setXGyroOffset(0);  //Set your gyro offset for axis X
  mpu.setYGyroOffset(0);  //Set your gyro offset for axis Y
  mpu.setZGyroOffset(0);  //Set your gyro offset for axis Z
  
  /*Print the defined offsets*/
  // Serial.print("\t");
  // Serial.print(mpu.getXAccelOffset());
  // Serial.print("\t");
  // Serial.print(mpu.getYAccelOffset()); 
  // Serial.print("\t");
  // Serial.print(mpu.getZAccelOffset());
  // Serial.print("\t");
  // Serial.print(mpu.getXGyroOffset()); 
  // Serial.print("\t");
  // Serial.print(mpu.getYGyroOffset());
  // Serial.print("\t");
  // Serial.print(mpu.getZGyroOffset());
  // Serial.print("\n");
}

void loop() {
  // catch IR signal and decide what to do

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



  int distanceCentimeters = getDistanceCentimeters();


  // Print the distance on the Serial Monitor.
  // Serial.print("Distance = ");
  // Serial.print(distanceCentimeters);
  // Serial.println(" cm");

  //mpu stuff
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
  
  theta = atan(ay/sqrt(ax^2+az^2));
  Serial.println(theta);

  
  
  delay(100);
  
  // challenge specific code
  switch (challenge_num) {
    case 1: challengeOne(); break;
    case 2: challengeTwo(); break;
    case 3: challengeThree(); break;
  }

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