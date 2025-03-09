
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>
#include <Servo.h> // servo dependencies


#include <Wire.h>

#include <Arduino.h>
#include <IRremote.hpp>
#include <Arduino_JSON.h>

// buzzer
const int buzzer = 13;
int note = 0;

using namespace std;
bool DONTMOVE = false;

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
SimpleKalmanFilter simpleKalmanFilter(100,2,0.05);
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t axNew, ayNew, azNew;
int16_t gxNew, gyNew, gzNew;
int prev_val = 0;
int diff = 0;


//SERVO SETUP
Servo lift;
  

//ULTRASONIC SENSOR SETUP

// Define Trig and Echo pins.
const int TRIG_PIN = 50;
const int ECHO_PIN = 48;

// movement variables
int prev_key = -1;
int pause = 0;
int key;
int turn_counter;

int challenge_num = 0;
int challenge1_counter = 0;
int angle = 30;

void setup() {

  Serial.begin(9600);
  Wire.begin();
  //0 is closed
  lift.attach(3);
  lift.write(angle);


  Serial.println("started");
  mpu.initialize();
  if(mpu.testConnection()){
    Serial.println("connected");
  } else {
    Serial.println("not successfull");
  }

  // buzzer
  pinMode(buzzer, OUTPUT);
  
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
  irmap["3091726080"] = 66;
  irmap["0"] = 99;

  // motor driver pinout setup
  pinMode(ena, OUTPUT); // pwm
  pinMode(enb, OUTPUT); // pwm

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);


  //ULTRASONIC SENSOR SETUP
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  // mpu offsets
  Serial.println("Updating internal sensor offsets...\n");
  mpu.setXAccelOffset(-5485); //Set your accelerometer offset for axis X
  mpu.setYAccelOffset(1383); //Set your accelerometer offset for axis Y
  mpu.setZAccelOffset(991); //Set your accelerometer offset for axis Z
  mpu.setXGyroOffset(69);  //Set your gyro offset for axis X
  mpu.setYGyroOffset(2);  //Set your gyro offset for axis Y
  mpu.setZGyroOffset(-6);  //Set your gyro offset for axis Z
  
  /*Print the defined offsets*/
  Serial.print("\t");
  Serial.print(mpu.getXAccelOffset());
  Serial.print("\t");
  Serial.print(mpu.getYAccelOffset()); 
  Serial.print("\t");
  Serial.print(mpu.getZAccelOffset());
  Serial.print("\t");
  Serial.print(mpu.getXGyroOffset()); 
  Serial.print("\t");
  Serial.print(mpu.getYGyroOffset());
  Serial.print("\t");
  Serial.print(mpu.getZGyroOffset());
  Serial.print("\n");

  if(challenge_num == 1) {
    lift.write(5); forwards(255); delay(1000); 
  }
}

void loop() {
  // catch IR signal and decide what to do

  if (pause >=3) {
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



  // int distanceCentimeters = getDistanceCentimeters();


  // Print the distance on the Serial Monitor.
  // Serial.print("Distance = ");
  // Serial.print(distanceCentimeters);
  // Serial.println(" cm");
  
  // challenge specific code
  switch (challenge_num) {
    case 0: move(key); break;
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
  // Serial.println(key);
  switch (key) {
    case 5: forwards(255); break;
    case 7: left(255); break;
    case 8: backwards(255); break;
    case 9: right(255); break;
    case 1: liftUp(); break;
    case 4: liftDown(); break;
    case 66: challenge_num = 1; lift.write(5); forwards(255); delay(1000); Serial.println("CHallenge 1 begins"); break;
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

//lift CONTORL FUNCTIONS


void liftUp() {
  Serial.println("up");
  if (angle >= -1){
    angle -= 2;
  }
  lift.write(angle);

}

void liftDown() {
  Serial.println("down");
  if (angle <= 40){
    angle += 2;
  }
  
  lift.write(angle);

}


// challenge operating stuff
void challengeOne() {
  
lift.write(30);

  //Get gyro data

  //mpu stuff
  mpu.getMotion6(&axNew, &ayNew, &azNew, &gxNew, &gyNew, &gzNew);
  ax = simpleKalmanFilter.updateEstimate(axNew) + 500;
  

  diff = ax - prev_val;

  Serial.print("a/g:\t");
  Serial.print(prev_val); Serial.print("\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(diff); Serial.print("\t");
  Serial.print(challenge1_counter); Serial.print("\t");
  Serial.println("");
  if (diff < -400){
    challenge1_counter += 1;
  }

  if (challenge1_counter >= 5){
    if (!DONTMOVE) {
        backwards(255);
      delay(800);
      DONTMOVE = true;
    }
    
    stop();
  } else{
    forwards(255);
  }

  

  prev_val = ax;
}

void challengeTwo() {
  return;
}

void challengeThree() {
  return;
}


