#include <Arduino.h>
#include <IRremote.hpp>
#include <Arduino_JSON.h>

using namespace std;

//IR reciever pinout and key-setup
const int RECV_PIN = A0;
JSONVar irmap;

// Motor driver pinout
const int ena = 13;
const int enb = 12;

const int in1 = 22;
const int in2 = 24;
const int in3 = 26;
const int in4 = 28;

// movement variables
int prev_key = 0;
int key;

int challenge_num = 1;

void setup() {
  Serial.begin(9600);


  
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


}

void loop() {
  // catch IR signal and decide what to do

  if (IrReceiver.decode()) {
    String str = String(IrReceiver.decodedIRData.decodedRawData);
    key = irmap[str];
    IrReceiver.resume(); // Enable receiving of the next value
  }
  move(key);

  
  // challenge specific code
  switch (challenge_num) {
    case 1: challengeOne(); break;
    case 2: challengeTwo(); break;
    case 3: challengeThree(); break;
  }

}

// movement functions
void move(int key) {
  if (key == 99) {
    key = prev_key;
  }
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
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void right(int speed) {
  Serial.println("Going roight?!");

  analogWrite(ena, speed);
  analogWrite(enb, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// challenge operating stuff
void challengeOne() {
  return;
}

void challengeTwo() {
  return;
}

void challengeThree() {
  return;
}