#include <Arduino.h>


#include <IRremote.hpp>

#include <Arduino_JSON.h>


using namespace std;


const int RECV_PIN = A0;
JSONVar irmap;

// put function declarations here:
int ChallengeOne(int, int);
int ChallengeTwo(int, int);
int ChallengeThree(int, int);

void setup() {
  Serial.begin(19200);

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
  // Setup IMU settings

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  //
}

void loop() {
  // put your main code here, to run repeatedly:

  // catch IR signal and decide what to do
  if (IrReceiver.decode()) {
    String str = String(IrReceiver.decodedIRData.decodedRawData);
    int key = irmap[str];
    IrReceiver.resume(); // Enable receiving of the next value
    }
}

// put function definitions here:
int ChallengeOne(int x, int y) {
  return x + y;
}

int ChallengeTwo(int x, int y) {
  return x + y;
}

int ChallengeThree(int x, int y) {
  return x + y;
}

#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

int IMU() {
   // === Read acceleromter data === //
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);
   Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
   //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
   AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
   AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
   AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
   // Calculating Roll and Pitch from the accelerometer data
   accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
   accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
   // === Read gyroscope data === //
   previousTime = currentTime;        // Previous time is stored before the actual time read
   currentTime = millis();            // Current time actual time read
   elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
   Wire.beginTransmission(MPU);
   Wire.write(0x43); // Gyro data first register address 0x43
   Wire.endTransmission(false);
   Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
   GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
   GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
   GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
   // Correct the outputs with the calculated error values
   GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
   GyroY = GyroY - 2; // GyroErrorY ~(2)
   GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
   // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
   gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
   gyroAngleY = gyroAngleY + GyroY * elapsedTime;
   yaw =  yaw + GyroZ * elapsedTime;
   // Complementary filter - combine acceleromter and gyro angle values
   roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
   pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
   
   // Print the values on the serial monitor
   Serial.print(roll);
   Serial.print("/");
   Serial.print(pitch);
   Serial.print("/");
   Serial.println(yaw);
}


