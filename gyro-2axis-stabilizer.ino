/*
  Arduino LSM6DSOX - Simple Gyroscope

  This example reads the gyroscope values from the LSM6DS3
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano RP2040 Connect

  created 10 May 2021
  by Arturo Guadalupi

  This example code is in the public domain.
*/

#include <Arduino_LSM6DSOX.h>
#include <math.h>
#include <Servo.h>

float lowPassWeight = 0.80;
float gyroWeight = 0.95;

float rawAccX, rawAccY, rawAccZ;
float filteredAccX, filteredAccY, filteredAccZ;

float rawGyroX, rawGyroY, rawGyroZ;
float calGyroX, calGyroY, calGyroZ;
float biasX, biasY, biasZ = 0.0;

float pitch, roll, yaw = 0.0;

float deltaT;
int prevTime;
float const num_sample = 100.0;

int preCameraYaw, preCameraPitch = 0;
int cameraYaw, cameraPitch = 0;

Servo servoH, servoV;

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (IMU.gyroscopeAvailable()) {
    for (int i = 0; i < (int)num_sample; i++) {
        Serial.println("Calabrating Gyro bias...");
        IMU.readGyroscope(rawGyroX, rawGyroY, rawGyroZ);
        //printRawGyro();
        biasX += rawGyroX;
        biasY += rawGyroY;
        biasZ += rawGyroZ;
    }
    biasX /= num_sample;
    biasY /= num_sample;
    biasZ /= num_sample;
   }

    servoH.attach(9);
    servoV.attach(10);
    //servoH.write(90);
    //servoV.write(90);
     prevTime = millis();
}

void loop() {

  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    IMU.readGyroscope(rawGyroX, rawGyroY, rawGyroZ);
    IMU.readAcceleration(rawAccX, rawAccY, rawAccZ);

    calGyroX = rawGyroX - biasX;
    calGyroY = rawGyroY - biasY;
    calGyroZ = rawGyroZ - biasZ;
    if (calGyroX > -0.1 && calGyroX < 0.1) calGyroX = 0.0;
    if (calGyroY > -0.1 && calGyroY < 0.1) calGyroY = 0.0;
    if (calGyroZ > -0.1 && calGyroZ < 0.1) calGyroZ = 0.0;

    // low pass filter
    filteredAccX = lowPassWeight * filteredAccX + (1 - lowPassWeight) * rawAccX;
    filteredAccY = lowPassWeight * filteredAccY + (1 - lowPassWeight) * rawAccY;
    filteredAccZ = lowPassWeight * filteredAccZ + (1 - lowPassWeight) * rawAccZ;

    deltaT = (millis() - prevTime) / 1000.0;
    pitch = gyroWeight * (pitch + calGyroY * deltaT) + (1 - gyroWeight) * atan2(filteredAccX, filteredAccZ) * 180 / PI;
    roll = gyroWeight * (roll + calGyroX * deltaT) + (1 - gyroWeight) * atan2(filteredAccY, filteredAccZ) * 180 / PI;
    yaw += calGyroZ * deltaT;
    prevTime = millis();

    cameraYaw = map((int)yaw, 90, -90, 0, 180);
    cameraPitch = map((int)pitch, 90, -90, 0, 180);

    if (cameraYaw != preCameraYaw) {
        servoH.write(cameraYaw);
        preCameraYaw = cameraYaw;
    }

    if (cameraPitch != preCameraPitch) {
        servoV.write(cameraPitch);
        preCameraPitch = cameraPitch;
    }
  
    //printBias();
    //printAll();
    //printAcc();
    printCamera();
    //printCalGyro();
  }
}

void printAll(){
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(yaw);
}

void printAcc() {
  Serial.print(rawAccX);
  Serial.print('\t');
  Serial.print(rawAccY);
  Serial.print('\t');
  Serial.println(rawAccZ);
}

void printCamera() {
  Serial.print(cameraYaw);
  Serial.print('\t');
  Serial.println(cameraPitch);
}

void printBias() {
  Serial.print(biasX);
  Serial.print('\t');
  Serial.print(biasY);
  Serial.print('\t');
  Serial.print(biasZ);
  Serial.print('\t');
}

void printCalGyro() {
  Serial.print(calGyroX);
  Serial.print('\t');
  Serial.print(calGyroY);
  Serial.print('\t');
  Serial.println(calGyroZ);
}

void printRawGyro() {
  Serial.print(rawGyroX);
  Serial.print('\t');
  Serial.print(rawGyroY);
  Serial.print('\t');
  Serial.println(rawGyroZ);
}

