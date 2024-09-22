#include <Wire.h>
#include <MPU6050.h>

// PID constants
#define Kp 1.0
#define Ki 0.5
#define Kd 0.1

// MPU6050
MPU6050 mpu;
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
float angleX, angleY;

// PID variables
float setPointX = 0.0; // Target angle for X axis (seesaw balance)
float setPointY = 0.0; // Target angle for Y axis (seesaw balance)

float inputX, inputY;
float outputX, outputY;

float integralX = 0.0;
float integralY = 0.0;

float previousErrorX = 0.0;
float previousErrorY = 0.0;

unsigned long previousTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  Serial.println("MPU6050 connected");

  // Initialize motor control pins
  pinMode(9, OUTPUT); // PWM pin for motor X
  pinMode(10, OUTPUT); // PWM pin for motor Y
  pinMode(8, OUTPUT); // Direction control pin for motor X
  pinMode(7, OUTPUT); // Direction control pin for motor Y
}

void loop() {
  // Get time delta
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // Read MPU6050 sensor data
  mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);

  // Convert accelerometer readings to angles (simplified)
  angleX = atan2(accelY, accelZ) * 180 / PI;
  angleY = atan2(accelX, accelZ) * 180 / PI;

  // Set input to PID controller
  inputX = angleX;
  inputY = angleY;

  // Compute PID for X
  float errorX = setPointX - inputX;
  integralX += errorX * deltaTime;
  float derivativeX = (errorX - previousErrorX) / deltaTime;
  outputX = Kp * errorX + Ki * integralX + Kd * derivativeX;
  previousErrorX = errorX;

  // Compute PID for Y
  float errorY = setPointY - inputY;
  integralY += errorY * deltaTime;
  float derivativeY = (errorY - previousErrorY) / deltaTime;
  outputY = Kp * errorY + Ki * integralY + Kd * derivativeY;
  previousErrorY = errorY;

  // Control motors based on PID output
  controlMotor(outputX, outputY);

  // Print results
  Serial.print("Angle X: ");
  Serial.print(angleX);
  Serial.print(" PID output X: ");
  Serial.println(outputX);

  Serial.print("Angle Y: ");
  Serial.print(angleY);
  Serial.print(" PID output Y: ");
  Serial.println(outputY);

  delay(100); // Adjust delay as needed
}

void controlMotor(float outputX, float outputY) {
  // Control the motor speed and direction based on PID output
  int motorSpeedX = constrain(map(abs(outputX), 0, 180, 0, 255), 0, 255);
  int motorSpeedY = constrain(map(abs(outputY), 0, 180, 0, 255), 0, 255);

  if (outputX > 0) {
    digitalWrite(8, HIGH); // Set motor direction for X
  } else {
    digitalWrite(8, LOW); // Set motor direction for X
  }

  if (outputY > 0) {
    digitalWrite(7, HIGH); // Set motor direction for Y
  } else {
    digitalWrite(7, LOW); // Set motor direction for Y
  }

  analogWrite(9, motorSpeedX); // PWM control for motor X
  analogWrite(10, motorSpeedY); // PWM control for motor Y
}
