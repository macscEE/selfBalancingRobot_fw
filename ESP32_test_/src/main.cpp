// Load necessary libraries

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// Define pin numbers
#define SDA_PIN 21
#define SCL_PIN 22

// Define PID parameters
// Input => Measured value
// Output => Voltage applied to motor
// Setpoint => Desired value of angle, stable at pi [radians] = 180 degrees
double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;

// Create an MPU6050 object
MPU6050 mpu;

// Create a PID controller object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);

  Serial.begin(115200);
  Wire.begin();

  Serial.println("Inizializzazione MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("Connessione al MPU6050 fallita!");
    while (1);
  }
  Serial.println("MPU6050 connesso correttamente!");
  
  // Calibra i sensori
  mpu.CalibrateAccel(6);  // 6 = numero di campioni
  mpu.CalibrateGyro(6);

  Serial.println("Calibrazione completata!");
  mpu.PrintActiveOffsets();  // Mostra gli offset calcolati

  // Imposta il setpoint iniziale
  Setpoint = 180; // Target angle in degrees
  myPID.SetMode(AUTOMATIC); // Attiva il PID controller

  Serial.println("Setup completato!");
  delay(1000);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  // Input of PID will be the y-axis angle
  // Input = mpu.getAngleY(&ay);

}