// Load necessary libraries

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// Define pin numbers
#define SDA_PIN 21
#define SCL_PIN 22

#define M_R_forward 5
#define M_R_backward 17
#define M_L_forward 4
#define M_L_backward 16 

#define reference 0 // Target angle in degrees

// Define PID parameters
// Input => Measured value
// Output => Voltage applied to motor
// Setpoint => Desired value of angle, stable at pi [radians] = 180 degrees
double Setpoint, Input, Output;
double Kp=0.01, Ki=0.1379, Kd=0.0022;

// Create an MPU6050 object
MPU6050 mpu;

// Create a PID controller object
PID controller(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Variables to hold raw sensor data
int16_t ax, ay, az, gx, gy, gz;

// Complementary filter parameters
float alpha = 0.98; // Complementary filter coefficient
float dt = 0.01;   // Time interval in seconds
float angle = 0;   // Filtered angle

void setup() {
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  pinMode(M_R_forward, OUTPUT);
  pinMode(M_R_backward, OUTPUT);
  pinMode(M_L_forward, OUTPUT);
  pinMode(M_L_backward, OUTPUT);

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
  Setpoint = reference; // Target angle in degrees
  controller.SetMode(AUTOMATIC); // Attiva il PID controller

  Serial.println("Setup completato!");
  delay(1000);
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Legge i dati grezzi da MPU6050
 
  // Estimation pitch angle (y-axis) accelerometer
  float pitchAcc = atan2(ax, az)*180/PI; // in degrees

  // Estimation pitch angle (y-axis) gyroscope
  float pitchGyro = gy / 131.0; // in degrees/s
  float angularVelocity = pitchGyro * PI/180; // radians/s
  
  // Complementary filter to combine accelerometer and gyroscope data
  angle = alpha * (angle + pitchGyro * dt) + (1 - alpha) * pitchAcc;

  // Input of PID will be the y-axis angle
  Input = angle * PI/180; // in radians
  controller.Compute(); // Calcola il nuovo output del PID
  controller.SetOutputLimits(-255, 255); // Limita l'output tra -255 e 255 -> da cambiare in modo tale che siano da -6 a 6 V, 

  // Serial.println("Output PID: " + String(Output) + "\t");
  Serial.println("Input (Angolo Y): " + String(angle) + "\t");

  if(Output > 0){
    analogWrite(M_R_forward, Output); // Applica l'output ai motori in avanti
    analogWrite(M_L_forward, Output);
    analogWrite(M_R_backward, 0);
    analogWrite(M_L_backward, 0);
  }
  else
  {
    analogWrite(M_R_backward, -Output); // Applica l'output ai motori indietro
    analogWrite(M_L_backward, -Output);
    analogWrite(M_R_forward, 0);
    analogWrite(M_L_forward, 0);
  }
}