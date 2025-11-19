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
 
// Target angle in degrees
#define reference 0 

/* Define PID parameters
 Input => Measured value
 Output => Voltage applied to motor
 Setpoint => Desired value of angle, stable at 0 degrees */
 double Setpoint, Input, Output;

// PID tuning parameters
double Kp=0.01, Ki=0.1379, Kd=0.0022;

// Create an MPU6050 object
MPU6050 mpu;

// Create a PID controller object
PID controller(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Variables to hold raw sensor data
int16_t ax, ay, az, gx, gy, gz;

// Variables for angle calculations
float pitchAcc, pitchGyro;

// Complementary filter parameters
float alpha = 0.98; // Complementary filter coefficient -> higher value gives more weight to gyroscope
float dt = 0.01;   // Time interval in seconds
float angle = 0;   // Filtered angle

// PWM definition at 12 bit
// Impostazioni PWM
const int pins[] = {M_R_forward, M_R_backward, M_L_forward, M_L_backward};
const int chanels[] = {0, 1, 2, 3};
const int freq = 19531;  // Frequenza massima per 12 bit (80 MHz / 4096)
const int res = 12;     // Risoluzione a 12 bit (0–4095)

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

  // Configura i canali PWM
  for (int i = 0; i < 4; i++) {
    ledcSetup(chanels[i], freq, res);
    ledcAttachPin(pins[i], chanels[i]);
  }
  delay(1000);
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Legge i dati grezzi da MPU6050
 
  angleEstimation(); // Stima l'angolo usando il filtro complementare

  PIDresponse(); // Calcola la risposta del PID 

  // Serial.println("Output PID: " + String(Output) + "\t");
  Serial.println("Input (Angolo Y): " + String(angle) + "\t");

  motorControl(); // Controlla i motori in base all'output del PID
}

// Function to estimate angle using complementary filter
void angleEstimation(){
  
  // Estimation pitch angle (y-axis) accelerometer
  pitchAcc = atan2(az, ax)*180/PI; // in degrees

  // Estimation pitch angle (y-axis) gyroscope
  pitchGyro = gy / 131.0; // in degrees/s, The value 131.0 corresponds to the sensitivity of the gyroscope at its default ±250°/s range
  
  // Complementary filter to combine accelerometer and gyroscope data
  angle = alpha * (angle + pitchGyro * dt) + (1 - alpha) * pitchAcc;
}

// PID response function
void PIDresponse(){
// Input of PID will be the y-axis angle
  Input = angle * PI/180; // in radians
  controller.Compute(); // Calcola il nuovo output del PID
  controller.SetOutputLimits(-4095, 4095); // Limita l'output tra -4095 e 4095 (12 bit)
}

 // Control motors based on PID output
void motorControl(){

  if (Output > 0) {
    // Move forward
    analogWrite(M_R_forward, Output);
    analogWrite(M_R_backward, 0);
    analogWrite(M_L_forward, Output);
    analogWrite(M_L_backward, 0);
  } 
  else if (Output < 0) {
    // Move backward
    analogWrite(M_R_forward, 0);
    analogWrite(M_R_backward, -Output);
    analogWrite(M_L_forward, 0);
    analogWrite(M_L_backward, -Output);
  } 
  else {
    // Stop
    analogWrite(M_R_forward, 0);
    analogWrite(M_R_backward, 0);
    analogWrite(M_L_forward, 0);
    analogWrite(M_L_backward, 0);
  }
}