// Load necessary libraries
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// Define pin numbers
#define SDA_PIN 21
#define SCL_PIN 22

#define M_R_forward 18
#define M_R_backward 17
#define M_L_forward 4
#define M_L_backward 16
 
// Target angle in degrees
#define reference 0 

#define CH_M_R_forward 0
#define CH_M_R_backward 1
#define CH_M_L_forward 2
#define CH_M_L_backward 3


/* Define PID parameters
 Input => Measured value
 Output => Voltage applied to motor
 Setpoint => Desired value of angle, stable at 0 degrees */
 double Setpoint, Input, Output;

// PID tuning parameters
double Kp=400, Ki=30, Kd=0.0022;
//double Kp = 140;double Ki = 10;double Kd = 3; <== VALORI A CASO PER TEST
// Create an MPU6050 object
MPU6050 mpu;

// Create a PID controller object
PID controller(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Variables to hold raw sensor data
int16_t ax, ay, az, gx, gy, gz;

// Variables for angle calculations
double pitchAcc, pitchGyro;

// Complementary filter parameters
double alpha = 0.30; // Complementary filter coefficient -> higher value gives more weight to gyroscope
double dt = 0.01;   // Time interval in seconds
double angle = 0;   // Filtered angle

// PWM definition at 12 bit
// Impostazioni PWM
const int pins[] = {M_R_forward, M_R_backward, M_L_forward, M_L_backward};
const int channels[] = {CH_M_R_forward, CH_M_R_backward, CH_M_L_forward, CH_M_L_backward};
const int freq = 100;//19531;  // Frequenza massima per 12 bit (80 MHz / 4096)
const int res = 8;     // Risoluzione a 12 bit (0–4095)

void angleEstimation();
void motorControl(int16_t pwm);
void PIDresponse();



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
  // Sensor range adjustment (cover at least 180°)
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // set gyro range to maximum 250 degrees (best resolution)
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8); // set accelerometer range to maximum 8g (a bit much, try reducing if problems arise)
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
    ledcSetup(channels[i], freq, res);
    ledcAttachPin(pins[i], channels[i]);
  }
  delay(1000);
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Legge i dati grezzi da MPU6050
  ax = ax * 0.002392578; // (9.8/4096) for 8g acceleration range
  ay = ay * 0.002392578; 
  az = az * 0.002392578;
  angleEstimation(); // Stima l'angolo usando il filtro complementare

  PIDresponse(); // Calcola la risposta del PID 
  delay(1); 
  // Serial.println("Output PID: " + String(Output) + "\t");
  motorControl(Output); // Controlla i motori in base all'output del PID
  Serial.println("Input (Angolo Y): " + String(angle) + " " + String(pitchAcc) + " " + String(pitchGyro) + " "+ String(Output));     //+ "\t");//
}

// Function to estimate angle using complementary filter
void angleEstimation(){
  
  // Estimation pitch angle (y-axis) accelerometer
  pitchAcc = atan2(ay, az)*(360/PI); // in degrees

  // Estimation pitch angle (y-axis) gyroscope
  pitchGyro = gx * 0.007629395; // [deg/s] (res of the gyroscope for ±250°/s range)
  
  // Complementary filter to combine accelerometer and gyroscope data
  angle = alpha * (angle + pitchGyro * dt) + (1 - alpha) * pitchAcc;
  //angle = angle * (PI/180);
  //angle = pitchGyro * dt;
}

// PID response function
void PIDresponse(){
// Input of PID will be the y-axis angle
  Input = angle * PI/180; // in radians
  controller.Compute(); // Calcola il nuovo output del PID
  controller.SetOutputLimits(-255, 255);//(-4095, 4095); // Limita l'output tra -4095 e 4095 (12 bit)
}

 // Control motors based on PID output
void motorControl(int16_t pwm){

  if (Output > 0) {
    // Move forward
    ledcWrite(CH_M_R_forward, pwm);//analogWrite(M_R_forward, Output);
    ledcWrite(CH_M_R_backward, 0);
    ledcWrite(CH_M_L_forward, pwm);
    ledcWrite(CH_M_L_backward, 0);
  } 
  else if (Output < 0) {
    // Move backward
    ledcWrite(CH_M_R_forward, 0);
    ledcWrite(CH_M_R_backward, -pwm);
    ledcWrite(CH_M_L_forward, 0);
    ledcWrite(CH_M_L_backward, -pwm);
  } 
  else {
    // Stop
    ledcWrite(CH_M_R_forward, 0);
    ledcWrite(CH_M_R_backward, 0);
    ledcWrite(CH_M_L_forward, 0);
    ledcWrite(CH_M_L_backward, 0);
  }
}