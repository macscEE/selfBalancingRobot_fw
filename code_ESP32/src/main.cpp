// Load necessary libraries
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// Define pin numbers
#define SDA_PIN 21
#define SCL_PIN 22

#define DEBUG_PIN_1 2
#define DEBUG_PIN_2 0

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

// STATE MACHINE STATES
typedef enum{
  STATE_INIT,
  STATE_MEASURE_ANGLE,
  STATE_COMPUTE_PID,
  STATE_DRIVE_MOTORS,
  STATE_RESTART,
  STATE_CRASHED
}RobotState;

// SERIAL PRINTING
struct Sample {
    double angleAcc;
    double angleGyro;
    double error;
    double control;
};
Sample buf[512];
volatile int wptr = 0;
volatile int wptr_old = 0;

/* Define PID parameters
 Input => Measured value
 Output => Voltage applied to motor
 Setpoint => Desired value of angle, stable at 0 degrees */
 double Setpoint, Input, Output;

// PID tuning parameters
//double Kp=300, Ki=0.34, Kd=0.0020;
uint16_t sampleTime = 4; //sampletime in ms 
double Kp=250, Ki=0.3, Kd=0.0040;
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
double alpha = 0.50; // Complementary filter coefficient -> higher value gives more weight to gyroscope
double dt = 0.008;   // Time interval in seconds
double angle = 0;   // Filtered angle

// PWM definition at 12 bit
// Impostazioni PWM
const int pins[] = {M_R_forward, M_R_backward, M_L_forward, M_L_backward};
const int channels[] = {CH_M_R_forward, CH_M_R_backward, CH_M_L_forward, CH_M_L_backward};
const int freq = 100;//19531;  // Frequenza massima per 12 bit (80 MHz / 4096)
const int res = 8;     // Risoluzione a 12 bit (0–4095)

hw_timer_t *Timer0_Cfg = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool tick = false;

RobotState currentState = STATE_INIT;

double angleEstimation();
void motorControl(int16_t pwm);
void PIDresponse(double angle_deg);

void IRAM_ATTR Timer0_ISR(){
  portENTER_CRITICAL_ISR(&timerMux);
  tick = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}


void setup() {
  
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  pinMode(M_R_forward, OUTPUT);
  pinMode(M_R_backward, OUTPUT);
  pinMode(M_L_forward, OUTPUT);
  pinMode(M_L_backward, OUTPUT);
  pinMode(DEBUG_PIN_1, OUTPUT);
  pinMode(DEBUG_PIN_2, OUTPUT);

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
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4); // set accelerometer range to maximum 4g
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

  Timer0_Cfg = timerBegin(0, 80, true); // 80e6/80 = 1e6 => 1 tick = 1 us
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 1000, true); //1000*1us = 1ms
  timerAlarmEnable(Timer0_Cfg);

  delay(1000);

  currentState = STATE_MEASURE_ANGLE;

}

void loop() {
  if(tick == true){
    portENTER_CRITICAL(&timerMux);
    tick = false;
    portEXIT_CRITICAL(&timerMux);
  
    switch(currentState){
      case STATE_MEASURE_ANGLE:{
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Legge i dati grezzi da MPU6050
        angle = angleEstimation();
        if(pitchAcc>60 || pitchAcc<-60){
          currentState = STATE_CRASHED;
        }else{
          currentState = STATE_COMPUTE_PID;
        }
        digitalWrite(DEBUG_PIN_1, 1);
        break;
      }
      case STATE_COMPUTE_PID:{
        PIDresponse(angle);
        currentState = STATE_DRIVE_MOTORS;
        break;
      }
      case STATE_DRIVE_MOTORS:{
        motorControl(Output);
        digitalWrite(DEBUG_PIN_1, 0);
        buf[wptr] = {pitchAcc, pitchGyro, angle, Output}; // buffer for serial print
        wptr = (wptr + 1) % 512;
        currentState = STATE_MEASURE_ANGLE;
        break;
      }
      case STATE_CRASHED:{
        motorControl(0);
        PIDresponse(0);
        buf[wptr] = {angle, Output}; // buffer for serial print
        wptr = (wptr + 1) % 512;
        //Serial.println("\nCRASHED!\n");
        currentState = STATE_MEASURE_ANGLE;
        break;
      }
      
      
    }
    //Serial.println("Angle: " + String(angle) + " Pitch Acc " + String(pitchAcc) + " Pitch Gyro " + String(pitchGyro) + " PID Output "+ String(Output)+",");
    //Serial.println(String(angle) +","+ String(pitchAcc) +","+ String(pitchGyro) +","+ String(Output)+" ");

    if((wptr != wptr_old) && (wptr%10==0)){
      Serial.print((int16_t)(buf[wptr].angleAcc));
      Serial.print(",");
      Serial.print((int16_t)(buf[wptr].angleGyro));
      Serial.print(",");
      Serial.print((int16_t)(buf[wptr].error));
      Serial.print(",");
      Serial.print((int16_t)(buf[wptr].control));
      Serial.print("\n");
      wptr_old = wptr;
    }
  }
}




// Function to estimate angle using complementary filter
double angleEstimation(){
  double angle_deg = 0;

  ax = ax * 0.001196289; // (9.8/4096) for 8g acceleration range
  ay = ay * 0.001196289;
  az = az * 0.001196289;

  // Estimation pitch angle (y-axis) accelerometer
  pitchAcc = atan2(ay, az)*(360/PI); // in degrees

  // Estimation pitch angle (y-axis) gyroscope
  pitchGyro = gx * 0.007629395; // [deg/s] (res of the gyroscope for ±250°/s range)
  
  // Complementary filter to combine accelerometer and gyroscope data
  return angle_deg = alpha * (angle + pitchGyro * dt) + (1 - alpha) * pitchAcc;
  //angle = angle * (PI/180);
  //angle = pitchGyro * dt;
}

// PID response function
void PIDresponse(double angle_deg){
// Input of PID will be the y-axis angle
  Input = angle_deg * PI/180; // in radians
  controller.Compute(); // Calcola il nuovo output del PID
  controller.SetOutputLimits(-255, 255);//(-4095, 4095); // Limita l'output tra -4095 e 4095 (12 bit)
  controller.SetSampleTime(sampleTime);
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