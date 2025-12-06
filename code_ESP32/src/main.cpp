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
#define CRASH_TRESHOLD 60

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

uint16_t sampleTime = 4; //sampletime in ms 
//double Kp=5, Ki=0.1, Kd=0.05;
double Kp=0.8, Ki=0.3, Kd=0.05;
// Create an MPU6050 object
MPU6050 mpu;

// Create a PID controller object
PID controller(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Variables to hold raw sensor data
int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

// Variables for angle calculations
double pitchAcc, pitchGyro, rateGyro;

// Accelerometer Filter
double ax_f = 0, ay_f = 0, az_f = 0;
double accel_alpha = 0.7;
double imuFilter_dt=0, imuFilter_now=0, imuFilter_lastCall=0; 
double gx_f = 0;
double gyro_alpha = 0.6;

// Complementary filter parameters
double imuFilter_alpha = 0.6; // Complementary filter coefficient -> higher value gives more weight to gyroscope
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
uint16_t time_count = 0;

RobotState currentState = STATE_INIT;

void angleEstimation();
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

    //if(time_count == 10){
      time_count = 0;
      switch(currentState){
        case STATE_MEASURE_ANGLE:{
          mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Legge i dati grezzi da MPU6050
          angleEstimation();
          if(pitchAcc>CRASH_TRESHOLD || pitchAcc<-CRASH_TRESHOLD){
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

    //time_count++;
  }
//}

// Function to estimate angle using complementary filter
void angleEstimation(){

  ax = ax * 0.002394202; // (9.8/4096) for 8g acceleration range
  ay = ay * 0.002394202;
  az = az * 0.002394202;

  ax_f = ax_f + accel_alpha * (ax - ax_f);
  ay_f = ay_f + accel_alpha * (ay - ay_f);
  az_f = az_f + accel_alpha * (az - az_f);

  // Estimation pitch angle (y-axis) accelerometer
  pitchAcc = atan2(ay_f, az_f)*(360/PI); // in degrees

  // Estimation pitch angle (y-axis) gyroscope
  //gx_f = gx_f + gyro_alpha * (gx - gx_f);
  rateGyro = gx * (1.0/131); // [deg/s] (resolution of the gyroscope for ±250 deg/s range)

  // This block finds the elapsed time between the current and the previous function call to determine the integration interval 
  imuFilter_now = micros();
  imuFilter_dt = (imuFilter_now - imuFilter_lastCall) / 1e6;
  imuFilter_lastCall = imuFilter_now;

  pitchGyro +=  rateGyro * imuFilter_dt; // [deg/s] integrated rate over time
  
  // Complementary filter to combine accelerometer and gyroscope data
  angle = imuFilter_alpha * (angle + pitchGyro) + (1 - imuFilter_alpha) * pitchAcc;  //dt =0.03 max limit for oscillations
  //angle = angle * (PI/180);
  //angle = pitchGyro * dt;
}

// PID response function
void PIDresponse(double angle_deg){
// Input of PID will be the y-axis angle
  Input = angle_deg; //* PI/180; // in radians
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