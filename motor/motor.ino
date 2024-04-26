#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <esp32-hal-ledc.h>

#define BRAKE 33

#define PWM1 4
#define PWM1_CH 1
#define DIR1 2

#define TIMER_BIT  8
#define MOTOR_FREQUENCY  20000

#define POT_KP 27
#define POT_KI 26
#define POT_KD 32
#define AIN1 18 // Define these pins if not already defined
#define AIN2 19

// IMU functions;
sensors_event_t a, g, temp;
Adafruit_MPU6050 mpu;

// Define physical constants
const float pi = 3.14159265359;

enum motorMode { shortBrake, CCW, CW, stop };
int kp_val = 0;
int ki_val = 0;
int kd_val = 0;
int speed = 0;

struct calibrate {
  sensors_vec_t gyro;
  sensors_vec_t acceleration;
};

struct calibrate Calibration;

// Alpha-beta filter values
float roll = 0;
float theta_roll;
float pitch = 0;
float theta_pitch;
float roll_dot = 0;
float pitch_dot = 0;
float alpha = 0.8;
float beta = 0.1;
const int T_int = 50; // Integration period in ms

// PID Parameters
double KP = 0; // Gains - tune later
double KI = 0;
double KD = 0;
float setpoint = 2.49; // 0 degrees?
float SETPOINT = 2.49;
float integral = 0;
float error = 0;
float previousError = 0;
float derivative = 0;
float PIDout = 0;
int time_ = 0;
int prevTime = 0;
int time_delta = 0;

const int ControlLoopFrequency = 75; // Increase for faster response


unsigned long balancingStartTime;

void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

void setup() {
  // Configure PWM channel
  ledcSetup(PWM1_CH, MOTOR_FREQUENCY, TIMER_BIT); // PWM channel 0, 8-bit resolution

  // Attach PWM channel to GPIO pin
  ledcAttachPin(PWM1, PWM1_CH); // Use PWM channel 0

  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);

  motorControl(0, 0);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(POT_KP, INPUT);
  pinMode(POT_KI, INPUT);
  pinMode(POT_KD, INPUT);
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  calibrateSensors();

  // Set time
  time_ = millis();
  prevTime = time_;

  balancingStartTime = millis();
}

void loop() {

  if (Serial.available() > 0) {
      // read the incoming bytes
      char incomingByte = Serial.read();

      // update the variables based on the received byte
      if (incomingByte == 'KP') {
        KP = readIntFromSerial();
      } else if (incomingByte == 'KI') {
        KI = readIntFromSerial();
      } else if (incomingByte == 'KD') {
        KD = readIntFromSerial();
      }
  }



  prevTime = time_;
  time_ = millis();

  time_delta = time_ - prevTime;

  kp_val = analogRead(POT_KP);
  ki_val = analogRead(POT_KI);
  kd_val = analogRead(POT_KD);

  KP = map(kp_val, 0, 4095, 3000, 3500);
  KI = map(ki_val, 0, 4095, 500, 5000);
  KD = map(kd_val, 0, 4095,500, 800);

  // KP = 3090;
  // KI = 2500;
  // KD = 500;


  // Serial.print(" KP:");
  // Serial.print(kp_val);
  // Serial.print(" ");
  // Serial.print(KP);
  // Serial.print(" KI:");
  //  Serial.print(KI);
  // Serial.print(" KD:");
  // Serial.println(KD);

  getAngles();

    // Add dithering to prevent acceleration in one direction at equilibrium
  if(pitch>=SETPOINT){
    setpoint -= .05*time_delta;
  }else{
    setpoint += .05*time_delta;
  }
  setpoint = constrain(setpoint, SETPOINT - .01, SETPOINT + .01);

  error = setpoint - theta_pitch;

  // Serial.print(" setpoint:");
  // // Serial.print(setpoint);
  // Serial.print(" Pitch:");
  // Serial.print(pitch);

  previousError = error;
  derivative = g.gyro.x;
  integral += error * (time_ - prevTime)/1000;
  integral = constrain(integral, -255, 255);

  PIDout = KP * error + KI * integral - KD * derivative;
  speed = constrain(PIDout, -255, 255);

  // Serial.print(" Speed:");
  // Serial.println(speed);

  motorControl(speed, PWM1_CH);

  // unsigned long loopStartTime = millis();

  //   // Delay to control loop frequency
  // unsigned long loopEndTime = millis();
  // int loopDuration = loopEndTime - loopStartTime;
  // if (loopDuration < 1000 / ControlLoopFrequency) {
  //   delay(1000 / ControlLoopFrequency - loopDuration);
  // }
  delay(10);
}

void getAngles() {
  // get imu data and calculate angles
  mpu.getEvent(&a, &g, &temp);
  getEulerAngles(a.acceleration);
  theta_roll = alphaFilter(roll, theta_roll);
  theta_pitch = alphaFilter(pitch, theta_pitch);
  // Serial.print("ThetaPitch:");
  // Serial.print(theta_pitch);
  // Serial.print(" Pitch:");
  // Serial.println(pitch);
}

void setMode(int mode) {
  switch (mode) {
    case shortBrake:
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, HIGH);
      break;
    case CCW:
      digitalWrite(DIR1, LOW);
      break;
    case CW:
      digitalWrite(DIR1, HIGH);
      break;
  }
}

void calibrateSensors() {
  float caltime = millis();
  float acc[3] = {0, 0, 0};
  float gyro[3] = {0, 0, 0};
  int c = 0;

  Serial.println("Calibrating Sensors for 1 second...");
  while ((caltime + 1000) > millis()) {
    mpu.getEvent(&a, &g, &temp);
    acc[0] += a.acceleration.x;
    acc[1] += a.acceleration.y;
    acc[2] += a.acceleration.z;
    gyro[0] += g.gyro.x;
    gyro[1] += g.gyro.y;
    gyro[2] += g.gyro.z;
    c++;
  }

  Calibration.gyro.x = g.gyro.x / c;
  Calibration.gyro.y = g.gyro.y / c;
  Calibration.gyro.z = g.gyro.z / c;
  Calibration.acceleration.x = a.acceleration.x / c;
  Calibration.acceleration.y = a.acceleration.y / c;
  Calibration.acceleration.z = a.acceleration.z / c;

  Serial.println("Done Calibrating");
  Serial.print("Calibrated Roll = ");
  Serial.println(Calibration.gyro.x);
  delay(1000);
}

void getEulerAngles(sensors_vec_t a) {
  pitch = atan2(a.y, a.z); //pitch
  roll = atan2(a.x, a.z);
}

void alphaBetaFilter(float xm, float vm, float *xk, float *vk) {
  float xk_next = *xk + (*vk * T_int);
  float vk_next = *vk;
  float rk = xm - xk_next;
  xk_next += alpha * rk;
  vk_next += (beta * rk) / T_int;
  *xk = xk_next;
  *vk = vk_next;
}

float alphaFilter(float xm, float xk) {
  float rk = xm - xk;
  xk = xk + alpha * rk;
  return xk;
}

void motorControl(int speed, int motorPIN) {
  if (speed < 0) {
    setMode(CW);
    speed = -speed;
  } else {
    setMode(CCW);
  }
  pwmSet(PWM1_CH, speed > 255 ? 255 : 255 - speed);
}

int readIntFromSerial() {
  // read the incoming bytes until a newline character is received
  String inputString = Serial.readStringUntil('\n');
  
  // convert the string to an integer
  return inputString.toInt();
}