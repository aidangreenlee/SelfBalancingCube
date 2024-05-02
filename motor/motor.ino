#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <esp32-hal-ledc.h>
#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 0
#define BRAKE 27
#define PWM1 14
#define PWM1_CH 1
#define DIR1 12
#define TIMER_BIT  8
#define MOTOR_FREQUENCY  20000
#define POT_KP 27
#define POT_KI 26
#define POT_KD 32
#define AIN1 18
#define AIN2 19

sensors_event_t a, g, temp;
Adafruit_MPU6050 mpu;

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

float roll = 0;
float theta_roll;
float pitch = 0;
float theta_pitch;
float roll_dot = 0;
float pitch_dot = 0;
float alpha = 0.8;
float beta = 0.1;
const int T_int = 50;

float KP;
float KI;
float KD;

float setpoint = -0.95;
float SETPOINT = -0.95;
float integral = 0;
float error = 0;
float previousError = 0;
float derivative = 0;
float PIDout = 0;
int time_ = 0;
int prevTime = 0;
int time_delta = 0;

unsigned long balancingStartTime;

void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

int receivedKP;
int receivedKI;
int receivedKD ;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  // if (len == sizeof(float)) {
    // receivedKP = *(float *)data;
      // Allocate memory for the identifier
    char identifier[4]; // Assuming the maximum length of the identifier is 3 characters

    // Copy the identifier from the received data
    strncpy(identifier, (char*)data, 3);
    identifier[3] = '\0'; // Ensure null termination

    if (strncmp(identifier, "KP=", 3) == 0){
      sscanf((char*)data,"KP=%f", &KP);
      Serial.print("Received KP: ");
      Serial.println(KP);
    }else if (strncmp(identifier, "KI=", 3) == 0){
      sscanf((char*)data,"KI=%f", &KI);
      Serial.print("Received KI: ");
      Serial.println(KI);
    }else if (strncmp(identifier, "KD=", 3) == 0){
      sscanf((char*)data,"KD=%f", &KD);
      Serial.print("Received KD: ");
      Serial.println(KD);
    }

    // Serial.println(receivedKP);
    // delay(3000);
  //   KP = receivedKP;
  //   Serial.println("Received KP: ");
  //   //Serial.println(receivedKP);
  // } else if (len == 2 * sizeof(float)) {
  //   receivedKI = *(float *)(data + sizeof(float));
  //   KI = receivedKI;
  //   Serial.println("Received KI: ");
  //   //Serial.println(receivedKI);
  // } else if (len == 3 * sizeof(float)) {
  //   receivedKD = *(float *)(data + 2 * sizeof(float));
  //   KD = receivedKD;
  //   Serial.println("Received KD: ");
  //   //Serial.println(receivedKD);
  // }
}

void setup() {
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  pinMode(DIR1, OUTPUT);
  ledcSetup(PWM1_CH, MOTOR_FREQUENCY, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  motorControl(0, 0); 

  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  calibrateSensors();

  delay(1000);

  time_ = millis();
  prevTime = time_;

  balancingStartTime = millis();

  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  prevTime = time_;
  time_ = millis();
  time_delta = time_ - prevTime;

  getAngles();

  if(pitch >= SETPOINT){
    setpoint -= 0.05 * time_delta;
  } else {
    setpoint += 0.05 * time_delta;
  }
  setpoint = constrain(setpoint, SETPOINT - 0.01, SETPOINT + 0.01);

  error = setpoint - theta_roll;

  previousError = error;
  derivative = g.gyro.y;
  integral += error * (time_ - prevTime) / 1000;
  integral = constrain(integral, -255, 255);

  PIDout = KP * error + KI * integral - KD * derivative;
  speed = constrain(PIDout, -255, 255);

  Serial.print("KP = ");
  Serial.print(KP);
  Serial.print(" KI = ");
  Serial.print(KI);
  Serial.print(" KD = ");
  Serial.print(KD);


  motorControl(speed, PWM1_CH);

  delay(20);
  Serial.println("");
}

void getAngles() {
  mpu.getEvent(&a, &g, &temp);
  getEulerAngles(a.acceleration);
  theta_roll = alphaFilter(roll, theta_roll);
  theta_pitch = alphaFilter(pitch, theta_pitch);
}

void setMode(int mode) {
  switch (mode) {
    case shortBrake:
      Serial.println("");
      break;
    case CW:
      digitalWrite(DIR1, LOW);
      break;
    case CCW:
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
  pitch = atan2(a.y, a.z);
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
  String inputString = Serial.readStringUntil('\n');
  return inputString.toInt();
}
