
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <esp32-hal-ledc.h>
#include <esp_now.h>
#include <WiFi.h>
#include "RGB.h"

#define CHANNEL 0
#define BRAKE 27


#define DIR3 33
#define PWM3 32
#define PWM3_CH 1

#define DIR2 26
#define PWM2 25
#define PWM2_CH 0

#define DIR1 12
#define PWM1 14
#define PWM1_CH 1

#define BLUE 1
#define RED 5
#define GREEN 15


const int SpeakerPin = 4;  // 16 corresponds to GPIO4

// setting PWM properties for speaker
const int freq = 1000;
const int speakerChannel = 3;
const int resolution = 8;

enum motorMode {CCW, CW };



#define TIMER_BIT 8
#define MOTOR_FREQUENCY 20000
#define POT_KP 27
#define POT_KI 26
#define POT_KD 32
#define AIN1 18
#define AIN2 19

sensors_event_t a, g, temp;
Adafruit_MPU6050 mpu;

const float pi = 3.14159265359;

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
const int T_int = 10;

float KP;
float KI;
float KD;

// float setpoint = 2.31;
// float SETPOINT = 2.31;
float setpoint = 0;
float SETPOINT = 0;
float ditherAngle = 1.5;
float integral = 0;
float error = 0;
float previousError = 0;
float derivative = 0;
float PIDout = 0;
int time_ = 0;
int prevTime = 0;
int time_delta = 0;

bool DISCOMODE = false;

unsigned long balancingStartTime;

void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

int receivedKP;
int receivedKI;
int receivedKD;

// Function for parsing bluetooth commands
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
 
  // Allocate memory for the identifier
  char identifier[4];  // Assuming the maximum length of the identifier is 3 characters

  // Copy the identifier from the received data
  strncpy(identifier, (char *)data, 3);
  identifier[3] = '\0';  // Ensure null termination

  if (strncmp(identifier, "KP=", 3) == 0) {
    sscanf((char *)data, "KP=%f", &KP);
    Serial.print("Received KP: ");
    Serial.println(KP);
  } else if (strncmp(identifier, "KI=", 3) == 0) {
    sscanf((char *)data, "KI=%f", &KI);
    integral = 0;
    Serial.print("Received KI: ");
    Serial.println(KI);
  } else if (strncmp(identifier, "KD=", 3) == 0) {
    sscanf((char *)data, "KD=%f", &KD);
    Serial.print("Received KD: ");
    Serial.println(KD);
  } else if (strncmp(identifier, "CAL", 3) == 0) {
    Serial.println("Calibrating");
    integral = 0;
    calibrateSensors();
  } else if (strncmp(identifier, "DT=", 3) == 0) {
    sscanf((char *)data, "DT=%f", &ditherAngle);
    Serial.print("Received DT: ");
    Serial.println(ditherAngle);
  } else if (strncmp(identifier, "STP", 3) == 0) {
    Serial.println("Stopping Motors: KP = KI = KD = 0");
    KP = 0;
    KI = 0;
    KD = 0;
    Serial.print("Setpoint: ");
    Serial.println(SETPOINT);
  } else if (strncmp(identifier, "DIS", 3) == 0){
    if (DISCOMODE){
      Serial.println("Deactivating Party");
      DISCOMODE = false;
    } else {
      Serial.println("Activating Party");
      DISCOMODE = true;
    }
  } else if (strncmp(identifier, "SP=", 3) == 0){
    Serial.print("Changing setpoint: ");
    sscanf((char*)data, "SP=%f", &SETPOINT);
    Serial.println(SETPOINT);
    // Serial.println(*data);
    setpoint = SETPOINT;
  }
   
}

void setup() {

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  analogWrite(RED, 255);
  analogWrite(GREEN, 255);
  analogWrite(BLUE, 255);

  // For Speaker
    // configure LED PWM functionalitites
  ledcSetup(speakerChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(SpeakerPin, speakerChannel);

  // To know we just reset/powered on
  ledcWriteTone(speakerChannel,5000);
  delay(350);
  ledcWriteTone(speakerChannel, 0);   

  // Motor configuration
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  pinMode(DIR3, OUTPUT);
  ledcSetup(PWM3_CH, MOTOR_FREQUENCY, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  motorControl(0, PWM3_CH);

  pinMode(DIR2, OUTPUT);
  ledcSetup(PWM2_CH, MOTOR_FREQUENCY, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  motorControl(0, PWM2_CH);

  pinMode(DIR1, OUTPUT);
  ledcSetup(PWM1_CH, MOTOR_FREQUENCY, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  motorControl(0, PWM1_CH);

  Serial.begin(115200);

  // Check MPU6050 found
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  calibrateSensors();

  delay(500);

  time_ = millis();
  prevTime = time_;

  // Set up connection between two ESP-32s
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

  // Dithering
  if (roll >= SETPOINT) {
    setpoint -= 0.05 * time_delta;
  } else {
    setpoint += 0.05 * time_delta;
  }
    setpoint = constrain(setpoint, SETPOINT - (ditherAngle * pi / 180.0), SETPOINT + (ditherAngle * pi / 180.0));

  error = setpoint - theta_roll;

  previousError = error;

  // filter gyro
  float gyro_y_filtered = alphaFilter(g.gyro.y, roll_dot);

  // Update roll_dot with filtered gyro reading
  roll_dot = gyro_y_filtered;

  derivative = roll_dot;
  integral += error * (time_ - prevTime) / 1000;
  integral = constrain(integral, -255, 255);


  PIDout = KP * error + KI * integral + KD * derivative;
  speed = constrain(PIDout, -255, 255);


  // Serial.print(" Pitch: ");
  // Serial.print(theta_pitch);
  // Serial.print(" Roll: ");
  // Serial.print(theta_roll);

  // Serial.print(" Setpoint: ");
  // Serial.print(setpoint);

  // Serial.print(" Gyro X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(" Gyro Y: ");
  // Serial.print(g.gyro.y);


  // Serial.print("KP = ");
  // Serial.print(KP);
  // Serial.print(" KI = ");
  // Serial.print(KI);
  // Serial.print(" integral = ");
  // Serial.print(integral);
  // Serial.print(" KD = ");
  // Serial.print(KD);
  // Serial.print(" Speed = ");
  // Serial.print(speed);

  motorControl(speed, PWM1_CH);
  //delay(100);
  motorControl(speed, PWM2_CH);

  if (DISCOMODE){
    Disco();
  }

  delay(10);
  // Serial.println("");
}

void getAngles() {
  mpu.getEvent(&a, &g, &temp);
  getEulerAngles(a.acceleration);
  theta_roll = alphaFilter(roll, theta_roll);
  theta_pitch = alphaFilter(pitch, theta_pitch);
}

void setMode(int mode) {
  switch (mode) {
    case CW:
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, HIGH);
      break;
    case CCW:
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, LOW);
      break;
  }
}

void calibrateSensors() {
  float caltime = millis();
  float acc[3] = { 0, 0, 0 };
  float gyro[3] = { 0, 0, 0 };
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

  // Beep when done
  ledcWriteTone(speakerChannel, 1000);   
  delay(350);
  ledcWriteTone(speakerChannel, 0);   
  Serial.println("Done Calibrating");
  delay(500);  
  getAngles();
  delay(500);  
  setpoint = theta_roll;
  SETPOINT = theta_roll;
  Serial.println("Setpoint: ");
  Serial.println(theta_roll);


}

void getEulerAngles(sensors_vec_t a) {
  pitch = atan2(a.y, a.z);
  roll = atan2(a.x, a.z);
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
  pwmSet(motorPIN, speed > 255 ? 255 : 255 - speed);
  // speed2RGB(speed);
}

void Disco(){
  int r, g, b;
  motorControl(10, PWM2_CH);

  while (DISCOMODE){

      // fade from blue to violet
      for (r = 0; r < 256; r++) { 
        analogWrite(RED, r);
        delay(2);
      } 
      // fade from violet to red
      for (b = 255; b > 0; b--) { 
        analogWrite(BLUE, b);
        delay(2);
      } 
      // fade from red to yellow
      for (g = 0; g < 256; g++) { 
        analogWrite(GREEN, g);
        delay(2);
      } 
      // fade from yellow to green
      for (r = 255; r > 0; r--) { 
        analogWrite(BLUE, r);
        delay(2);
      } 
      // fade from green to teal
      for (b = 0; b < 256; b++) { 
        analogWrite(BLUE, b);
        delay(2);
      } 
      // fade from teal to blue
      for (g = 255; g > 0; g--) { 
        analogWrite(GREEN, g);
        delay(2);
      }
  }
}

void speed2RGB(int speed){
  analogWrite(RED, RGB_LUT[speed]);
  analogWrite(GREEN, RGB_LUT[speed * 3 + 1]);
  analogWrite(BLUE, RGB_LUT[speed * 3 + 2]);
}
/// Unused Code
// void alphaBetaFilter(float xm, float vm, float *xk, float *vk) {
//   float xk_next = *xk + (*vk * T_int);
//   float vk_next = *vk;
//   float rk = xm - xk_next;
//   xk_next += alpha * rk;
//   vk_next += (beta * rk) / T_int;
//   *xk = xk_next;
//   *vk = vk_next;
// }