#include <Adafruit_MPU6050.h>

#define PWM 13
#define AIN2 11
#define AIN1 9
#define OUTA 2
#define OUTB 3
#define POT A15
#define ALPHA_POT A8
#define BETA_POT A9

// define physical constants
const float pi = 3.14159265359;

enum motorMode{shortBrake,CCW,CW,stop};
int potValue = 0;
int alphaValue = 0;
int betaValue = 0;
int speed = 0;
int time = 0;

struct calibrate {
  sensors_vec_t gyro;
  sensors_vec_t acceleration;
};

struct calibrate Calibration;


// Alpha-beta filter values
// Position and velocity are both assumed to be zero initially
float theta_k[2] = { 0, 0 };
float theta_dot_k[2] = { 0, 0 };
float alpha = .1;
float beta = .1;
const int T_int = 50;  // Integration period in ms

// IMU functions;
sensors_event_t a, g, temp;
Adafruit_MPU6050 mpu;

// PID Parameters
double KP = 0.0; // Gains -tune later
float KI = 0.0; 
float KD = 0.0;
float setpoint = 0; // 0 degrees?
float output = PID(setpoint,KP,KI,KD);

void setup() {
  // setup pins
  pinMode(PWM, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(OUTA, INPUT);
  pinMode(OUTB, INPUT);
  pinMode(POT, INPUT);
  pinMode(ALPHA_POT, INPUT);
  pinMode(BETA_POT, INPUT);
  Serial.begin(115200);

  // Initialize IMU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  calibrateSensors();

}

void loop() {
  // Read pin values
  potValue = analogRead(POT);
  alphaValue = analogRead(ALPHA_POT);
  betaValue = analogRead(BETA_POT);

  // remap alpha/beta tuning potentiometers
  alpha = log10(9.0 * (alphaValue / 1023.0) + 1.0);
  beta = log10(9.0 * (betaValue / 1023.0) + 1.0);

  getAngles();
  // set motor speed using the filtered output
  // TODO: implement PID control
  speed = round(theta_k[0] / (pi / 4) * 255.0);
  speed = constrain(speed, -255, 255);
  motorControl(speed, PWM);

  // Serial.println(" ");
}

void getAngles(){
  // get imu data and calculate angles
  mpu.getEvent(&a, &g, &temp);
  float *angles = getEulerAngles(a.acceleration);
  // Filter data
  //alphaBetaFilter(angles[0],g.gyro.pitch, &theta_k[0], &theta_dot_k[0]);
  alphaFilter(angles[0], &theta_k[0]);
  // Serial.print(" ");
  //alphaBetaFilter(angles[1],g.gyro.roll, &theta_k[1], &theta_dot_k[1]);
  alphaFilter(angles[1], &theta_k[1]);

  // Serial.print(" ");
  // Serial.print(alpha);
  // Serial.print(" ");
  // Serial.print(beta);

  // free memory from Euler angle function
  free(angles);
}

// Function to set motor direction based on enum
void setMode(int mode) {
  switch (mode) {
    case shortBrake:
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, HIGH);
      break;
    case CCW:
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      break;
    case CW:
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      break;
    case stop:
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      break;
  }
}

// Calibration function for IMU position
// Set target accelerations and truth down
void calibrateSensors() {
  float caltime = millis();
  float acc[3] = {0, 0, 0};
  float gyro[3] = {0, 0, 0};
  int c = 0;

  // Take average over 1 second window to collect biases
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
  Serial.println("Done Calibrating");

  // Set biases in Calibration struct
  Calibration.gyro.x = g.gyro.x/c;
  Calibration.gyro.y = g.gyro.y/c;
  Calibration.gyro.z = g.gyro.z/c;
  Calibration.acceleration.x = a.acceleration.x/c;
  Calibration.acceleration.y = a.acceleration.y/c;
  Calibration.acceleration.z = a.acceleration.z/c;
}

// return array of Euler angles
// ZYX / yaw pitch roll convention
// only returns pitch/roll because that's what we can
// get from the acceleromter
float *getEulerAngles(sensors_vec_t a) {
  float *angles = malloc(sizeof(float) * 2);
  // angles[0]  = atan2( a.y, sqrt(a.x * a.x + a.z * a.z));
  angles[0] = atan2(a.y, a.z);
  angles[1] = atan2(-a.x, sqrt(a.x * a.x + a.z * a.z));
  return angles;
}

// alpha-beta filter assumes constant velocity over integration time
// may need a kalman filter to account for velocity
void alphaBetaFilter(float xm, float vm, float *xk, float *vk) {

  float xk_next = *xk + (*vk * T_int);
  float vk_next = *vk;

  float rk = xm - xk_next;

  xk_next += alpha * rk;
  vk_next += (beta * rk) / T_int;

  *xk = xk_next;
  *vk = vk_next;
  // Serial.print(*xk * 180 / pi);
}

// alpha filter:does not use gyro velocities
void alphaFilter(float xm, float *xk) {
  float rk = xm - *xk;
  *xk = *xk + alpha * rk;
  // Serial.print(*xk * 180 / pi);
}

void motorControl(int speed, int motorPIN) {
  if (abs(speed) < 10){
    setMode(stop);
  } else if (speed < 0) {
    setMode(CCW);
    speed = -speed;
  } else {
    setMode(CW);
  }

  analogWrite(motorPIN, speed > 255 ? 255 : speed);
}
