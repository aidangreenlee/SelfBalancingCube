#include <Adafruit_MPU6050.h>

#define PWM 13
#define AIN2 11
#define AIN1 9
#define OUTA 3 //encoder A
#define OUTB 2 //encoder B
#define POT_KP A0
#define POT_KI A1
#define POT_KD A2

// define physical constants
const float pi = 3.14159265359;

enum motorMode{shortBrake,CCW,CW,stop};
int kp_val = 0;
int ki_val = 0;
int kd_val = 0; 
int speed = 0;
int time = 0;

struct calibrate {
  sensors_vec_t gyro;
  sensors_vec_t acceleration;
};

struct calibrate Calibration;


// Alpha-beta filter values
// Position and velocity are both assumed to be zero initially
float roll = 0;
float theta_roll;
float pitch = 0;
float theta_pitch;
float roll_dot = 0;
float pitch_dot = 0;
float alpha = .2;
float beta = .1;
const int T_int = 50;  // Integration period in ms

// IMU functions;
sensors_event_t a, g, temp;
Adafruit_MPU6050 mpu;

// PID Parameters
double KP = 1000; // Gains -tune later
float KI = 0; 
float KD = 0;
float setpoint = -0.835; // 0 degrees?
float integral = 0;
float error = 0;
float previousError = 0;
float derivative = 0;
float PIDout = 0;

int encoder_count = 1;

void setup() {
  // setup pins
  pinMode(PWM, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(OUTA, INPUT);
  pinMode(OUTB, INPUT);
  pinMode(POT_KP, INPUT);
  pinMode(POT_KI, INPUT);
  pinMode(POT_KD, INPUT);
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


  //could  use encoder to correct for imu drift as a feedback mechanism
  attachInterrupt(digitalPinToInterrupt(OUTA),encoder_func,RISING);

}

void loop() {
  // Read pin values
  kp_val = analogRead(POT_KP);
  ki_val = analogRead(POT_KI);
  kd_val = analogRead(POT_KD);

  KP = map(kp_val,0,1023,1,1000);
  KI = map(ki_val,0,1023,0,5000);
  KD = map(kd_val,0,1023,1,10000);


  // remap alpha/beta tuning potentiometers
  // alpha = log10(9.0 * (alphaValue / 1023.0) + 1.0);
  //beta = log10(9.0 * (betaValue / 1023.0) + 1.0);

  getAngles();
 

   // Calculate PID output
  error = setpoint - roll; // Calculate error between setpoint and pitch
  if (abs(error) <= 0.087){ // set 5 degree dead zone for wheel to mitigate effects of gear backlash
    error = 0;
    speed = 0;
    Serial.println("zero error");
  }else{
  //Serial.println(error); 
  previousError = error; // Save error for next iteration
  // integral += error * T_int; // Update integral
 // integral = constrain(integral,-100,100);
  // float derivative = (error - previousError) / T_int; // Calculate derivative
  derivative = g.gyro.y;
  PIDout = KP * error + KI * integral + KD * derivative; // Calculate PID output
  // Serial.print("error:");
  // Serial.print(error);
  // Serial.print(" KP:");
  // Serial.print(KP);
  // Serial.print(" output:");
  // Serial.print(output);

  // set motor speed using the PID output
  speed = constrain(PIDout, -255, 255); // Ensure speed is within limits
  // Serial.print(" speed:");
  // Serial.print(speed);
  // Serial.print("-2 2 ");
  // Serial.print(alpha);
  // Serial.print(" ");
  // Serial.print(theta_k[0]);
  // Serial.print(" ");
  // Serial.println(theta_k[1]); 
  }

  motorControl(speed, PWM);



  // set motor speed using the filtered output 
  // TODO: implement PID control
  // speed = round(theta_k[0] / (pi / 4) * 255.0);
  // speed = constrain(speed, -255, 255);
  // motorControl(speed, PWM);

  // Serial.println(" ");
  // Serial.println("loop");
}

void getAngles(){
  // get imu data and calculate angles
  mpu.getEvent(&a, &g, &temp);
  getEulerAngles(a.acceleration);
  // Filter data
  //alphaBetaFilter(angles[0],g.gyro.pitch, &theta_k[0], &theta_dot_k[0]);
  theta_roll = alphaFilter(roll, theta_roll);
  // Serial.print(" ");
  //alphaBetaFilter(angles[1],g.gyro.roll, &theta_k[1], &theta_dot_k[1]);
  theta_pitch = alphaFilter(pitch, theta_pitch);
  // Serial.print(" ");
  // Serial.print(alpha);
  // Serial.print(" ");
  // Serial.print(beta);
  // Serial.println(theta_roll);
  // free memory from Euler angle function
  // Serial.println("getAngles");
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
    // case stop:
    //   digitalWrite(AIN1, LOW);
    //   digitalWrite(AIN2, LOW);
    //   break;
  }
  // Serial.println("setMode");
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

  // Set biases in Calibration struct
  Calibration.gyro.x = g.gyro.x/c;
  Calibration.gyro.y = g.gyro.y/c;
  Calibration.gyro.z = g.gyro.z/c;
  Calibration.acceleration.x = a.acceleration.x/c;
  Calibration.acceleration.y = a.acceleration.y/c;
  Calibration.acceleration.z = a.acceleration.z/c;
  
  Serial.println("Done Calibrating");
  Serial.print("Calibrated Roll = ");
  Serial.println(Calibration.gyro.x);
  delay(1000);
}

// return array of Euler angles
// ZYX / yaw pitch roll convention
// only returns pitch/roll because that's what we can
// get from the acceleromter
void getEulerAngles(sensors_vec_t a) {
  pitch = atan2(a.y, a.z); //pitch
  roll = atan2(a.x, a.z);
  // Serial.println("getEulerAngles");
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
float alphaFilter(float xm, float xk) {
  float rk = xm - xk;
  xk = xk + alpha * rk;
  //Serial.println(*xk * 180 / pi);
  // Serial.println("alphaFilter");
  return xk;
}

void motorControl(int speed, int motorPIN) {
  // if (abs(speed) < 10){
  //   setMode(stop);
  // }
    if (speed < 0) {
    setMode(CW);
    speed = -speed;
  } else {
    setMode(CCW);
  }

  analogWrite(motorPIN, speed > 255 ? 255 : speed);
  //Serial.println(speed);
  // Serial.println("motorControl");
}


void encoder_func() {

//If encoder A > encoder B, increment. 
//This function is called on rising edge of A, so increment rising and decrement falling

if (digitalRead(OUTA) > digitalRead(OUTB)) { 
  encoder_count++;
}
else{
    encoder_count--;
  }

}
