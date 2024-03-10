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


// Alpha-beta filter values
// Position and velocity are both assumed to be zero initially
float theta_k[2] = {0,0};
float theta_dot_k[2] = {0,0};
float alpha = .1;
float beta = .1;
const int T_int = 50; // Integration period in ms

// IMU functions;
sensors_event_t a, g, temp;
Adafruit_MPU6050 mpu;

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
  if (!mpu.begin()){
    Serial.println("Failed to find MPU6050 chip");
    while(1){
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void loop() {
  // Read pin values
  potValue = analogRead(POT);
  alphaValue = analogRead(ALPHA_POT);
  betaValue = analogRead(BETA_POT);

  // remap alpha/beta tuning potentiometers
  alpha = log10(9.0 * (alphaValue / 1023.0) + 1.0);
  beta = log10(9.0 * (betaValue / 1023.0) + 1.0);

  // get imu data and calculate angles
  mpu.getEvent(&a, &g, &temp);
  float *angles = getEulerAngles(a.acceleration);
  
  // Filter data
  // if ((millis() - time) > T_int){
    //alphaBetaFilter(angles[0],g.gyro.pitch, &theta_k[0], &theta_dot_k[0]);
    alphaFilter(angles[0], &theta_k[0]);
    Serial.print(" ");
    //alphaBetaFilter(angles[1],g.gyro.roll, &theta_k[1], &theta_dot_k[1]);
    alphaFilter(angles[1], &theta_k[1]);
    Serial.print(" ");
    Serial.print(alpha);
    Serial.print(" ");
    // Serial.print(beta);
    // time = millis();
  // }

  // set motor speed using the filtered output
  // TODO: implement PID control
  speed = round(theta_k[0] / (pi/2) * 255.0);
  speed = constrain(speed, -255, 255);
  motorControl(speed, PWM);

  // free memory from Euler angle function
  free(angles);
  Serial.println(" ");

}

// Function to set motor direction based on enum
void setMode(int mode){
  switch(mode){
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
void calibrateSensors(){
  Serial.println("Calibrating Sensors...");
}

// return array of Euler angles
// ZYX / yaw pitch roll convention
// only returns pitch/roll because that's what we can
// get from the acceleromter
float *getEulerAngles(sensors_vec_t a){
  float *angles = malloc(sizeof(float) * 2);
  // angles[0]  = atan2( a.y, sqrt(a.x * a.x + a.z * a.z));
  angles[0] = atan2(a.y,a.z);
  angles[1] = atan2(-a.x, sqrt(a.x * a.x + a.z * a.z));
  return angles;
}

// alpha-beta filter assumes constant velocity over integration time
// may need a kalman filter to account for velocity
void alphaBetaFilter(float xm, float vm, float *xk, float *vk){

  float xk_next = *xk + (*vk * T_int);
  float vk_next = *vk;

  float rk = xm - xk_next;

  xk_next += alpha * rk;
  vk_next += (beta * rk) / T_int;

  *xk = xk_next;
  *vk = vk_next;
  Serial.print(*xk*180/pi);
}

// alpha filter:does not use gyro velocities
void alphaFilter(float xm, float *xk){
  float rk = xm - *xk;
  *xk = *xk + alpha * rk;
  Serial.print(*xk*180/pi);
}

void motorControl(int speed, int motorPIN){
  if (speed < 0){
    setMode(CCW);
    speed = -speed;
  }else{
    setMode(CW);
  }
  analogWrite(motorPIN, speed > 255 ? 255 : speed);
}