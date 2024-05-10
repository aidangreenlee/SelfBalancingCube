#include <Arduino.h>
#include <Wire.h>

const int YOUR_PWM_PIN = 16;    // Replace with your GPIO pin number
const int MOTOR_FREQUENCY = 1000;  // Motor speed in Hz

// Define the direction pin
const int DIRECTION_PIN = 17;  // Replace with your GPIO pin number

void setup() {
  // Configure PWM channel
  ledcSetup(0, MOTOR_FREQUENCY, 8);  // PWM channel 0, 8-bit resolution

  // Attach PWM channel to GPIO pin
  ledcAttachPin(YOUR_PWM_PIN, 0);  // Use PWM channel 0

  // Set direction pin as OUTPUT
  pinMode(DIRECTION_PIN, OUTPUT);
}

void setMotorSpeed(int frequency) {
  // Set PWM frequency
  ledcWriteTone(0, frequency);
}

void loop() {
  // Set motor direction to clockwise (connected to positive terminal)
  digitalWrite(DIRECTION_PIN, HIGH);

  // Set motor speed to half speed (500 Hz)
  setMotorSpeed(1000);
  delay(1000);  // Delay for 1 second

  // Set motor direction to counterclockwise (hanging)
  digitalWrite(DIRECTION_PIN, LOW);

  // Set motor speed to full speed (1000 Hz)
  setMotorSpeed(1000);
  delay(1000);  // Delay for 1 second
}
