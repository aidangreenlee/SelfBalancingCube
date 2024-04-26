#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <esp32-hal-ledc.h>


#define PWM 18
#define POT 15

//int K 


void setup() {
  // put your setup code here, to run once:

   // Configure PWM channel
  ledcSetup(0, 1000, 8);  // PWM channel 0, 8-bit resolution

  // Attach PWM channel to GPIO pin
  ledcAttachPin(PWM, 0);  // Use PWM channel 0
  ledcWrite(0,10);// duty cycle


  pinMode(POT, INPUT);

  //Serial.begin(115200);


}

void loop() {
  // put your main code here, to run repeatedly:

    int val = analogRead(POT);
    int KP = map(val, 0, 4095, 1, 1000);
    //Serial.println(KP);
    //ledcWriteTone(0,KP);
    ledcSetup(0,KP,8);

    delay(20);
    //ledc_set_freq()



}
