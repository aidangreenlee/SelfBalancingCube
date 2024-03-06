#define PWM 13
#define AIN2 11
#define AIN1 9
#define OUTA 2
#define OUTB 3

enum motorMode{shortBrake,CCW,CW,stop};



void setup() {
  // put your setup code here, to run once:
  pinMode(PWM, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(OUTA, INPUT);
  pinMode(OUTB, INPUT);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  setMode(CW, AIN1, AIN2);
  delay(5000);
  setMode(CCW, AIN1, AIN2);
  delay(5000);
  setMode(shortBrake, AIN1, AIN2);
  delay(100);
  setMode(stop, AIN1, AIN2);
  delay(2000);
  analogWrite(PWM, 255);
}

void setMode(int mode, int IN1, int IN2){
  switch(mode){
    case shortBrake:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
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
