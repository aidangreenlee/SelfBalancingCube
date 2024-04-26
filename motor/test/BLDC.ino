#define cbi(sfr, bit1) (_SFR_BYTE(sfr) &= ~_BV(bit1)) // clear bit
#define sbi(sfr, bit1) (_SFR_BYTE(sfr) |= _BV(bit1)) // set bit

void setup() {
  Serial.begin(9600);
  pinMode(8, OUTPUT);

  // set prescale on timer 4 to 8
  // prescale options are 1, 8, 64, 256, or 1024
  cbi(TCCR4B,CS40);
  sbi(TCCR4B,CS41);
  cbi(TCCR4B,CS42);

  // set WGM4 to 10 - allows frequency to be define with IRC4
  sbi(TCCR4B, WGM43);
  cbi(TCCR4B, WGM42);
  sbi(TCCR4A, WGM41);
  cbi(TCCR4A, WGM40);

  ICR4 = 400;
  analogWrite(8,128);
  OCR4C = 100;
}

void loop() {

}
