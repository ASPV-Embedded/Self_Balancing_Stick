void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  int analogValue;
  float voltage;

  analogValue = analogRead(A0);
  voltage = analogValue/204.6;

  Serial.println(voltage);

  //delay(10);

//pinMode(3, OUTPUT);
//  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
//  TCCR2B = _BV(CS22);
//  OCR2A = 180;
//  OCR2B = 50;

  
}
