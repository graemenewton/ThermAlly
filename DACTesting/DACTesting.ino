     
const int lowPin = 2;
const int DACInPin = 23; // DIG PIN 23 for bnc in       
const int APin = A0;
float AFloat;
float voltage = 0;
void setup() {

  pinMode(DACInPin, INPUT);
  pinMode(lowPin, OUTPUT);
  Serial.begin(9600);

  analogReadResolution(12);
  analogWriteResolution(12);
}

void loop() {
  digitalWrite(lowPin, LOW);
  AFloat = analogRead(0);
  voltage = (AFloat*3.3/4095);
Serial.print(digitalRead(DACInPin)); Serial.print(" "); Serial.print(analogRead(APin)); Serial.print(" "); Serial.println(voltage);
delay(100);


}

//digital read displays as "1" aka HIGH, when pin voltage in is above 1.4V (approx 438 on integer scale)
//tested level shifter, works nicely. The Voltage put out is the same as that supplied to the VccA and VccB pins, so each side will put out whatever the main pins are.
// essentially this means any TTL voltage can be supplied to the 5v side, and 3.3v will be supplied to the arduino.
//however, ttl out must be either 3.3v or 5v. 2 DAC outs? one at each? to let user choose 3.3v or 5v ttl to hook up to?
