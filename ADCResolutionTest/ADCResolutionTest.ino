      
//Arduino Due does not have an analog output voltage from 0 V to Vref, but from 1/6 to 5/6 of the reference. Therefore, DAC can put out 0.55-2.75V.
// this means the range is 2.2v, with ADC resolution of 2.2 /4095 = 0.5372 mV. I will use an LM358 Operational Amplifier to get rid of offset and amplify, 1.5x to
// give a range of 0-3.3V.

const int PotPin = A0;   // potentiometer connected to analog pin 0
int DACOut = 0; 
const int DACInPin = A1;
float DACIn;
        


void setup() {

pinMode(12, OUTPUT);
digitalWrite(12, LOW);

analogReadResolution(12);
analogWriteResolution(12);
  Serial.begin(9600);
}

void loop() {
  DACOut = analogRead(PotPin);  // read the input pin
  analogWrite(DAC0, DACOut);
  DACIn = analogRead(DACInPin);
   
Serial.print("Pot:" ); Serial.print(analogRead(PotPin)); Serial.print(" "); Serial.print("DACOut: "); Serial.print(DACOut); Serial.print(" "); 
Serial.print("DACIn: "); Serial.println(analogRead(DACInPin));
delay(100);
}
