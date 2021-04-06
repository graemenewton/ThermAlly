/*script successfully writes a voltage to the DAC which reads accurately on the A0
pin with respecto to the commanded voltage*/
 

#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

#define DAC_RESOLUTION (9)

const int DACInPin = A0; // DIG PIN 23 for bnc in       
float AFloat;
float voltage = 0;


void setup() {

  pinMode(DACInPin, INPUT);
  Serial.begin(9600);
  Serial.print("Hello World!");

  analogReadResolution(12);
  analogWriteResolution(12);

  dac.begin(0x63);

  delay(500);
}

void loop() {


dac.setVoltage(4095/2, false);
  AFloat = analogRead(DACInPin);
  voltage = (AFloat*3.3/4095);
Serial.print(digitalRead(DACInPin)); Serial.print(" "); Serial.print(analogRead(DACInPin)); Serial.print(" "); Serial.println(voltage);
delay(1);


}
