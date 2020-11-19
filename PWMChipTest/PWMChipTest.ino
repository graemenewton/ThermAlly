const int PWMPin = 8;      
const int PotPin = A0;   // potentiometer connected to analog pin 0
int PWMValue = 0;         

//capacitor from IC pin 6 to 2 is essential for getting fans going, was using 1uF. R1 = 100k, R2 = 1K. when R1 was changed to 10k, and thus R1/R2 ratio decreased to 10,
//the LED became dimmer. Adding C1 means LED come on high brightness and loses adjustability. Likely not issue with higher current fans.

//30/10/2020 - noted that a 630 pF capacitor does good job

void setup() {
  pinMode(PWMPin, OUTPUT);  // sets the pin as output
  Serial.begin(9600);
}

void loop() {
  PWMValue = analogRead(PotPin);  // read the input pin
  analogWrite(PWMPin, PWMValue/4); // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
Serial.print(analogRead(PotPin)); Serial.print(" "); Serial.println(PWMValue/4);
delay(100);
}
