const int PWMPin = 8;      
const int PotPin = A0;   // potentiometer connected to analog pin 0
int PWMValue = 0;         

//tested with 9V battery. 3.3V PWM from arduino can drive. Whole breakoutboard can be supplied with 3.3v and work fine.


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
