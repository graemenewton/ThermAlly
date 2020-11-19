//something i noticed is that if you use integers in arithmetic, it doesn't work.
//i.e. the TempPotValue from A0, which is a 0-1023 integer mjst be stored as a 
// float to allow for mathermatical operations


const int TempPotPin = A0;
float TempPotValue;
float TempPotScaled;
float TempPot;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

TempPotValue = analogRead(TempPotPin);
TempPotScaled = (TempPotValue/1023);
TempPot = (TempPotScaled*50);

Serial.print(TempPot); Serial.print(" ");
Serial.print(TempPotScaled); Serial.print(" ");
Serial.print(TempPotValue/1023*50); Serial.print(" "); Serial.println(TempPotValue);


delay(100);
}
