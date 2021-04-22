//Useful notes: Arduino due uses 3.3V logic, digital pins output 130mA.
// 10 bits resolution returns values between 0-1023, 8 bits resolution returns values between 0-255, 12 bits resolution returns values between 0-4096


//green LED indicates that heating is enabled
//yellow led indicates that device is busy, i.e. selectin temperature or doing a ramp
//red indicates not ready for operation

//direct from MCP_9600 github, converstion to celcius is by *0.0625
/* float Adafruit_MCP9600::readThermocouple(void) {
  // if (!enabled()) {
  // return NAN; }

  // define the register
  // Adafruit_I2CRegister therm_reg =
  //   Adafruit_I2CRegister(i2c_dev, MCP9600_HOTJUNCTION, 2, MSBFIRST);

  // read a signed 16 bit value
  // int16_t therm = therm_reg.read();

  // convert to floating and shift to celsius
  // float temp = therm;
  // temp *= 0.0625; // 0.0625*C per LSB!
  // return temp;
*/



/* Libraries */
#include <Wire.h> //This library allows you to communicate with I2C / TWI devices
#include <Adafruit_I2CDevice.h> //
#include <Adafruit_I2CRegister.h>
#include <Adafruit_MCP4725.h> //include the repository for the DAC
#include <Adafruit_MCP9600.h> //include the repository for the thermocuple amplifiers
#include <LiquidCrystal.h> //allows for easy interfacing with LCD display using digital pins. Requires no I2C adapter.

/* Analog Pins */
const int PotPin1 = A0; //10 turn potentiometer
const int PotPin2 = A1; //3 turn potentiometer


/* Digital Pins */
//const int sets constant integers, such as pin numbers.

//PWM Pins
const int HeatPWMPin = 2;
const int FanPWMPin = 3;
const int PeltierLPWMPin = 4;
const int PeltierRPWMPin = 5;
const int TempLEDRedPin = 6;
const int TempLEDGreenPin = 7;
const int TempLEDBluePin = 8;
//LCD
const int LCDRS = 22;
const int LCDRW = 23;
const int LCDEnable = 24;
const int LCDD4 = 25;
const int LCDD5 = 26;
const int LCDD6 = 27;
const int LCDD7 = 28;
//Button Signals
const int GreenButtonSignalPin = 30;
const int RedButtonSignalPin = 31;
const int YellowButtonSignal1Pin = 32;
const int YellowButtonSignal2Pin = 33;
const int YellowButtonSignal3Pin = 34;
const int YellowButtonSignal4Pin = 35;
//Peltier Enable Signals
const int PeltierLEnablePin = 36;
const int PeltierREnablePin = 37;
//LEDs
const int RedLEDPin = 45;
const int GreenLEDPin = 46;
const int YellowLEDPin = 47;
//Button LEDs
const int GreenButtonLEDPin = 48;
const int RedButtonLEDPin = 49;
const int YellowButtonLED1Pin = 50;
const int YellowButtonLED2Pin = 51;
const int YellowButtonLED3Pin = 52;
const int YellowButtonLED4Pin = 53;



/* Variables */
//Integers
int TempLEDRedPWM; //int set integers (whole numbers) which can be masked for PWM outs and ADC reads.
int TempLEDGreenPWM;
int TempLEDBluePWM;
int LEDBrightness;
int BaselineTempInteger;
int BaselineTempState = 0;
int GreenButtonSwitchState;
int DACTempOutput;
int HeatEnable = 0;
int HeatRampSetupState = 0;
int HeatRampState = 0;
int HeatRampTempInteger;


//Floats
float T1Temp; //T1Temp is heat exchanger, store as float T1Temp //note that floats must be used in equations as dividing integers will cause errors as they
//cannot have decimal points.
float T2Temp; //T2Temp is bath, store as float T2Temp // e.g. 7/2 is 3.5 but this will not work with integer notation
float TempVoltage;
float DACTempVoltage;
float MaxDACTemp = 50.00; //max temp is 50C, this controls the DAC resolution. 0-50C will be split into a 12 bit scale (4096). Resolution of 0.012C for every number.

float BaselineTemp = 32.0;
float BaselineTempMin;
float BaselineTempMax;
float HeatRampTemp;

unsigned long CurrentTime = 0;
unsigned long PreviousDisplayTime = 0;
unsigned long DisplayInterval = 100;
unsigned long PreviousLEDTime = 0;
unsigned long LEDInterval = 100;
unsigned long PreviousHeatTime = 0;
unsigned long HeatInterval = 10;
unsigned long PreviousHeatEnableTime = 0;
unsigned long HeatEnableInterval = 50;
unsigned long PreviousHeatRampSetupTime = 0;
unsigned long HeatRampSetupInterval = 50;
unsigned long PreviousHeatRampTime = 0;
unsigned long HeatRampInterval = 50;
unsigned long PreviousDACTime = 0;
unsigned long DACInterval = 1;


/* Syntax for LCD pin numbers:
  LiquidCrystal(rs, enable, d4, d5, d6, d7)
  LiquidCrystal(rs, rw, enable, d4, d5, d6, d7) <------------- this one used
  LiquidCrystal(rs, enable, d0, d1, d2, d3, d4, d5, d6, d7)
  LiquidCrystal(rs, rw, enable, d0, d1, d2, d3, d4, d5, d6, d7) */

LiquidCrystal lcd(LCDRS, LCDRW, LCDEnable, LCDD4, LCDD5, LCDD6, LCDD7); //set the LCD pins (as described in above pin map)

Adafruit_MCP9600 mcp1; //define mcp1 as an adafruit mcp9600 module
Adafruit_MCP9600 mcp2; //define mcp2 as an adafruit mcp9600 module
#define TC1_Address (0x67) //thermcouple 1 is the normal adafruit mcp9600 board I2C address (0x67)
#define TC2_Address (0X66) //thermcouple 2 is has solder bridge across first addr pannel giving 0x66 I2C address

Adafruit_MCP4725 dac; //define dac as an mcp4725 module


void setup()
{

  /* Set the pin modes. */
  pinMode(PotPin1, INPUT); //10 turn potentiometer
  pinMode(PotPin2, INPUT); //3 turn potentiometer


  //PWM Pins
  pinMode(HeatPWMPin, OUTPUT);
  pinMode(FanPWMPin, OUTPUT);
  pinMode(PeltierLPWMPin, OUTPUT);
  pinMode(PeltierRPWMPin, OUTPUT);
  pinMode(TempLEDRedPin, OUTPUT);
  pinMode(TempLEDGreenPin, OUTPUT);
  pinMode(TempLEDBluePin, OUTPUT);
  //LCD Pins
  pinMode(LCDRS, OUTPUT);
  pinMode(LCDRW, OUTPUT);
  pinMode(LCDEnable, OUTPUT);
  pinMode(LCDD4, OUTPUT);
  pinMode(LCDD5, OUTPUT);
  pinMode(LCDD6, OUTPUT);
  pinMode(LCDD7, OUTPUT);
  //Button Signals
  pinMode(GreenButtonSignalPin, INPUT);
  pinMode(RedButtonSignalPin, INPUT);
  pinMode(YellowButtonSignal1Pin, INPUT);
  pinMode(YellowButtonSignal2Pin, INPUT);
  pinMode(YellowButtonSignal3Pin, INPUT);
  pinMode(YellowButtonSignal4Pin, INPUT);
  //Peltier Enable Signals
  pinMode(PeltierLEnablePin, OUTPUT);
  pinMode(PeltierREnablePin, OUTPUT);
  //LED Pins//
  pinMode(RedLEDPin, OUTPUT);
  pinMode(GreenLEDPin, OUTPUT);
  pinMode(YellowLEDPin, OUTPUT);
  //Button LED Pins
  pinMode(GreenButtonLEDPin, OUTPUT);
  pinMode(RedButtonLEDPin, OUTPUT);
  pinMode(YellowButtonLED1Pin, OUTPUT);
  pinMode(YellowButtonLED2Pin, OUTPUT);
  pinMode(YellowButtonLED3Pin, OUTPUT);
  pinMode(YellowButtonLED4Pin, OUTPUT);

  /*Set LEDs etc on for setup profile*/
  digitalWrite(RedLEDPin, HIGH);
  digitalWrite(GreenLEDPin, LOW);
  digitalWrite(YellowLEDPin, LOW);


  /* Print startup message to LCD. */
  lcd.begin(20, 4); //begin LCD communication, device has 20 coloumbs and 4 rows
  lcd.setCursor(0, 1);
  lcd.print("  Thermal Control");
  lcd.setCursor(0, 2);
  lcd.print("System Initialising");
  delay(2000);

  /* Begin serial communication at 115200 baud */

  Serial.begin(115200); //start serial communication with computer at 115200 baud rate
  while (!Serial) { //wait until serial has started (this may need to be removed if the arduino will be used without being attached to USB)
    delay(10); //slight delay to give breathing time to arduino when checking if serial connected
  }

  /* Print setup information to the LCD and serial communication port. */

  Serial.println("Checking Thermocouple"); //print that we are testing the thermcouples to check if present
  lcd.clear(); //clear lcd if anything on
  lcd.setCursor(0, 1); //set to start at upper middle left
  lcd.print("      Checking      ");
  lcd.setCursor(0, 2); //set to start at lower middle left
  lcd.print("    Thermocouples   ");
  delay(2000); //wait for 2 secs before starting new command to allow message to be on screen for long enough for user
  lcd.clear(); //clear lcd

  /* Initialise the driver with I2C_ADDRESS and the default I2C bus for TC1. */
  /* Thermcouple 1 checks. */

  if (! mcp1.begin(TC1_Address)) { //if TC1 MCP9600 not detected then message will be printed and program will pause
    Serial.println("Thermocouple 1 not found!"); //will print message in the serial monitor and pause program
    lcd.clear(); //clear lcd
    lcd.setCursor(0, 0); //Will also print it to the LCD
    lcd.print("   Thermocouple 1   ");
    lcd.setCursor(0, 1);
    lcd.print("     Not Found!     ");
    lcd.setCursor(0, 2);
    lcd.print("Check connections");
    lcd.setCursor(0, 3);
    lcd.print("then reset Arduino.");
    while (1); //infinite loop. Pauses program here.
  }

  Serial.println("Found Thermocouple 1!"); //if mcp9600 for TC1 is detected, the program will not pause and this message will be printed to the serial monitor
  lcd.clear(); //will also print to LCD
  lcd.setCursor(0, 1);
  lcd.print("       Found       ");
  lcd.setCursor(0, 2);
  lcd.print("  Thermocouple 1!");
  delay(2000); //wait 2 secs to allow user to read
  lcd.clear();
  delay(500);

  /* Initialise the driver with I2C_ADDRESS and the default I2C bus for TC2. */
  /* Thermcouple 2 checks. */

  if (! mcp2.begin(TC2_Address)) {
    Serial.println("Thermocouple 2 not found!"); //will print message in the serial monitor and pause program if MCP9600 cannot be detected
    lcd.setCursor(0, 0); //Will also print it to the LCD
    lcd.print("   Thermocouple 2   ");
    lcd.setCursor(0, 1);
    lcd.print("     Not Found!     ");
    lcd.setCursor(0, 2);
    lcd.print("Check connections");
    lcd.setCursor(0, 3);
    lcd.print("then reset Arduino.");
    while (1); //infinite loop. Pauses program here.
  }

  Serial.println("Found Thermocouple 2!"); //if MCP9600 for TC2 is detected, the program will not pause and this message will be printed to the serial monitor
  lcd.clear(); //will also print to LCD
  lcd.setCursor(0, 1);
  lcd.print("       Found       ");
  lcd.setCursor(0, 2);
  lcd.print("  Thermocouple 2!");
  delay(2000); //wait 2 secs to allow user to read
  lcd.clear();
  delay(500);

  /* Set ADC resolution for the MCP9600 modules and print to serial bus. */

  mcp1.setADCresolution(MCP9600_ADCRESOLUTION_18); // set resolution of mcp1 to 18 bits
  mcp2.setADCresolution(MCP9600_ADCRESOLUTION_18); // set resolution of mcp2 to 18 bits

  Serial.print("TC1 ADC resolution set to ");
  switch (mcp1.getADCresolution()) { //get the current ADC resolution for TC1 MCP9600
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;  //using cases means 18 will be printed rather than the full phrase MCP9600_ADCRESOLUTION_18
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bit"); //print the units, in this case bits, after the resultion

  Serial.print("TC2 ADC resolution set to ");
  switch (mcp2.getADCresolution()) { //get the current ADC resolution for TC2 MCP9600
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;  //using cases means 18 will be printed rather than the full phrase MCP9600_ADCRESOLUTION_18
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bit"); //print the units, in this case bits, after the resultion

  /* Print thermcouple types to serial bus. */

  mcp1.setThermocoupleType(MCP9600_TYPE_T); //set TC1 to T-type theromcouple
  Serial.print("TC1 Thermocouple type set to "); //print to serial monitor
  switch (mcp1.getThermocoupleType()) { //get thermcouple type setting from TC1 MCP9600 board
    case MCP9600_TYPE_K:  Serial.print("K"); break;
    case MCP9600_TYPE_J:  Serial.print("J"); break;
    case MCP9600_TYPE_T:  Serial.print("T"); break;
    case MCP9600_TYPE_N:  Serial.print("N"); break;
    case MCP9600_TYPE_S:  Serial.print("S"); break;
    case MCP9600_TYPE_E:  Serial.print("E"); break;
    case MCP9600_TYPE_B:  Serial.print("B"); break;
    case MCP9600_TYPE_R:  Serial.print("R"); break;
  }
  Serial.println(" type"); //print to serial monitor

  mcp2.setThermocoupleType(MCP9600_TYPE_T); //set TC2 to T-type theromcouple
  Serial.print("TC 2 Thermocouple type set to "); //print to serial monior
  switch (mcp2.getThermocoupleType()) { //get thermcouple type setting from TC2 MCP9600 board
    case MCP9600_TYPE_K:  Serial.print("K"); break;
    case MCP9600_TYPE_J:  Serial.print("J"); break;
    case MCP9600_TYPE_T:  Serial.print("T"); break;
    case MCP9600_TYPE_N:  Serial.print("N"); break;
    case MCP9600_TYPE_S:  Serial.print("S"); break;
    case MCP9600_TYPE_E:  Serial.print("E"); break;
    case MCP9600_TYPE_B:  Serial.print("B"); break;
    case MCP9600_TYPE_R:  Serial.print("R"); break;
  }
  Serial.println(" type"); //print to serial monitor

  /* Set filter coefficients for the thermocouples to 0 - meaning no moving average filter applied. */

  mcp1.setFilterCoefficient(0); //moving average filter coef set to 0, no filtering desired
  Serial.print("TC1 Filter coefficient value set to: ");
  Serial.println(mcp1.getFilterCoefficient());

  mcp2.setFilterCoefficient(0); //moving average filter coef set to 0, no filtering desired
  Serial.print("TC2 Filter coefficient value set to: ");
  Serial.println(mcp2.getFilterCoefficient());


  /* Print the thermocouple type, filter coefficient and ADC resolution to the LCD for the user to see. */

  lcd.setCursor(0, 0);
  lcd.print("TC1 = ");
  switch (mcp1.getThermocoupleType()) { //get thermocouple type setting from TC2 MCP9600 board
    case MCP9600_TYPE_K:  lcd.print("K"); break; //print the letter to the lcd
    case MCP9600_TYPE_J:  lcd.print("J"); break;
    case MCP9600_TYPE_T:  lcd.print("T"); break;
    case MCP9600_TYPE_N:  lcd.print("N"); break;
    case MCP9600_TYPE_S:  lcd.print("S"); break;
    case MCP9600_TYPE_E:  lcd.print("E"); break;
    case MCP9600_TYPE_B:  lcd.print("B"); break;
    case MCP9600_TYPE_R:  lcd.print("R"); break;
  }
  lcd.print(" & "); //add & after the letter
  lcd.print("TC2 = ");
  switch (mcp2.getThermocoupleType()) { //get thermocouple type setting from TC2 MCP9600 board
    case MCP9600_TYPE_K:  lcd.print("K"); break; //print the letter to the lcd
    case MCP9600_TYPE_J:  lcd.print("J"); break;
    case MCP9600_TYPE_T:  lcd.print("T"); break;
    case MCP9600_TYPE_N:  lcd.print("N"); break;
    case MCP9600_TYPE_S:  lcd.print("S"); break;
    case MCP9600_TYPE_E:  lcd.print("E"); break;
    case MCP9600_TYPE_B:  lcd.print("B"); break;
    case MCP9600_TYPE_R:  lcd.print("R"); break;
  }
  lcd.print("."); //add .
  lcd.setCursor(0, 1);
  lcd.print("Moving AVG. Filter");
  lcd.setCursor(0, 2);
  lcd.print("Coefs = ");
  lcd.print(mcp1.getFilterCoefficient()); //print moving avg filter coef to the lcd (0 means no filter)
  lcd.print(" & ");
  lcd.print(mcp2.getFilterCoefficient()); //print moving avg filter coef to the lcd (0 means no filter)
  lcd.print(".");
  lcd.setCursor(0, 3);
  lcd.print("ADC Res. = ");
  switch (mcp1.getADCresolution()) { //get the current ADC resolution for TC1 MCP9600
    case MCP9600_ADCRESOLUTION_18:   lcd.print("18"); break;  //print adc res to the lcd
    case MCP9600_ADCRESOLUTION_16:   lcd.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   lcd.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   lcd.print("12"); break;
  }
  lcd.print(" & ");
  switch (mcp2.getADCresolution()) { //get the current ADC resolution for TC1 MCP9600
    case MCP9600_ADCRESOLUTION_18:   lcd.print("18"); break;  //print adc res to lcd
    case MCP9600_ADCRESOLUTION_16:   lcd.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   lcd.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   lcd.print("12"); break;
  }
  lcd.print(".");
  delay(5000); //wait to allow user to read
  lcd.clear();

  /* Code can be added here to set alerts. These can be used for safety features such as to shut off in the case of overheating. */

  mcp1.setAlertTemperature(1, 60); //an exmaple of how an alert can be set, e.g. for auto shut off if temps hit over 50
  Serial.print("Alert #1 temperature set to ");
  Serial.println(mcp1.getAlertTemperature(1)); //get alert 1 temperature from TC1 MCP9600
  mcp1.configureAlert(1, true, true);  // alert 1 enabled, rising temp

  /* Enable the two MCP9600 I2C devices. */

  mcp1.enable(true);
  mcp2.enable(true);

  /* Print ready to operate message and turn red light off + green light on. */

  Serial.println(F("------------------------------")); //divide temp recording data from setup info

  lcd.setCursor(0, 1);
  lcd.print("   Setup Complete!");
  lcd.setCursor(0, 2);
  lcd.print("  Ready to operate!");
  delay(1000); //slight delay before starting loop, the message will be displayed until something new is overwritten

  /* The ADC and DAC resolution is set to 10 bit by default for compatability purposes with other arduino boards, however, the Due is capable of higher resolution. */
  analogReadResolution(12); //12 bits is maximum resolution which can be run on the Arduino Due, this command will unlock the best possible ADC resolution for us.
  analogWriteResolution(12); //12 bits is maximum resolution which can be run on the Arduino Due, this command will unlock the best possible DAC/PWM resolution for us.
  // analogWrite() is 8 bits by default (0-255), changing this means it will now be 0-4095. analogRead() is 10 bits (0-1023) by default, chaning this means it
  // would now be 0-4095. The map() function is useful here, e.g. map(sensorVal, 0, 1023, 0, 4095) - this would essentially scale 10 bits to 12 bits.

  /* Code for picking the baseline temperature */
  digitalWrite(RedLEDPin, LOW);
  digitalWrite(YellowLEDPin, HIGH);
  digitalWrite(GreenButtonLEDPin, HIGH); //turn gren button LED on

  while (BaselineTempState == 0) //while the baselinetemp has not been set do the following:
  {
    BaselineTempInteger = map(analogRead(PotPin1), 0, 4095, 0 , 5000); //map the 10-turn potentiometer to 0-50 degrees C
    BaselineTemp = (float)BaselineTempInteger / 100; //pass BaselineTempInteger as a float so it can be divided into non whole numbers and store float as BaselineTemp
    lcd.clear(); //clear the LCD and print the baseline temperature annotation
    lcd.setCursor(0, 0);
    lcd.print("Baseline Temperature");
    lcd.setCursor(0, 1);
    lcd.print("        "), lcd.print(BaselineTemp);
    lcd.setCursor(0, 2);
    lcd.print(" Press Green Button");
    lcd.setCursor(0, 3);
    lcd.print("     To Confirm     ");
    if (digitalRead(GreenButtonSignalPin) == HIGH) //if the green button is pressed then:
    {
      digitalWrite(GreenButtonLEDPin, LOW);
      BaselineTempState = 1; //make the state of the baseline temperature 1, marking it as chosen, and exit the while() loop
      BaselineTempMin = (BaselineTemp - 1); //once baseline temp is selected, calculate the min
      BaselineTempMax = (BaselineTemp + 1); //once baseline temp is selected, calculate the max
    }
    delay(100); //delay used here rather than millis() as no other loops required to run while selecting the baseline temp.
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Baseline Temperature");
  lcd.setCursor(0, 1);
  lcd.print("      Selected      ");
  lcd.setCursor(0, 2);
  lcd.print("        "), lcd.print(BaselineTemp);
  Serial.print("Baseline Temperature Selected: "); Serial.print(BaselineTemp); Serial.println(" ºC");
  delay(2000);



  dac.begin(0x63); //begin communications with the MCP4725 dac module at I2C address 0x62

  digitalWrite(RedLEDPin, LOW);
  digitalWrite(YellowLEDPin, LOW);
  digitalWrite(GreenLEDPin, LOW);
  digitalWrite(YellowButtonLED1Pin, LOW);
  digitalWrite(YellowButtonLED2Pin, LOW);
  digitalWrite(YellowButtonLED3Pin, LOW);
  digitalWrite(YellowButtonLED4Pin, LOW);
}


void loop()
{

  CurrentTime = millis(); //store the millis() value as the current time


  /* Display refresh*/
  if (CurrentTime - PreviousDisplayTime > DisplayInterval) // display interval is 100ms, so: if 100ms has passed, then the lcd and serial monitors will print info
  {
    PreviousDisplayTime = CurrentTime; //update the previous display time with the current time
    TCSerialPrint(); //uses custom function to print Theromcouple paramaters to Serial port. See TCSerialPrint.ino tab
    TCLCDPrint(); //uses custom function to print Theromcouple parameters to LCD display. See TCLCDPrint.ino tab
  }

  /* Temperature LED  */
  if (CurrentTime - PreviousLEDTime > LEDInterval) // LED interval is 100ms. so if 100ms pass, then update the LED Colour
  {
    PreviousLEDTime = CurrentTime; //update the previous LED time with the current time

    /* Code below regultes the RGB PWM numbers for the RGB LED - this is used so operators can guage the temperature based on LED colour from a distance
      without having to get close to the screen. */
    LEDTempColour(); //custom function for assigning PWM integers based on bath temperature for LED colouring, function defined in LEDTempColour.ino Tab
    LEDBrightness = (4095 - analogRead(PotPin2)); //99

    //use PWM numbers set above to control colour of RGB LED.
    analogWrite(TempLEDRedPin, (map(TempLEDRedPWM, 0, 255, 0, 4095) / map(LEDBrightness, 0, 4095, 1, 10)));
    analogWrite(TempLEDGreenPin, (map(TempLEDGreenPWM, 0, 255, 0, 4095) / map(LEDBrightness, 0, 4095, 1, 10)));
    analogWrite(TempLEDBluePin, (map(TempLEDBluePWM, 0, 255, 0, 4095) / map(LEDBrightness, 0, 4095, 1, 10)));
  }


  /* Store temperatures for T1 and T2 as variables */
  T1Temp = mcp1.readThermocouple(); //T1Temp is heat exchanger, store as float T1Temp
  T2Temp = mcp2.readThermocouple(); //T2Temp is bath, store as float T2Temp

  /* DAC Output Temperature  */
  if (CurrentTime - PreviousDACTime > DACInterval) //DAC Interval is 1ms, aka 1000Hz
  {
    TempVoltage = (mcp2.readThermocouple() / MaxDACTemp); //v high resolution for DAC scaling, but limited to 0-50C temp range.
    DACTempVoltage = (TempVoltage * 4096); // 12 bit resolution, max of 3.3V as that is Vcc.
    DACTempOutput = ((int)DACTempVoltage); //pass the DACTempVoltage float as an integer and on 12 bit scale (0-4095) to set the DAC voltage
    dac.setVoltage(DACTempOutput, false); //false means the value is not saved to DAC I2C board EEPROM
  }

  /* Heat On/Off Switch*/
  if (CurrentTime - PreviousHeatEnableTime > HeatEnableInterval) //Heat switch state unterval is 50ms. SO if 50ms pass, check the heat control button
  {
    if (digitalRead(YellowButtonSignal4Pin) == HIGH) //if yellow button 4 is pressed then
    {
      if (HeatEnable == 0) //if the state was OFF, turn it ON
      {
        HeatEnable = 1; //make the state ON
        digitalWrite(YellowButtonLED4Pin, HIGH); //turn button 4 LED ON
        digitalWrite(GreenLEDPin, HIGH);
      }
      else if (HeatEnable == 1) //if the state was ON, turn it OFF
      {
        HeatEnable = 0; // make the state OFF
        digitalWrite(YellowButtonLED4Pin, LOW); //turn button 4 LED off
        digitalWrite(GreenLEDPin, LOW);
      }
    }
  }

  /* Baseline Heating  */
  if (CurrentTime - PreviousHeatTime > HeatInterval) // Heat interval is 10ms. so if 20ms pass, check the temperatures and heat accordingly
  {
    if ((T2Temp < BaselineTempMin) && (HeatEnable == 1)) // if temp is lower than minimu, heat at full power if heat is ON
    {
      analogWrite(HeatPWMPin, 4096); //100% duty cycle aka full power
    }

    else if ((T2Temp >= BaselineTempMin) && (T2Temp <= BaselineTemp) && (HeatEnable == 1)) //if temp is just below target temp, then heat with 75% power if heat is ON
    {
      analogWrite(HeatPWMPin, 3072); //75% duty cycle, aka 75 % power
    }

    else if ((T2Temp <= BaselineTempMax) && (T2Temp >= BaselineTemp) && (HeatEnable == 1))// if temp is just above target temp, then heat at 50% power if heat is ON
    {
      analogWrite(HeatPWMPin, 2048); //50% duty cycle, aka 50% power
    }

    else if ((T2Temp > BaselineTempMax) && (HeatEnable == 1)) //if temp is above the target temp, then heat at 25% power if heat in ON
    {
      analogWrite(HeatPWMPin, 1024); //25% duty cycle, aka 25% power
    }

  }

  /* Heat Ramp Temperature Selection */
  if (CurrentTime - PreviousHeatRampSetupTime > HeatRampSetupInterval) //every 50ms check button 1 and if it is pressed, begin the heat ramp setup
  {
    if (digitalRead(YellowButtonSignal1Pin) == HIGH) //if the first yellow button is pressed
    {
      digitalWrite(YellowLEDPin, HIGH); //turn the busy light on
      digitalWrite(GreenButtonLEDPin, HIGH); //turn green button LED on to indicated it can be operated
      HeatRampSetupState = 0; //set heat ramp setupstate to 0
      while (HeatRampSetupState == 0) //while the heat ramp temp has not been set do the following:
      {
        HeatRampTempInteger = map(analogRead(PotPin1), 0, 4095, 0 , 5000); //map the 10-turn potentiometer to 0-50 degrees C
        HeatRampTemp = (float)HeatRampTempInteger / 100; //pass heatramptempinteger as a float so it can be divided into non whole numbers and store float
        lcd.clear(); //clear the LCD and print the baseline temperature annotation
        lcd.setCursor(0, 0);
        lcd.print("   Heat Ramp Temp   ");
        lcd.setCursor(0, 1);
        lcd.print("        "), lcd.print(HeatRampTemp);
        lcd.setCursor(0, 2);
        lcd.print(" Press Green Button");
        lcd.setCursor(0, 3);
        lcd.print("     To Confirm     ");
        if (digitalRead(GreenButtonSignalPin) == HIGH) //if the green button is pressed then:
        {
          digitalWrite(GreenButtonLEDPin, LOW); //turn the button led off
          HeatRampSetupState = 1; //make the state of the heat ramp to 1, marking it as chosen, and exit the while() loop
        }
        delay(100); //delay used here rather than millis() as no other loops required to run while selecting the heat ramp temp.
      }
    }
  }

  /* Heat Ramp Execution*/
  if (CurrentTime - PreviousHeatRampTime > HeatRampInterval)
  {

  }



}
