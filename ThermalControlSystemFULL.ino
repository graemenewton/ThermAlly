 //Useful notes: Arduino due uses 3.3V logic, digital pins output 130mA.

#include <Wire.h> //This library allows you to communicate with I2C / TWI devices
#include <Adafruit_I2CDevice.h> //
#include <Adafruit_I2CRegister.h>
#include <Adafruit_MCP9600.h> //delete this if works, changes "" at ends <>.
#include <LiquidCrystal.h> //allows for easy interfacing with LCD display using digital pins. Requires no I2C adapter.

const int ReadyToOperateLEDPin = 53; //const int sets constant integers, such as pin numbers.
const int BusyLEDPin = 51;
const int TempLEDRedPin = 2;
const int TempLEDGreenPin = 3;
const int TempLEDBluePin = 4;

int TempLEDRedPWM; //int set integers (whole numbers) which can be masked for PWM outs and ADC reads.
int TempLEDGreenPWM;
int TempLEDBluePWM;


float T1Temp; //T1Temp is heat exchanger, store as float T1Temp //note that floats must be used in equations as dividing integers will cause errors as they cannot have decimal points.
float T2Temp; //T2Temp is bath, store as float T2Temp // e.g. 7/2 is 3.5 but this will not work with integer notation


/* Syntax for LCD pin numbers:
LiquidCrystal(rs, enable, d4, d5, d6, d7)
LiquidCrystal(rs, rw, enable, d4, d5, d6, d7)
LiquidCrystal(rs, enable, d0, d1, d2, d3, d4, d5, d6, d7)
LiquidCrystal(rs, rw, enable, d0, d1, d2, d3, d4, d5, d6, d7) */

LiquidCrystal lcd(31, 29, 33, 27, 25, 23, 22); //set the LCD pins (as described in above pin map)

Adafruit_MCP9600 mcp1; //define mcp1 as an adafruit mcp9600 module
Adafruit_MCP9600 mcp2; //define mcp2 as an adafruit mcp9600 module
#define TC1_Address (0x67) //thermcouple 1 is the normal adafruit mcp9600 board I2C address (0x67)
#define TC2_Address (0X66) // thermcouple 2 is has solder bridge across first addr pannels giving 0x66 I2C address

void setup()
{
/* Set the pins. */

  pinMode(ReadyToOperateLEDPin, OUTPUT);
  pinMode(BusyLEDPin, OUTPUT);
  pinMode(TempLEDRedPin, OUTPUT);
  pinMode(TempLEDGreenPin, OUTPUT);
  pinMode(TempLEDBluePin, OUTPUT);

  digitalWrite(ReadyToOperateLEDPin, LOW);
  digitalWrite(BusyLEDPin, HIGH);

/* Print startup message to LCD. */

  lcd.begin(20, 4); //begin LCD communication, device has 20 coloumbs and 4 rows
  lcd.setCursor(0,1);
  lcd.print("  Thermal Control");
  lcd.setCursor(0,2);
  lcd.print("System Initialising");
  delay(3000);

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
  delay(3000); //wait for 2 secs before starting new command to allow message to be on screen for long enough for user
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
    lcd.setCursor(0,2);
    lcd.print("Check connections");
    lcd.setCursor(0,3);
    lcd.print("then reset Arduino.");
    while (1); //infinite loop. Pauses program here.
  }

  Serial.println("Found Thermocouple 1!"); //if mcp9600 for TC1 is detected, the program will not pause and this message will be printed to the serial monitor
  lcd.clear(); //will also print to LCD
  lcd.setCursor(0, 1);
  lcd.print("       Found       ");
  lcd.setCursor(0, 2);
  lcd.print("  Thermocouple 1!");
  delay(3000); //wait 2 secs to allow user to read
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
    lcd.setCursor(0,2);
    lcd.print("Check connections");
    lcd.setCursor(0,3);
    lcd.print("then reset Arduino.");
    while (1); //infinite loop. Pauses program here.
  }

  Serial.println("Found Thermocouple 2!"); //if MCP9600 for TC2 is detected, the program will not pause and this message will be printed to the serial monitor
  lcd.clear(); //will also print to LCD
  lcd.setCursor(0, 1);
  lcd.print("       Found       ");
  lcd.setCursor(0, 2);
  lcd.print("  Thermocouple 2!");
  delay(3000); //wait 2 secs to allow user to read
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
  delay(12000); //wait to allow user to read
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
  digitalWrite(BusyLEDPin, LOW);
  digitalWrite(ReadyToOperateLEDPin, HIGH);

  delay(1000); //slight delay before starting loop, the message will be displayed until something new is overwritten
}


void loop()
{

/* Print temperature values to Serial Bus and LCD. Hot Junction is the probe temp, Cold Junction is the reference built into the Adafruit MCP9600 I2C board  */

  Serial.print("Hot Junction 1: "); Serial.println(mcp1.readThermocouple()); Serial.print(" ºC");
  Serial.print("Cold Junction 1: "); Serial.println(mcp1.readAmbient()); Serial.print(" ºC");
  Serial.print("Hot Junction 2: "); Serial.println(mcp2.readThermocouple()); Serial.print(" ºC");
  Serial.print("Cold Junction 2: "); Serial.println(mcp2.readAmbient()); Serial.print(" ºC");
  Serial.print("ADC: "); Serial.print(mcp1.readADC() * 2); Serial.println(" uV");
  Serial.print("ADC: "); Serial.print(mcp2.readADC() * 2); Serial.println(" uV");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T1="); lcd.print(mcp1.readThermocouple()); lcd.print(" R1="); lcd.print(mcp1.readAmbient()); //read theromcouple and print straight to LCD
  lcd.setCursor(0, 1);
  lcd.print("T2="); lcd.print(mcp2.readThermocouple()); lcd.print(" R2="); lcd.print(mcp2.readAmbient());

/* Store temperatures for T1 and T2 as variables */

  T1Temp = mcp1.readThermocouple(); //T1Temp is heat exchanger, store as float T1Temp
  T2Temp = mcp2.readThermocouple(); //T2Temp is bath, store as float T2Temp

/* Code below regultes the RGB PWM numbers for the RGB LED - this is used so operators can guage the temperature based on LED colour from a distance without having to get close to the screen. */

  if (T2Temp < 2.5){ //if temp is near freezing then blue
    TempLEDRedPWM = 0;
    TempLEDGreenPWM = 0;
    TempLEDBluePWM = 255;
  }
  else if ((T2Temp >= 2.5) && (T2Temp <= 5.0)){ //if temp is 2.5-5 then blue/azure
    TempLEDRedPWM = 0;
    TempLEDGreenPWM = 100;
    TempLEDBluePWM = 255;
  }
  
  else if ((T2Temp > 5.0) && (T2Temp <= 7.5)){ //if temp is 5-7.5 then azure
    TempLEDRedPWM = 0;
    TempLEDGreenPWM = 150;
    TempLEDBluePWM = 255;
  }

  else if ((T2Temp > 7.5) && (T2Temp <= 10.0)){ //if temp is 7.5-10 then azure/cyan
    TempLEDRedPWM = 0;
    TempLEDGreenPWM = 200;
    TempLEDBluePWM = 255;
  }
  
  else if ((T2Temp > 10.0) && (T2Temp <= 15)){ //if temp is 10-15 then cyan
    TempLEDRedPWM = 0;
    TempLEDGreenPWM = 255;
    TempLEDBluePWM = 255;
  }
  
  else if ((T2Temp > 15) && (T2Temp <= 20)){ //if temp is 15-20 then cyan/aquamarine
    TempLEDRedPWM = 0;
    TempLEDGreenPWM = 255;
    TempLEDBluePWM = 200;
  }

  else if ((T2Temp > 20) && (T2Temp <= 25)){ //if temp is 20-25 then aquamarine
    TempLEDRedPWM = 0;
    TempLEDGreenPWM = 255;
    TempLEDBluePWM = 150;
  }
  
  else if ((T2Temp > 25) && (T2Temp <= 30)){ //if temp is 25-30 then aquamarine/green
    TempLEDRedPWM = 0;
    TempLEDGreenPWM = 255;
    TempLEDBluePWM = 100;
  }

//THIS IS THE BASELINE COLOUR 
  else if ((T2Temp > 30) && (T2Temp <= 32.5)){ //if temp is 30-32.5 then green BASELINE TEMP
    TempLEDRedPWM = 0;
    TempLEDGreenPWM = 255;
    TempLEDBluePWM = 0;
  }

  else if ((T2Temp > 32.5) && (T2Temp <= 34.5)){ //if temp is 32.5-34.5 then green/chartreuse
    TempLEDRedPWM = 100;
    TempLEDGreenPWM = 255;
    TempLEDBluePWM = 0;
  }
  
  else if ((T2Temp > 34.5) && (T2Temp <= 36.0)){ //if temp is 34.5-36 then chartreuse
    TempLEDRedPWM = 150;
    TempLEDGreenPWM = 255;
    TempLEDBluePWM = 0;
  }

  else if ((T2Temp > 36.0) && (T2Temp <= 37.5)){ //if temp is 36-37.5 then chartreuse/yellow
    TempLEDRedPWM = 200;
    TempLEDGreenPWM = 255;
    TempLEDBluePWM = 0;
  }

  else if ((T2Temp > 37.5) && (T2Temp <= 39.0)){ //if temp is 37.5-39 then yellow
    TempLEDRedPWM = 255;
    TempLEDGreenPWM = 255;
    TempLEDBluePWM = 0;
  }

  else if ((T2Temp > 39.0) && (T2Temp <= 40.5)){ //if temp is 39-40.5 then yellow/orange
    TempLEDRedPWM = 255;
    TempLEDGreenPWM = 200;
    TempLEDBluePWM = 0;
  }

  else if ((T2Temp > 40.5) && (T2Temp <= 42.0)){ //if temp is 40.5-42 then orange
    TempLEDRedPWM = 255;
    TempLEDGreenPWM = 150;
    TempLEDBluePWM = 0;
  }

  else if ((T2Temp > 42.0) && (T2Temp <= 43.5)){ //if temp is 42-43.5 then orange/red
    TempLEDRedPWM = 255;
    TempLEDGreenPWM = 100;
    TempLEDBluePWM = 0;
  }

  else if (T2Temp > 43.5) { //if temp is over 43.5 then red
    TempLEDRedPWM = 255;
    TempLEDGreenPWM = 0;
    TempLEDBluePWM = 0;
  }
  
//use PWM numbers set above to control colour of RGB LED.
  analogWrite(TempLEDRedPin, TempLEDRedPWM);
  analogWrite(TempLEDGreenPin, TempLEDGreenPWM);
  analogWrite(TempLEDBluePin, TempLEDBluePWM);

  delay(100); //repeat this every 100ms
  
}
