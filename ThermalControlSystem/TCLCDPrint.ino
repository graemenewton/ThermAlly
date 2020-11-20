/* CUSTOM FUNCTION FOR PRINTING THERMOCUPLE PARAMETERS TO THE LCD SCREEN */

void TCLCDPrint()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T1="); lcd.print(mcp1.readThermocouple()); lcd.print(" R1="); lcd.print(mcp1.readAmbient()); //read theromcouple and print straight to LCD
  lcd.setCursor(0, 1);
  lcd.print("T2="); lcd.print(mcp2.readThermocouple()); lcd.print(" R2="); lcd.print(mcp2.readAmbient());
}
