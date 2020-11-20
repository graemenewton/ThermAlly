/* CUSTOM FUNCTION FOR PRINT THERMOCUPLE PARAMETERS TO SERIAL INTERFACE */

/* Print temperature values to Serial Bus and LCD. Hot Junction is the probe temp, Cold Junction is the reference built into the Adafruit MCP9600 I2C board  */

void TCSerialPrint()
{

  Serial.print("Hot Junction 1: "); Serial.println(mcp1.readThermocouple()); Serial.print(" ºC");
  Serial.print("Cold Junction 1: "); Serial.println(mcp1.readAmbient()); Serial.print(" ºC");
  Serial.print("Hot Junction 2: "); Serial.println(mcp2.readThermocouple()); Serial.print(" ºC");
  Serial.print("Cold Junction 2: "); Serial.println(mcp2.readAmbient()); Serial.print(" ºC");
  Serial.print("ADC: "); Serial.print(mcp1.readADC() * 2); Serial.println(" uV");
  Serial.print("ADC: "); Serial.print(mcp2.readADC() * 2); Serial.println(" uV");
}
