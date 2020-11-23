/* CUSTOM FUNCTION FOR PRINT THERMOCUPLE PARAMETERS TO SERIAL INTERFACE */

/* Print temperature values to Serial Bus and LCD. Hot Junction is the probe temp, Cold Junction is the reference built into the Adafruit MCP9600 I2C board  */
// Serial.print(value, 4) prints 4 decimal places if value is float for example
// The raw ADC resolution in 18-bit mode is 0.5 uV. So multiplying by 2 gives you a reading in uV. Hence why *2 is seen below when print voltage.


void TCSerialPrint()
{

  Serial.print("Hot Junction 1: "); Serial.println(mcp1.readThermocouple(), 4); Serial.print(" ºC");
  Serial.print("Cold Junction 1: "); Serial.println(mcp1.readAmbient(), 4); Serial.print(" ºC");
  Serial.print("Hot Junction 2: "); Serial.println(mcp2.readThermocouple(), 4); Serial.print(" ºC");
  Serial.print("Cold Junction 2: "); Serial.println(mcp2.readAmbient(), 4); Serial.print(" ºC");
  Serial.print("ADC: "); Serial.print(mcp1.readADC() * 2, 4); Serial.println(" uV");
  Serial.print("ADC: "); Serial.print(mcp2.readADC() * 2, 4); Serial.println(" uV");
}
