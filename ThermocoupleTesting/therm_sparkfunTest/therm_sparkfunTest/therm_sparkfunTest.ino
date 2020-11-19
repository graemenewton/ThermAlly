
#include <SparkFun_MCP9600.h>
MCP9600 tempSensor;

uint8_t setThermocoupleType(Thermocouple_Type TYPE_T); 

void setup(){
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000);
    tempSensor.begin(0x67);       // Uses the default address (0x60) for SparkFun Thermocouple Amplifier
    //tempSensor.begin(0x66); // Default address (0x66) for SparkX Thermocouple Amplifier

    //check if the sensor is connected
    if(tempSensor.isConnected()){
        Serial.println("0x67 sensor connected!");
    }
    else {
        Serial.println("0x67 sensor not connected!");
        while(1); //hang forever
    }
    
 //change the thermocouple type being used
    Serial.println("Setting Thermocouple Type!");
    tempSensor.setThermocoupleType(TYPE_T);

    //make sure the type was set correctly!
    if(tempSensor.getThermocoupleType() == TYPE_T){
        Serial.println("Thermocouple Type set sucessfully!");
    }

    else{
        Serial.println("Setting Thermocouple Type failed!");
    }
}

void loop(){ //print the thermocouple, ambient and delta temperatures every 200ms if available
        Serial.print("Thermocouple: ");
        Serial.print(tempSensor.getThermocoupleTemp());
        Serial.print(" °C   Ambient: ");
        Serial.print(tempSensor.getAmbientTemp());
        Serial.print(" °C   Temperature Delta: ");
        Serial.print(tempSensor.getTempDelta());
        Serial.print(" °C");
        Serial.println();
        delay(20); //don't hammer too hard on the I2C bus
}
