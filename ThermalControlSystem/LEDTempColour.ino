/* Code below regultes the RGB PWM numbers for the RGB LED - this is used so operators can guage the temperature based on LED colour from a distance without having to get close to the screen. */


void LEDTempColour()
{

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
}
