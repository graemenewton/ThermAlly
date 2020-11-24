#include <LiquidCrystal.h>



//Syntax
//LiquidCrystal(rs, enable, d4, d5, d6, d7)
//LiquidCrystal(rs, rw, enable, d4, d5, d6, d7)
//LiquidCrystal(rs, enable, d0, d1, d2, d3, d4, d5, d6, d7)
//LiquidCrystal(rs, rw, enable, d0, d1, d2, d3, d4, d5, d6, d7)

LiquidCrystal lcd(31, 29, 33, 27, 25, 23, 22);

void setup() {
lcd.begin(20, 4);
lcd.setCursor(0,1);
lcd.print("Hello world!");
}

void loop() {



delay(100);




}
