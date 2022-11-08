#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//Display

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{

  lcd.init();
  lcd.backlight();

}

void loop()
{

 lcd.clear();
 lcd.setCursor (0,0);
 lcd.print("Hello World!");
 delay(500);

}