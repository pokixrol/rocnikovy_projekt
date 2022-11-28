#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>

#define PIN_MQ135 A0

MQ135 mq135_sensor = MQ135(PIN_MQ135);

float temperature = 21.0;
float humidity = 25.0;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{

  lcd.init();
  lcd.backlight();
}

void loop()
{
  float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity);
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MQ135 RZero: ");
  lcd.setCursor(0, 1);
  lcd.print(rzero);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Corrected RZero: ");
  lcd.setCursor(0, 1);
  lcd.print(correctedRZero);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Resistance: ");
  lcd.setCursor(0, 1);
  lcd.print(resistance);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PPM: ");
  lcd.setCursor(0, 1);
  lcd.print(ppm);
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Corrected PPM: ");
  lcd.setCursor(0, 1);
  lcd.print(correctedPPM);
  delay(2000);
}