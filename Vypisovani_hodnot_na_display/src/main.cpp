#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>

#define PIN_MQ135 A0
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
MQ135 mq135_sensor = MQ135(PIN_MQ135);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void vypisHodnot()
{
  float tmp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float hum = bme.readHumidity();
  float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(tmp, hum);
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(tmp, hum);
  String text[9] = {"MQ135 RZero:  ", "Corrected RZero: ", "Resistance: ", "PPM: ", "Corrected PPM: ", "Temperature: ", "Pressure: ", "Altitude: ", "Humidity: "};
  int i;
  for (i = 0; i < 9; i++)
  {
    do
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(text[i]);
      lcd.setCursor(0, 1);

      switch (i)
      {
      case 0:
        lcd.print(rzero);
        break;
      case 1:
        lcd.print(correctedRZero);
        break;
      case 2:
        lcd.print(resistance);
        break;
      case 3:
        lcd.print(ppm);
        break;
      case 4:
        lcd.print(correctedPPM);
        break;
      case 5:
        lcd.print(tmp);
        lcd.print(" *C");
        break;
      case 6:
        lcd.print(pressure);
        lcd.print(" hPa");
        break;
      case 7:
        lcd.print(altitude);
        lcd.print(" m");
        break;
      case 8:
        lcd.print(hum);
        lcd.print(" %");
        break;
      }
      delay(2000);
    } while (millis() <= 2000);
  }
  return;
}

void setup()
{
  Serial.begin(9600);

  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  lcd.init();
  lcd.backlight();
}

void loop()
{
  vypisHodnot();
}