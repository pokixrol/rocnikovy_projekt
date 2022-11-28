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
    float tmp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float hum = bme.readHumidity();
  float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(tmp, hum);
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(tmp, hum);

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
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temperature: ");
  lcd.setCursor(0, 1);
  lcd.print(tmp);
  lcd.print(" *C");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pressure: ");
  lcd.setCursor(0, 1);
  lcd.print(pressure);
  lcd.print(" hPa");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Altitude: ");
  lcd.setCursor(0, 1);
  lcd.print(altitude);
  lcd.print(" m");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Humidity: ");
  lcd.setCursor(0, 1);
  lcd.print(hum);
  lcd.print(" %");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Konec hlaseni :)");
  delay(3000);
}