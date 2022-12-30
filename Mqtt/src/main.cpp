#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>

#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <AsyncElegantOTA.h>
#include <ESPAsync_WiFiManager.h>
#include <ESPAsyncDNSServer.h>

#if !(defined(ESP8266))
#error This code is intended to run on ESP8266 platform! Please check your Tools->Board setting.
#endif

#define wifi_ssid ""
#define wifi_password ""

#define mqtt_server "" // ip na které běží home assistant
#define mqtt_user ""           // mqtt-user
#define mqtt_password ""       // heslo k mqtt-user 

#define humidity_topic "sensor/humidity"
#define temperature_topic "sensor/temperature"
#define pressure_topic "sensor/pressure"
#define altitude_topic "sensor/altitude"

#define PIN_MQ135 A0
#define SEALEVELPRESSURE_HPA (1013.25)

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;
MQ135 mq135_sensor = MQ135(PIN_MQ135);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void vypisHodnotLCD()
{
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float hum = bme.readHumidity();
  float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(temp, hum);
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(temp, hum);
  String text[9] = {"MQ135 RZero:  ", "Corrected RZero: ", "Resistance: ", "PPM: ", "Corrected PPM: ", "Temperature: ", "Pressure: ", "Altitude: ", "Humidity: "};
  int i;
  for (i = 0; i < 9; i++)
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
      lcd.print(temp);
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
    delay(3000);
  }
  return;
}

void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

bool checkBound(float newValue, float prevValue, float maxDiff)
{
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

void setup()
{

  Serial.begin(9600); //musím dát jiný speed pokud chci integrovat wifimanager 115200

  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // Set SDA and SCL ports
  Wire.begin(4, 5);
  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  lcd.init();
  lcd.backlight();
}

long lastForceMsg = 0;
bool forceMsg = false;
long lastMsg = 0;
float tempr = 0.0;
float humi = 0.0;
float pres = 0.0;
float alt = 0.0;
float diff = 1.0;

void loop()
{
  vypisHodnotLCD();
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 1000)
  {
    lastMsg = now;

if (now - lastForceMsg > 300000) {
      lastForceMsg = now;
      forceMsg = true;
      Serial.println("Forcing publish every 5 minutes...");
    }

    float newTemp = bme.readTemperature();
    float newHum = bme.readHumidity();
    float newPres = bme.readPressure() / 100.0F;
    float newAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);

    if (checkBound(newTemp, tempr, diff))
    {
      tempr = newTemp;
      Serial.print("New temperature:");
      Serial.println(String(tempr).c_str());
      client.publish(temperature_topic, String(tempr).c_str(), true);
    }

    if (checkBound(newHum, humi, diff))
    {
      humi = newHum;
      Serial.print("New humidity:");
      Serial.println(String(humi).c_str());
      client.publish(humidity_topic, String(humi).c_str(), true);
    }

    if (checkBound(newPres, pres, diff))
    {
      pres = newPres;
      Serial.print("New pressure:");
      Serial.println(String(pres).c_str());
      client.publish(pressure_topic, String(pres).c_str(), true);
    }

    if (checkBound(newAlt, alt, diff))
    {
      alt = newAlt;
      Serial.print("New Altitude:");
      Serial.println(String(alt).c_str());
      client.publish(altitude_topic, String(alt).c_str(), true);
    }
    forceMsg = false;
  }
}