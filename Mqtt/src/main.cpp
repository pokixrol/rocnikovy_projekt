#include <Arduino.h>

// Khihovny pracující s hardwarem.
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>

// Knihovny pracující s bezdrátovým připojením.
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <AsyncElegantOTA.h>
#include <ESPAsync_WiFiManager.h>
#include <ESPAsyncDNSServer.h>

// Připojení hlavičkových souborů.
#include <configuration.hpp>
#include <variables.hpp>

#if !(defined(ESP8266))
#error This code is intended to run on ESP8266 platform! Please check your Tools->Board setting.
#endif

// Definování témat (topiců) pro MQTT.
#define humidity_topic "sensor/humidity"
#define temperature_topic "sensor/temperature"
#define pressure_topic "sensor/pressure"
#define altitude_topic "sensor/altitude"

// Definování symbolických proměnných pro pin senzoru MQ-135 a hodnoty atmosférického tlaku u hladiny moře.
#define PIN_MQ135 A0
#define SEALEVELPRESSURE_HPA (1013.25)

// Vytvoření objektů na základě použitých knihoven aby jsme mohli přistupovat k jednotlivým funkcím.
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;
MQ135 mq135_sensor = MQ135(PIN_MQ135);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Funkce pro výpis měřených hodnot na LCD displej.
void vypisHodnotLCD()
{
  // Získání hodnot do proměnných.
  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
  correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity);
  resistance = mq135_sensor.getResistance();
  correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);

  // Pole textů vypisujících se na displej.
  String text[7] = {"Temperature: ", "Pressure: ", "Altitude: ", "Humidity: ","RZero: ", "Resistance: ", "PPM: "};

  int i;
  for (i = 0; i < 7; i++)
  {
    // Vyčištění displeje a nastavení kurzoru.
    lcd.clear();
    lcd.setCursor(0, 0);
    // Vypsání textu podle indexu v poli.
    lcd.print(text[i]);
    // Nastavení kurzoru.
    lcd.setCursor(0, 1);

    // Switch pro výpis správné proměnné a jednotky.
    switch (i)
    {
    case 0:
      lcd.print(temperature);
      lcd.print(" *C");
      break;
    case 1:
      lcd.print(pressure);
      lcd.print(" hPa");
      break;
    case 2:
      lcd.print(altitude);
      lcd.print(" m");
      break;
    case 3:
      lcd.print(humidity);
      lcd.print(" %");
      break;
    case 4:
      lcd.print(correctedRZero);
      break;
    case 5:
      lcd.print(resistance);
      break;
    case 6:
      lcd.print(correctedPPM);
      break;
    }
    // Spoždění 3 sekundy na zobrazení každé měřené hodnoty.
    delay(3000);
  }
  return;
}

// Funkce pro nastavení komunikace přes WiFi.
void setup_wifi()
{
  // Výpis ssid sítě, ke které se připojujeme.
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  // Začátek komunikace.
  WiFi.begin(wifi_ssid, wifi_password);

  // Pomyslné načítání dokud nejsme připojeni.
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  // Výpis IP adresy nově připojeného zařízení.
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Funkce pro opětovné připojení.
void reconnect()
{
  // Pokud klient není připojen.
  while (!client.connected())
  {
    // Výpis hlášky pokoušjící se o připojení.
    Serial.print("Attempting MQTT connection...");
    // Znovu připojení.
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password))
    {
      // Výpis hlášky připojeno.
      Serial.println("connected");
    }
    else
    {
      // Výpis statusu pokud nastane chyba.
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Funkce kontrolující rozdíl mezi nově naměřenou hodnotou a jejím předchůdcem.
bool checkDiff(float newValue, float prevValue, float maxDiff)
{
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

// Funkce pro vynucené publikování (publish).
void forcedPublish(int publishAfterMs){
 long now = millis();
 // Když je momentální čas - čas poslední zprávy větší než 1000.
  if (now - lastMsg > 1000)
  {
    // Čas poslední zprávy = momentální čas.
    lastMsg = now;
  // Když je momentální čas - čas poslední vynucené zprávy větší než námi nastavený čas pro vynucené publikování.
  if (now - lastForceMsg > publishAfterMs) {
      lastForceMsg = now;
      forceMsg = true;
      Serial.println("Forcing publish every 5 minutes...");
    }
  }
}

// Funkce pro publikování zpráv k příslušným tématům.
void publish(){

    // Vyvolání funkce pro vynucené publikování.
    forcedPublish(300000);

    // Získání nových hodnot do proměnných.
    newTemp = bme.readTemperature();
    newHum = bme.readHumidity();
    newPres = bme.readPressure() / 100.0F;
    newAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);

    // Kontrola rozdílu nové a minulé hodnoty nebo vynucené publikování.
    if (checkDiff(newTemp, temp, diff) || forceMsg)
    {
      // Minulá hodnota = nová hodnota.
      temp = newTemp;
      // Výpis a publikování.
      Serial.print("New temperature:");
      Serial.println(String(temp).c_str());
      client.publish(temperature_topic, String(temp).c_str(), true);
    }

    if (checkDiff(newHum, hum, diff) || forceMsg)
    {
      hum = newHum;
      Serial.print("New humidity:");
      Serial.println(String(hum).c_str());
      client.publish(humidity_topic, String(hum).c_str(), true);
    }

    if (checkDiff(newPres, pres, diff) || forceMsg)
    {
      pres = newPres;
      Serial.print("New pressure:");
      Serial.println(String(pres).c_str());
      client.publish(pressure_topic, String(pres).c_str(), true);
    }

    if (checkDiff(newAlt, alt, diff) || forceMsg)
    {
      alt = newAlt;
      Serial.print("New Altitude:");
      Serial.println(String(alt).c_str());
      client.publish(altitude_topic, String(alt).c_str(), true);
    }
    forceMsg = false;
  }


void setup()
{

  // Nastavení sériové linky.
  Serial.begin(9600); //musím dát jiný speed pokud chci integrovat wifimanager 115200

  // Vyvolání funkce pro připojení k WiFi.
  setup_wifi();
  // Nastavení MQTT serveru.
  client.setServer(mqtt_server, 1883);

  // Počátek I2C.
  Wire.begin(4, 5);

  // Když senzor BME280 není na správné adrese.
  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  // Inicializace LCD.
  lcd.init();
  lcd.backlight();
}

void loop()
{
  // Vyvolání funkce pro výpis hodnot na displej.
  vypisHodnotLCD();
  // Když není klient připojen.
  if (!client.connected())
  {
    // Vyvolání funkce pro opětovné připojení.
    reconnect();
  }
  client.loop();
  publish();
 
}