# Ročníkový projekt - Meteostanice

>Maturitní práce, Karolína Říčná, IT4 2022/23, SŠPU Opava

## Popis
Projekt se zabývá tvorbou meteorologické stanice se zobrazováním hodnot měřených veličin v systému pro domácí automatizaci – Home Assistant. 

Projekt se skládá z hardwarové a softwarové části. 

Základ hardwarové části tvoří vývojová deska WeMos D1 Mini. Zařízení pomocí senzoru BME280 měří teplotu, atmosférický tlak, vlhkost vzduchu a absolutní nadmořskou výšku. Program reaguje na změnu měřených veličin, jakmile ji zaznamená změna se projeví v grafech. 

Programová část stanice, která obstarává správu měřícího senzoru a odesílání zpráv je řešena v jazyce Arduino, což je kombinace jazyků C a C++. 

Další konfigurace jsou provedeny v konfiguračním souboru Home Assistantu – configuration.yaml nebo přímo v uživatelském rozhraní Home Assistantu či jeho add-onů.

## Cíle
>Cílem projektu je vytvořit funkční sestavu, pochopit a prostudovat technologii MQTT, propojit zařízení s Home Assistantem a v grafech vykreslit hodnoty měřených veličin měnící se v čase. 

Měřené veličiny:
  - Teplota
  - Vlhkost vzduchu
  - Atmosférický tlak
  - Kvalita vzduchu ?
  
## Technologie
  - Home Assistant
  - ESP Home
  - Mosquitto MQTT Broker
  - InfluxDB
  - Grafana
  
  Knihovny:
  - <Arduino.h>
  - <Wire.h>
  - <LiquidCrystal_I2C.h>
  - <Adafruit_Sensor.h>
  - <Adafruit_BME280.h>
  - <MQ135.h>
  - <PubSubClient.h>
  - <ESP8266WiFi.h>
  - <AsyncElegantOTA.h>
  - <ESPAsync_WiFiManager.h>
  - <ESPAsyncDNSServer.h>
  
## Součástky
  - WeMos D1 Mini ESP8266 Wifi modul
  - Senzor tlaku, teploty a vlkosti BME 280
  - Senzor pro detekci kvality vzduchu MQ-135
  - LCD displej 1602, I2C převodník

## Postup
### Hardwarový model
schéma obrázek
### Testování součástek
Testování jednotlivých součástek probíhalo po malých krocích. Zapojovala jsem jednu součástku po druhé do nepájivého pole a zkoušela jejich funkčnost pomocí jednoduchých příkladů. Celou softwarovou část týkající se hardwarového modelu řeším v IDE Visual Studio Code s rozšířením PlatformIO. Po vyzkoušení všech součástek jsem začala dávat dohromady samotnou sestavu.
#### WeMos D1 Mini
K testování vývojové desky jsem vytvořila jednoduchý příklad blikání LEDky na stisk tlačítka.
##### Knihovny
Byla použita pouze knihovna `#include <Arduino.h>`, díky které můžeme v projektu využívat frameworku Arduino. Tuto knihovnu bude obsahovat každý další příklad, proto ji vynechám ať se neopakuji.
##### Kód příkladu
```
byte led = 14;
byte button = 12;

void setup() {
  pinMode(led, OUTPUT);
  pinMode(button, INPUT);

}

void loop() {
  if(digitalRead(button)){
  digitalWrite(led, HIGH);
  } else{
  digitalWrite(led,LOW);
  }
}
```
#### BME280
Hlavní senzor mého projektu měřící teplotu, vlhkost vzduchu, atmosférický tlak a absolutní nadmořskou výšku. Měření veličin probíhá pomocí funkcí read z knihovny Adafruit_BME280. Naměřené hodnoty se vypisují na serial monitor.
##### Knihovny
```
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
```
##### Kód příkladu
```
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

void setup() {
	Serial.begin(9600);

	if (!bme.begin(0x76)) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
}

void loop() {
	Serial.print("Temperature = ");
	Serial.print(bme.readTemperature());
	Serial.println("*C");

	Serial.print("Pressure = ");
	Serial.print(bme.readPressure() / 100.0F);
	Serial.println("hPa");

	Serial.print("Approx. Altitude = ");
	Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
	Serial.println("m");

	Serial.print("Humidity = ");
	Serial.print(bme.readHumidity());
	Serial.println("%");

	Serial.println();
	delay(1000);
}
```
#### MQ-135
Senzor měřící kvalitu ovzduší. Měření veličin probíhá pomocí funkcí get z knihovny MQ135. Naměřené hodnoty se vypisují na serial monitor.
##### Knihovny
```
#include <MQ135.h>
```
##### Kód příkladu
```
#define PIN_MQ135 A0

MQ135 mq135_sensor = MQ135(PIN_MQ135);

void setup() {
  Serial.begin(9600);
}

void loop() {
  float rzero = mq135_sensor.getRZero();
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();

  Serial.print("MQ135 RZero: ");
  Serial.print(rzero);
  Serial.print("\t Resistance: ");
  Serial.print(resistance);
  Serial.print("\t PPM: ");
  Serial.print(ppm);

  delay(300);
}
```
#### LCD displej 1602
Zobrazovací médium. Pomocí funkce `setCursor()` nastavíme řádek a sloupec kde náš výpis bude začínat a požadovaný text se vypíše funkcí `print()` z knihovny LiquidCrystal_I2C.
##### Knihovny
```
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
```
##### Kód příkladu
```
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
```
### Instalace Home Assistant
Budu popisovat postup, který jsem použila já, tedy instalace Home Assistant Operating System na Windows 10. Abychom mohli začít používat Home Assistant, nejprve je nutné stáhnout správnou [image](https://www.home-assistant.io/installation/windows) a to tu s příponou `.vdi` pro VirtualBox.  
#### VirtualBox
Pro další postup je nutné mít funkční virtualizační nástroj [VirtualBox](https://www.virtualbox.org/wiki/Downloads). 

Vytvoříme nový virtuální stroj s těmito parametry:
  - Vlastní název
  - Vlastní cesta k umístění virtuálního stroje
  - Typ operačního systému Linux s verzí Linux 2.6 / 3.x / 4.x (64-bit) popř. 32-bit
  - 2 GB RAM
  - 32 GB Storage
  - 2vCPU

Dále zvolíme možnost použití existujícího virtuálního hard disku. Klikneme na přidat a zvolíme námi již předtím staženou image. Po tomto kroku by se nám měl vytvořit nový virtuální stroj.

Ještě před spuštěním je potřeba provést úprava některých nastavení. Klikneme na settings -> stystem -> Motherboard a povolíme EFI. Dále klikneme na network -> adapter 1 -> bridged adapter a zvolíme naši stíťovou kartu.

Po uložení nastavení můžeme spustit virtuální stroj a nechat ho naběhnout. 

Po naběhnutí se nám zobrazí systémové údaje. Přejdeme do prohlížeče připojíme se ke správné IP adrese a portu např. `http://192.168.0.1:8123`. Po tomto kroku se dostaneme do uživatelského rozhraní Home Assistanta.
### Home Assistant uživatelské rozhraní
### Instalace add-onů
### Práce s add-ony
#### ESP Home
#### InfluxDB
#### Grafana
### MQTT
### Úpravy configuration.yaml
...


## Zdroje
#### Home Assistant
  - https://www.youtube.com/watch?v=i72K1wyuTfg&t=433s 
  - https://www.youtube.com/watch?v=cZV2OOXLtEI&t=547s 
  - https://www.home-assistant.io/
    - https://www.home-assistant.io/installation/windows
    - https://www.home-assistant.io/integrations/mqtt/
    - https://www.home-assistant.io/blog/2015/10/11/measure-temperature-with-esp8266-and-report-to-mqtt/

#### YouTube kanál Hello-Future CZ, playlist Home assistant
   - https://www.youtube.com/watch?v=XUv89aVMZNE&list=PLmEu9kI4avpRZ4o1WXfSlVWO4LYC2PyLs 

#### Integrace InfluxDB, Grafana
  - https://www.youtube.com/watch?v=lILY8eSspEo 
  - https://www.youtube.com/watch?v=KM6UC4tMVYo&list=LL&index=1

#### Wifi Manager
  - https://www.youtube.com/watch?v=UlRLTvl4DRc&list=LL&index=4

#### BME280
  - https://lastminuteengineers.com/bme280-arduino-tutorial/
  - https://randomnerdtutorials.com/esp8266-nodemcu-mqtt-publish-bme280-arduino/
  
#### Ostatní
  - https://randomnerdtutorials.com/
  - https://lastminuteengineers.com/
  
## To do list
  - Sehnat hardware 
    - Složit model
     - Otestovat
  - Naprogramovat vypisování na LCD displej
  - Vyzkoušet práci s WiFI Managerem
  - Rozjet Home Assistant
    - Nainstalovat add-ony ESP Home, MQTT Broker, InfluxDB, Grafana
    - Vyzkoušet add-ony
    - Naprogramovat komunikaci mezi ESP, MQTT Brokerem a Home Assistantem
    - Zařídít vykreslování komplexnějších grafů pomocí InfluxDB a Grafany
  - Vhodně upravit readme.md
    - Pravidelně aktualizovat
      - Přidat vhodné popisy
      - Doplnit důležité informace
  - Dokumentace
  - Prezentace
  - Video 
## Vylepšení do budoucna
  - Vyměnit nepájivé pole za plošný spoj
  - Vyměnit spouštění Home Assistantu pomocí VirtualBoxu za Raspberry Pi
  - Připojování k Wifi skrze Wifimanager
  - Měření rychlosi a směru větru
  - Dodělat/vylepšit měření kvality ovzduší
    - Identifikovat jednotlivé plyny
    - Zobrazení v Home Assistantu
  - Napájení z externího zdroje
  - Krabička
