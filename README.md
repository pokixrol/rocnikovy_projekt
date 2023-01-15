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
  - VirtualBox
  - Home Assistant
  - ESP Home
  - Mosquitto MQTT Broker
  - InfluxDB
  - Grafana
  - KiCad
  
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
Schéma zapojení

![image](https://user-images.githubusercontent.com/66769522/211378547-4e97b572-9e53-49f9-b8c7-764c39fa181b.png)

Pohled shora zapojení bez senzoru MQ-135

![image](https://user-images.githubusercontent.com/66769522/211378494-af1f47b9-fc82-4f1d-9f26-5a0fa08dbb74.png)

Přední detail na sestavu 
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

Po naběhnutí se nám zobrazí systémové údaje. Přejdeme do prohlížeče a připojíme se ke správné IP adrese a portu např. `http://192.168.0.1:8123`. Po tomto kroku se dostaneme do uživatelského rozhraní Home Assistanta.

### Home Assistant uživatelské rozhraní
Vítejte v Home Assistant!

První věcí po tom co se dostaneme do uživatelského rozhraní je, že si vytvořímě uživatelský účet.

![image](https://user-images.githubusercontent.com/66769522/211581740-7d51b00e-dd7d-4ddd-84e3-283222c558d5.png)

Po vytvoření uživatelského účtu zadáme na další stránce základní informace jako polohu nebo jaký systém jednotek používáme.

![image](https://user-images.githubusercontent.com/66769522/211585379-6f038ebc-f660-442d-a6c1-51a8ec35450e.png)

Po zadání základních informací Home Assistant sám detekuje zařízení a služby, které jsou reprezentovány jako integrace. Můžeme je nastavit nyní, ale klidně i později. Já pokračuji dále bez nastavování nalezených integrací.

![image](https://user-images.githubusercontent.com/66769522/211583864-0735b58f-6416-474b-aa5f-0256ea569f30.png)

Po dokončení posledního kroku se dostaneme do přehledu Home Assistantu.

### Instalace add-onů
Bez add-onů neboli doplňků se v Home Assistantu nehneme z místa, proto je pro další postup nutné hned někoik si jich nainstalovat.

Klikneme na Settings (nastavení) -> Add-ons (doplňky) -> Add-on store (obchod s doplňky). 
V obchodu si nainstalujeme následující doplňky: 
  - Studo Code Server
  - Mosquitto Broker
  - InfluxDB
  - Grafana
  - ESPHome (Volitelné, k výslednému produktu není potřeba mít tento add-on. Já jej použila pro inspiraci a testování.)

Všechny add-ony nainstalujeme pomocí jednoduchého kliknutí na tlačítko "Install". U každého add-onu je možnost povolit nebo zakázat určité akce např. autoupdate nebo watchdog. Je čistě na vašem uvážení jaká nastavení si povolíte, já doporučuji minimálně u všech kde je to možné povolit zobrazení v postranním panelu pro lepší přístup. U některých add-onů je potřeba po instalaci jestě pro jejich spuštění zmáčknout tlačítko "Start".

### Práce s add-ony
S některými add-ony se pracuje lehce a jednoduše s jinými už to tak jednoduché není, proto se v této části budu věnovat každému add-onu zvášť a u těch složitějších přesně popíšu postup práce.
#### Studio Code server
Mnoho úprav lze provést přímo v uživatelském rozhraní Home Assistantu, ale ne všechny. Studio Code Server je souborový editor, který budeme používat k úpravám souboru configuration.yaml.
#### ESP Home
Pro jednodušší integrování vašich senzorů můžete využít add-on ESPHome. Pokud byste o to měli zájem zde je pěkný tutoriál v [češtině](https://www.youtube.com/watch?v=xwjwmeov054&t=489s) a zde v [angličtině](https://www.youtube.com/watch?v=iufph4dF3YU&t=28s). Vyskoušela jsem si práci ESPHome a následně použila pro inspiraci jak by mohl můj výsledný projekt vypadat.
#### Mosquitto broker
##### Teorie
Jedná se o Open Source MQTT broker. MQTT protokol funguje na principu publish/subscribe, kde broker funguje jako prostředník pro předávání zpráv mezi klienty. Zprávy jsou tříděny do témat (topic) a zařízení v daném tématu buď publikuje (publish) nebo odebírá (subscribe). 

Pro lepší pochopení se můžete [zde](https://www.youtube.com/watch?v=NXyf7tVsi10&list=LL&index=2) podívat na video s vysvětlením nebo níže na grafické schéma.

![image](https://user-images.githubusercontent.com/66769522/211756970-5e109fcf-9614-4db5-bd43-f60425ec6616.png)

##### Praxe
###### Vytvoření a přiřazení MQTT uživatele
V uživatelském rozhranní Home Assistantu klikneme na nastavení -> lidé -> uživatelé -> přidat uživatele. Poté se nám objeví jednoduchý formulář kde si vytvoříme našeho mqtt uživatele. Po vytvoření našeho uživatele klikneme na nástroje pro vývojáře a dáme zkontrolovat konfiguraci. Pokud kontrola proběhla v pořádku dáme restartovat a chvíli počkáme.

Po restartování přejdeme do nastavení -> zařízení a služby, zde si najdeme náš MQTT broker zmáčkneme tlačítko configure a následně submit. Poté klikneme znovu na tlačítko configure a poté na re-configure MQTT. Opět se nám objeví jednoduchý formulář kde změníme pole uživatelské jméno a heslo na údáje námi předtím vytvořeného uživatele. 

Pokud vám vytvoření uživatele dělá problémy můžete postupovat s pomocí tohoto [tutoriálu](https://www.youtube.com/watch?v=dqTn-Gk4Qeo&t=477s).
###### Vytvoření projektu
Vytvořila jsem v PlatformIO nový projekt s názvem `Mqtt`. Do tohoto projektu jsem zahrnula nejen kód týkající se MQTT ale i výpis na LCD displej, jedná se tak o finální projekt. 
###### Vytvoření hlavičkových souborů .hpp
V projektu Mqtt v podadresáři src najdeme soubor main.cpp, což je náš hlavní soubor kde budeme tvořit náš kód. Kromě tohotou souboru vytvoříme v podadresáři src další dva soubory s příponou .hpp a to configuration.hpp a variables.hpp (pojmenovat si je samozřejmě můžete jakkoli jinak).
###### Připojení hlavičkových souborů .hpp
Nahoru do souboru main. cpp vložíme následující kód pro připojení hlavičkových souborů.
```
#include <configuration.hpp>
#include <variables.hpp>
```
###### Knihovny
Nahoru do souboru main.cpp připojíme následující knihovny.
```
// Khihovny pracující s hardwarem.
#include <Wire.h>
#include <LiquidCrystal_I2C.h>		// Tuto knihovnu není potřeba ve zjednodušeném příkladu níže použít, ve finálním projektu však ano.
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>			// Tuto knihovnu není potřeba ve zjednodušeném příkladu níže použít, ve finálním projektu však ano.

// Knihovny pracující připojením a mqtt.
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
```
Nesmíme zapomenout upravit i soubor platformio.ini, jinak knihovny nebudou fungovat.
###### Úpravy souboru configuration.hpp
Do souboru configuration.hpp vložíme následující kód a upravíme podle vlastních potřeb.
```
#define wifi_ssid "Your-wifi-ssid"            // Ssid (název) vaší Wi-Fi sítě.
#define wifi_password "Your-wifi-password"    // Heslo k vaší Wi-Fi síti.

#define mqtt_server "Mqtt-ip"                 // IPv4 adresa na které běží home assistant.
#define mqtt_user "Mqtt-user-name"            // Jméno vašeho MQTT uživatele
#define mqtt_password "Mqtt-user-password"    // Heslo vašeho MQTT uživatele
```
###### Úpravy souboru variables.hpp
V souboru variables.hpp vytvoříme většinu proměných, které budeme v souboru main.cpp používat. Já zde uvedu pouze ty, které budu potřebovat v příkladu níže. Veškeré mnou použité proměnné naleznete v repozitáři výše.
```
long lastPubeMsg = 0;
bool Msg = false;
long lastMsg = 0;
float temp = 0;
float newTemp = 0;
```
###### Úpravy souboru main.cpp
Po připojení knihoven a hlavičkových souborů je na řadě samotné programování. Celý okomentovaný kód naleznete výše v repozitáři. Zde uvedu pouze krátky příklad práce s MQTT, který si můžete vyzkoušet.

 Definování tématu (topicu) pro MQTT.
```
#define temperature_topic "sensor/temperature"
```
Vytvoření objektů na základě použitých knihoven aby jsme mohli přistupovat k jednotlivým funkcím.
```
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;
```
Funkce pro nastavení komunikace přes WiFi.
```
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
```
Funkce pro opětovné připojení.
```
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
```
Funkce pro publikování po určitém čase.
```
void publishAfter(int publishAfterMs){
 long now = millis();
 // Když je momentální čas - čas poslední zprávy větší než 1000.
  if (now - lastMsg > 1000)
  {
    // Čas poslední zprávy = momentální čas.
    lastMsg = now;
  // Když je momentální čas - čas poslední publikované zprávy větší než námi nastavený čas pro publikování.
  if (now - lastPubMsg > publishAfterMs) {
      lastPubMsg = now;
      Msg = true;
      Serial.println("Publish ...");
    }
  }
}
```
Funkce pro publikování zprávy k příslušnému tématu.
```
void publish(){

    // Vyvolání funkce pro publikování s nastavením času na 1 minutu (60000 ms = 1 min).
     publishAfter(60000);

    // Získání nové hodnoty do proměnné.
    newTemp = bme.readTemperature();
   
    // Když je Msg true.
    if (Msg)
    {
      // Minulá hodnota = nová hodnota.
      temp = newTemp;
      // Výpis a publikování.
      Serial.print("New temperature:");
      Serial.println(String(temp).c_str());
      client.publish(temperature_topic, String(temp).c_str(), true);
    }
    Msg = false;
  }
```
```
void setup()
{

  // Nastavení sériové linky.
  Serial.begin(9600);

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
}
```
```
void loop()
{
 
  // Když není klient připojen.
  if (!client.connected())
  {
    // Vyvolání funkce pro opětovné připojení.
    reconnect();
  }
  client.loop();
  publish();
 
}
```
###### Úpravy configuration.yaml
Programová část v PlatformIO ovšem není vše. Dále budeme potřebovat provést už jen jednoduché úpravy konfiguračního souboru configuration.yaml. Vrátíme se zpět do uživatelského rozhraní Home Assistantu a otevřeme si náš souborový editor Studio Code Server. V souborovém editoru zvolíme konfigurační soubor configuration.yaml a vložíme následující kód.

Tento kód stačí pro náš vzorový příklad.
```
mqtt:
  sensor:
    - name: "Temperature"
      state_topic: "sensor/temperature"
      qos: 0
      unit_of_measurement: "ºC"
```
Zde je zbytek kódu, který jsem použila ve svém projektu.
```
    - name: "Humidity"
      state_topic: "sensor/humidity"
      qos: 0
      unit_of_measurement: "%"

    - name: "Pressure"
      state_topic: "sensor/pressure"
      qos: 0
      unit_of_measurement: "hPa"

    - name: "Approximate altitude"
      state_topic: "sensor/altitude"
      qos: 0
      unit_of_measurement: "m"
```

Po úpravách konfiguračního souboru je nutné Home Assistanta restartovat. Klikneme na nástroje pro vývojáře a dáme zkontrolovat konfiguraci. Pokud kontrola proběhla v pořádku dáme restartovat a chvíli počkáme.
###### Kompilace, upload kódu
V dalším kroku se vrátíme zpět do PlatformIO, náš kód zkompilujeme, opravíme případné chyby a nahrajeme. Pokud vše proběhlo v pořádku, měli bychom v serial monitoru vidět, zda se zařízení úspěšně připojilo a měří hodnoty.
###### Home Assistant overview dashboard
Pokud jste došli až sem tak vám gratuluji, právě nastal čas na to sklidit plody vaší práce. Vytvořímě totiž jednoduchý dashboard v overview Home Assistantu.

V uživatelském rozhranní klikneme na Overview (přehled) -> Edit (upravit ovládací panel) -> Add caard (přidat kartu). Objeví se nám list předem vytvořených karet, ze kterých si můžeme vybrat dle vlastního uvážení. Já jsem pro svůj projekt zvolila kartu "Entities", ale pokud jste postupovali podle mého vzorového příkladu měla by stačit karta "Entity".

V kartě si můžeme nastavit spoustu věcí např. název, ikonu, jednotku, motiv, toto všechno je čistě na vás jak si kartu upravíte. Nejdůležitější však je výběr správné entity. Název námi požadované entity se schoduje s názvem, který jsme jí dali v konfiguračním souboru configuration.yaml, v našem případě "Temperature". Po výběru správné netity (nebo entit) kartu uložíme. Klikneme na "hotovo" a máme hotovo.
#### InfluxDB, Grafana
Pokud se nespokojíte s "obyčejným" dashboardem v přehledu, můžeme využít dalších dvou add-onů, které jsme si na začátku nainstalovali a to InfluxDB a Grafanu. Tyto dva add-ony spolu úzce souvisí, proto je zmiňuji v jednom bloku.
##### InfluxDB
Začneme s vytvořením nové databáze. Přejdeme do uživatelského rozhraní InfluxDB a klikneme na InfluxDB Admin -> Databases -> Create database. Databázi pojmenujeme a potvrdíme. Dále si vytvoříme uživatele pro tuto databázi kliknutím na tlačítko InfluxDB Admin -> Users -> Create user. Uživatele pojmenujeme, nastavíme mu heslo a potvrdíme. Následně našemu uživateli změníme práva v kolonce permissions z none na all.
##### Úpravy configuration.yaml
Po vytvoření databáze a uživatele je ještě nutné pozměnit konfigurační soubor configuratin.yaml.
```
influxdb:
  host: 192.168.0.1
  port: 8086
  database: DB_for_Grafana
  username: grafana-user
  password: grafana-user-password
  max_retries: 3
  default_measurement: state
```
Po úpravách konfiguračního souboru je nutné Home Assistanta restartovat. Klikneme na nástroje pro vývojáře a dáme zkontrolovat konfiguraci. Pokud kontrola proběhla v pořádku dáme restartovat a chvíli počkáme.
##### Grafana
Po vytvoření databáze přejdeme do uživatelského rozhraní Grafany. Klikneme na Configuration -> Data sources -> Add data source a vybereme InfluxDB. Objemví se nám poměrně rozsáhlé nastavení my doplníme následující: do kolonky URL doplníme `http://ip_na_ktere_bezi_home_assistant:8086`, do kolonky Database doplníme název naší vytvořené databáze, do kolonky User doplníme jméno uživatele pro tuto databázi, do kolonky Password doplníme heslo tohoto uživatele, v kolonce HTTP Method zvolíme možnost GET. Klikneme na tlačítko save and test a pokud je vše v pořádku máme hotovo. Nyní velmi jednoduše můžeme tvořit komplexní grafy.

[Zde](https://www.youtube.com/watch?v=KM6UC4tMVYo&list=LL&index=3&t=309s) naleznete video s tutoriálem jak na Grafanu a InfluxDB i s ukázkou tvoření grafů.
## Doporučení
  - Skvělé YouTube kanály o domácí automatizaci
    - https://www.youtube.com/@EverythingSmartHome/videos
    - https://www.youtube.com/@SmartHomeAddict/videos
    - https://www.youtube.com/@ITandAutomationAcademy/videos
    - https://www.youtube.com/@HelloFutureCZ/videos
    - https://www.youtube.com/@HomeAutomationGuy/videos

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
  - Uživatelsky ovládaný výpis na displeji
    - Menu, výpis hodnot na vyžádání (ovládání tlačítky)
  - Měření rychlosi a směru větru
  - Dodělat/vylepšit měření kvality ovzduší
    - Identifikovat jednotlivé plyny
    - Zobrazení v Home Assistantu
  - Napájení z externího zdroje
  - Krabička
