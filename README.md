# Ročníkový projekt - Meteostanice

>Maturitní práce, Karolína Říčná, IT4 2022/23, SŠPU Opava

## Popis
>Multifunkční meteostanice.

## Cíle
>Měření hodnot a jejich zobrazování v Home Assistantu.

Měřené veličiny:
  - Teplota
  - Vlhkost
  - Tlak
  - Kvalita vzduchu
  
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
>

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
  - Měření rychlosi a směru větru
  - Dodělat/vylepšit měření kvality ovzduší
    - Identifikovat jednotlivé plyny
    - Zobrazení v Home Assistantu
  - Napájení z externího zdroje
  - Krabička
