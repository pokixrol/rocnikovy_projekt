#include <Arduino.h>
#include <AsyncElegantOTA.h>
#include <ESPAsync_WiFiManager.h> //https://github.com/khoih-prog/ESPAsync_WiFiManager
#include <ESPAsyncDNSServer.h>
#if !(defined(ESP8266))
#error This code is intended to run on ESP8266 platform! Please check your Tools->Board setting.
#endif
AsyncWebServer webServer(80);
DNSServer dnsServer;
 
//Customized home page
String myHostName = "My-ESP8266";
String myESP8266page = "<a href='/update'>Update Firmware</span></a>";
String myNotFoundPage = "<h2>Error, page not found! <a href='/'>Go back to main page!</a></h2>";
 
//ESP8266 Access credential
const char *myUsername = "myUserName";
const char *myPass = "myPass";
 
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    while (!Serial)
        ;
    delay(200);
    Serial.print("\nStarting Async_AutoConnect_ESP8266_minimal on " + String(ARDUINO_BOARD));
    Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION);
    ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer, "My-ESP8266");
    //ESPAsync_wifiManager.resetSettings();   //reset saved settings
    //ESPAsync_wifiManager.setAPStaticIPConfig(IPAddress(192,168,186,1), IPAddress(192,168,186,1), IPAddress(255,255,255,0));
    Serial.println("Connect to previously saved AP...");
    ESPAsync_wifiManager.autoConnect("My-ESP8266");
 
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print(F("Connected. Local IP: "));
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));
        Serial.println("Can't connect! Enter WiFi config mode...");
        Serial.println("Restart...");
        ESP.reset();
    }
 
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                 { request->send(200, "text/html", myESP8266page); });
    webServer.onNotFound([](AsyncWebServerRequest *request)
                         { request->send(404, "text/html", myNotFoundPage); });
 
    AsyncElegantOTA.begin(&webServer, myUsername, myPass); // Start ElegantOTA
    webServer.begin();
    Serial.println("FOTA server ready!");
}
 
void loop()
{
}