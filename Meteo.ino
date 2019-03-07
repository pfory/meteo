//BMP085   - pressure sensor
//DS18B20  - temperature sensor
//SI7021   - temperature and humidity sensor

//Pinout NODEMCU 1.0
//D4 - DS18B20
//D5 - SCL
//D6 - SDA
//3.3
//GND

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
//#include <DNSServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <FS.h>          //this needs to be first
#include <Ticker.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include "DoubleResetDetector.h" // https://github.com/datacute/DoubleResetDetector
#include "Sender.h"
#include "SI7021.h"
#include <Wire.h>

SI7021 si7021;

#define ota
#ifdef ota
#include <ArduinoOTA.h>
#define HOSTNAMEOTA   "meteo"
#endif

#define serverHTTP
#ifdef serverHTTP
#include <ESP8266WebServer.h>
ESP8266WebServer server(80);
#endif

#define time
#ifdef time
#include <TimeLib.h>
#include <Timezone.h>
WiFiUDP EthernetUdp;
static const char     ntpServerName[]       = "tik.cesnet.cz";
//const int timeZone = 2;     // Central European Time
//Central European Time (Frankfurt, Paris)
TimeChangeRule        CEST                  = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule        CET                   = {"CET", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET);
unsigned int          localPort             = 8888;  // local port to listen for UDP packets
time_t getNtpTime();
#endif


#define verbose
#ifdef verbose
 #define PORTSPEED 115200
 #define DEBUG_PRINT(x)         Serial.print (x)
 #define DEBUG_PRINTDEC(x)      Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)       Serial.println (x)
 #define DEBUG_PRINTF(x, y)     Serial.printf (x, y)
 #define SERIAL_BEGIN           Serial.begin(PORTSPEED)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x)
 #define DEBUG_PRINTF(x, y)
 #define SERIAL_BEGIN
#endif 

char                  mqtt_server[40]       = "192.168.1.56";
uint16_t              mqtt_port             = 1883;
char                  mqtt_username[40]     = "datel";
char                  mqtt_key[20]          = "hanka12";
char                  mqtt_base[60]         = "/home/Meteo";
char                  static_ip[16]         = "192.168.1.102";
char                  static_gw[16]         = "192.168.1.1";
char                  static_sn[16]         = "255.255.255.0";

#define DRD_TIMEOUT       1
// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS       0
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

#define CFGFILE "/config.json"

float                 humidity, tempSI7021, dewPoint;
bool                  SI7021Present        = false;

const unsigned long   sendDelay             = 30000; //in ms
const unsigned long   sendStatDelay         = 60000;
    
float versionSW                             = 1.81;
char versionSWString[]                      = "METEO v"; //SW name & version
uint32_t heartBeat                          = 0;

ADC_MODE(ADC_VCC);

bool isDebugEnabled()
{
#ifdef verbose
  return true;
#endif // verbose
  return false;
}

//for LED status
#include <Ticker.h>
Ticker ticker;

#define SDAPIN D6 //- GPI12 on ESP-201 module
#define SCLPIN D5 //- GPI14 on ESP-201 module

#include <timer.h>
auto timer = timer_create_default(); // create a timer with default settings
Timer<> default_timer; // save as above

#include <OneWire.h>
#include <DallasTemperature.h>
#define               ONE_WIRE_BUS          D4 //D7
OneWire onewire(ONE_WIRE_BUS); // pin for onewire DALLAS bus
DallasTemperature dsSensors(&onewire);
DeviceAddress tempDeviceAddress;
#define               NUMBER_OF_DEVICES     1
const unsigned long   measTime            = 750; //in ms
float                 temperature         = 0.f;
const unsigned long   measDelay           = 5000; //in ms
bool                  DS18B20Present      = false;

#include <Adafruit_BMP085.h> 
Adafruit_BMP085 bmp;
float                 high_above_sea      = 369.0;
float                 pressure            = 0.f;
float                 temperature085      = 0.f;
bool                  BMP085Present       = false;


#ifdef serverHTTP
void handleRoot() {
	char temp[600];
  // DEBUG_PRINT(year());
  // DEBUG_PRINT(month());
  // DEBUG_PRINT(day());
  // DEBUG_PRINT(hour());
  // DEBUG_PRINT(minute());
  // DEBUG_PRINT(second());
  printSystemTime();
  DEBUG_PRINTLN(" Client request");
  digitalWrite(BUILTIN_LED, LOW);
  
	snprintf ( temp, 400,
      "<html>\
        <head>\
          <meta charset='UTF-8'>\
        </head>\
        <body>\
          T2899BDCF02000076,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%s%d.%02d<br />\
          Humidity,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%d.00<br />\
          Press,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%d.00<br />\
          DewPoint,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%s%d.%02d<br />\
        </body>\
      </html>",
      year(), month(), day(), hour(), minute(), second(),
      temperature<0 && temperature>-1 ? "-":"",
      (int)temperature, 
      abs((temperature - (int)temperature) * 100),
      year(), month(), day(), hour(), minute(), second(),
      (int)humidity,
      year(), month(), day(), hour(), minute(), second(),
      (int)pressure,
      year(), month(), day(), hour(), minute(), second(),
      dewPoint<0 && dewPoint>-1 ? "-":"",
      (int)dewPoint, 
      abs((dewPoint - (int)dewPoint) * 100)
	);
	server.send ( 200, "text/html", temp );
  digitalWrite(BUILTIN_LED, HIGH);
}
#endif

void tick()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}
  
//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  DEBUG_PRINTLN("Should save config");
  shouldSaveConfig = true;
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  DEBUG_PRINTLN("Entered config mode");
  DEBUG_PRINTLN(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  DEBUG_PRINTLN(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void setup() {
  // put your setup code here, to run once:
  SERIAL_BEGIN;
  DEBUG_PRINT(versionSWString);
  DEBUG_PRINTLN(versionSW);
 
  pinMode(BUILTIN_LED, OUTPUT);
  ticker.attach(1, tick);
  bool _dblreset = drd.detectDoubleReset();
    
  WiFi.printDiag(Serial);
    
  bool validConf = readConfig();
  if (!validConf) {
    DEBUG_PRINTLN(F("ERROR config corrupted"));
  }
  
  rst_info *_reset_info = ESP.getResetInfoPtr();
  uint8_t _reset_reason = _reset_info->reason;
  DEBUG_PRINT("Boot-Mode: ");
  DEBUG_PRINTLN(_reset_reason);
  heartBeat = _reset_reason;
  
  // drd.stop();

  // if (_dblreset) {
    // WiFi.disconnect(); //  this alone is not enough to stop the autoconnecter
    // WiFi.mode(WIFI_AP);
  // }
  
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();
  
  //wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  IPAddress _ip,_gw,_sn;
  _ip.fromString(static_ip);
  _gw.fromString(static_gw);
  _sn.fromString(static_sn);

  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  
  DEBUG_PRINTLN(_ip);
  DEBUG_PRINTLN(_gw);
  DEBUG_PRINTLN(_sn);

  //if (WiFi.SSID()!="") wifiManager.setConfigPortalTimeout(60); //If no access point name has been previously entered disable timeout.

  
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  
    //DEBUG_PRINTLN("Double reset detected. Config mode.");

  WiFiManagerParameter custom_mqtt_server("mqtt_server", "MQTT server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT server port", String(mqtt_port).c_str(), 5);
  WiFiManagerParameter custom_mqtt_uname("mqtt_uname", "MQTT username", mqtt_username, 40);
  WiFiManagerParameter custom_mqtt_key("mqtt_key", "MQTT password", mqtt_key, 20);
  WiFiManagerParameter custom_mqtt_base("mqtt_base", "MQTT topic end without /", mqtt_base, 60);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_uname);
  wifiManager.addParameter(&custom_mqtt_key);
  wifiManager.addParameter(&custom_mqtt_base);

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  
  wifiManager.setTimeout(30);
  wifiManager.setConnectTimeout(10); 
  wifiManager.setConfigPortalTimeout(60);
  //wifiManager.setBreakAfterConfig(true);
  
  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("Meteo", "password")) { 
    DEBUG_PRINTLN("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  } 
  
  validateInput(custom_mqtt_server.getValue(), mqtt_server);
  mqtt_port = String(custom_mqtt_port.getValue()).toInt();
  validateInput(custom_mqtt_uname.getValue(), mqtt_username);
  validateInput(custom_mqtt_key.getValue(), mqtt_key);
  validateInput(custom_mqtt_base.getValue(), mqtt_base);
  
  if (shouldSaveConfig) {
    saveConfig();
  }
  
  //if you get here you have connected to the WiFi
  DEBUG_PRINTLN("CONNECTED");
  DEBUG_PRINT("Local ip : ");
  DEBUG_PRINTLN(WiFi.localIP());
  DEBUG_PRINTLN(WiFi.subnetMask());

#ifdef serverHTTP
  server.on ( "/", handleRoot );
  server.begin();
  DEBUG_PRINTLN ( "HTTP server started!!" );
#endif

#ifdef time
  DEBUG_PRINTLN("Setup TIME");
  EthernetUdp.begin(localPort);
  DEBUG_PRINT("Local port: ");
  DEBUG_PRINTLN(EthernetUdp.localPort());
  DEBUG_PRINTLN("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  
  printSystemTime();
#endif

#ifdef ota
  //OTA
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAMEOTA);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    // String type;
    // if (ArduinoOTA.getCommand() == U_FLASH)
      // type = "sketch";
    // else // U_SPIFFS
      // type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    //DEBUG_PRINTLN("Start updating " + type);
    DEBUG_PRINTLN("Start updating ");
  });
  ArduinoOTA.onEnd([]() {
   DEBUG_PRINTLN("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_PRINTF("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_PRINTF("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) DEBUG_PRINTLN("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) DEBUG_PRINTLN("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) DEBUG_PRINTLN("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) DEBUG_PRINTLN("Receive Failed");
    else if (error == OTA_END_ERROR) DEBUG_PRINTLN("End Failed");
  });
  ArduinoOTA.begin();
#endif


  Wire.begin();
  
  DEBUG_PRINT("\nProbe SI7021: ");
  if (si7021.begin(SDAPIN, SCLPIN)) {
    SI7021Present = true;
  }

  if (SI7021Present == true) {
    DEBUG_PRINTLN("Sensor found.");
  } else {
    DEBUG_PRINTLN("Sensor missing!!!!");
  }

  DEBUG_PRINT("Probe DS18B20: ");
  dsSensors.begin(); 
  if (dsSensors.getDeviceCount()>0) {
    DEBUG_PRINTLN("Sensor found.");
    DS18B20Present = true;
    dsSensors.setResolution(12);
    dsSensors.setWaitForConversion(false);
  } else {
    DEBUG_PRINTLN("Sensor missing!!!!");
  }

  DEBUG_PRINT("Probe BMP085: ");
  if (bmp.begin()==1) {
    BMP085Present = true;
    DEBUG_PRINTLN("Sensor found.");
    } else {
    DEBUG_PRINTLN("Sensor missing!!!");
  }

  //setup timers
  timer.every(measDelay, meass);
  timer.every(sendDelay, sendDataHA);
  timer.every(sendStatDelay, sendStatisticHA);

  DEBUG_PRINTLN(" Ready");
 
  ticker.detach();
  //keep LED on
  digitalWrite(BUILTIN_LED, HIGH);
}



void loop() {
  timer.tick(); // tick the timer
#ifdef serverHTTP
  server.handleClient();
#endif

#ifdef ota
  ArduinoOTA.handle();
#endif
}


bool meass(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  
  if (DS18B20Present) {
    dsSensors.requestTemperatures(); // Send the command to get temperatures
    delay(measTime);
    if (dsSensors.getCheckForConversion()==true) {
      temperature = dsSensors.getTempCByIndex(0);
    }
    DEBUG_PRINTLN("-------------");
    DEBUG_PRINT("Temperature DS18B20: ");
    DEBUG_PRINT(temperature); 
    DEBUG_PRINTLN(" *C");
  } else {
    temperature = 0.0; //dummy
  }
  
  if (SI7021Present) {
    humidity=si7021.getHumidityPercent();
    tempSI7021=si7021.getCelsiusHundredths() / 100;
    //si7021.triggerMeasurement();

    if (humidity>100) {
      humidity = 100;
    }
    DEBUG_PRINT("Temperature SI7021: ");
    DEBUG_PRINT(tempSI7021);
    DEBUG_PRINTLN(" *C");
    DEBUG_PRINT("Humidity SI7021: ");
    DEBUG_PRINT(humidity);
    DEBUG_PRINTLN(" %Rh");
  } else {
    humidity = 0.0;    //dummy
    tempSI7021 = 0.0;  //dummy
  }
  
  if (BMP085Present) {
    DEBUG_PRINT("Temperature BMP085: ");
    temperature085 = bmp.readTemperature();
    pressure = bmp.readSealevelPressure(high_above_sea);
    DEBUG_PRINT(temperature085);
    DEBUG_PRINTLN(" *C");
    DEBUG_PRINT("Pressure: ");
    DEBUG_PRINT(pressure);
    DEBUG_PRINTLN(" Pa");
  } else {
    temperature085 = 0.0;  //dummy
    pressure = 0;     //Pa - dummy
  }
  
  dewPoint = calcDewPoint(humidity, temperature);
  
  digitalWrite(BUILTIN_LED, HIGH);

  return true;
}

float calcDewPoint (float humidity, float temperature)  
{  
    float logEx;  
    logEx = 0.66077 + (7.5 * temperature) / (237.3 + temperature)  
            + (log10(humidity) - 2);  
    return (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);  
}

void validateInput(const char *input, char *output)
{
  String tmp = input;
  tmp.trim();
  tmp.replace(' ', '_');
  tmp.toCharArray(output, tmp.length() + 1);
}

bool saveConfig() {
  DEBUG_PRINTLN(F("Saving config..."));

  // if SPIFFS is not usable
  if (!SPIFFS.begin() || !SPIFFS.exists(CFGFILE) ||
      !SPIFFS.open(CFGFILE, "w"))
  {
    DEBUG_PRINTLN(F("Need to format SPIFFS: "));
    SPIFFS.end();
    SPIFFS.begin();
    DEBUG_PRINTLN(SPIFFS.format());
  }

  StaticJsonDocument<1024> doc;

  doc["MQTT_server"] = mqtt_server;
  doc["MQTT_port"]   = mqtt_port;
  doc["MQTT_uname"]  = mqtt_username;
  doc["MQTT_pwd"]    = mqtt_key;
  doc["MQTT_base"]   = mqtt_base;
  
  doc["ip"] = WiFi.localIP().toString();
  doc["gateway"] = WiFi.gatewayIP().toString();
  doc["subnet"] = WiFi.subnetMask().toString();

  // Store current Wifi credentials
  // doc["SSID"]        = WiFi.SSID();
  // doc["PSK"]         = WiFi.psk();

  File configFile = SPIFFS.open(CFGFILE, "w+");
  if (!configFile) {
    DEBUG_PRINTLN(F("Failed to open config file for writing"));
    SPIFFS.end();
    return false;
  } else {
    if (isDebugEnabled) {
      serializeJson(doc, Serial);
    }
    serializeJson(doc, configFile);
    configFile.close();
    SPIFFS.end();
    DEBUG_PRINTLN(F("\nSaved successfully"));
    return true;
  }
}

bool readConfig() {
  DEBUG_PRINT(F("Mounting FS..."));

  if (SPIFFS.begin()) {
    DEBUG_PRINTLN(F(" mounted!"));
    if (SPIFFS.exists(CFGFILE)) {
      // file exists, reading and loading
      DEBUG_PRINTLN(F("Reading config file"));
      File configFile = SPIFFS.open(CFGFILE, "r");
      if (configFile) {
        DEBUG_PRINTLN(F("Opened config file"));
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, configFile);
        JsonObject obj=doc.as<JsonObject>();

        if (error) {
          DEBUG_PRINTLN(F("Failed to read file, using default configuration"));
          return false;
        } else {
          DEBUG_PRINTLN(F("Parsed json"));
          
          if (obj.containsKey("MQTT_server")) {
            strcpy(mqtt_server, obj["MQTT_server"]);
            DEBUG_PRINT(F("MQTT server: "));
            DEBUG_PRINTLN(mqtt_server);
          }
          if (obj.containsKey("MQTT_port")) {
            mqtt_port = obj["MQTT_port"];
            DEBUG_PRINT(F("MQTT port: "));
            DEBUG_PRINTLN(mqtt_port);
          }
          if (obj.containsKey("MQTT_uname")) {
            strcpy(mqtt_username, obj["MQTT_uname"]);
            DEBUG_PRINT(F("MQTT username: "));
            DEBUG_PRINTLN(mqtt_username);
          }
          if (obj.containsKey("MQTT_pwd")) {
            strcpy(mqtt_key, obj["MQTT_pwd"]);
            DEBUG_PRINT(F("MQTT password: "));
            DEBUG_PRINTLN(mqtt_key);
          }
          if (obj.containsKey("MQTT_base")) {
            strcpy(mqtt_base, obj["MQTT_base"]);
            DEBUG_PRINT(F("MQTT base: "));
            DEBUG_PRINTLN(mqtt_base);
          }
          // if (obj.containsKey("SSID")) {
            // my_ssid = (const char *)obj["SSID"];
          // }
          // if (obj.containsKey("PSK")) {
            // my_psk = (const char *)obj["PSK"];
          // }
          
          if(obj["ip"]) {
            DEBUG_PRINTLN("setting custom ip from config");
            strcpy(static_ip, obj["ip"]);
            strcpy(static_gw, obj["gateway"]);
            strcpy(static_sn, obj["subnet"]);
            DEBUG_PRINTLN(static_ip);
          } else {
            DEBUG_PRINTLN("no custom ip in config");
          }

          
          DEBUG_PRINTLN(F("Parsed config:"));
          DEBUG_PRINTLN(error.c_str());
          DEBUG_PRINTLN();  
          return true;
        }
      }
      DEBUG_PRINTLN(F("ERROR: unable to open config file"));
    } else {
      DEBUG_PRINTLN(F("ERROR: config file not exist"));
    }
  } else {
    DEBUG_PRINTLN(F(" ERROR: failed to mount FS!"));
  }
  return false;
}

bool sendDataHA(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  printSystemTime();
  DEBUG_PRINTLN(F(" - I am sending data to HA"));
  
//Adafruit_MQTT_Subscribe restart                = Adafruit_MQTT_Subscribe(&mqtt, MQTTBASE "restart");
  SenderClass sender;
  sender.add("Temperature", temperature);
  sender.add("Press", pressure);
  sender.add("Temp085", temperature085);
  sender.add("Humidity", humidity);
  sender.add("Temp7021", tempSI7021);
  sender.add("DewPoint", dewPoint);
  DEBUG_PRINTLN(F("Calling MQTT"));

  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);
  digitalWrite(BUILTIN_LED, HIGH);
  return true;
}

bool sendStatisticHA(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  printSystemTime();
  DEBUG_PRINTLN(F(" - I am sending statistic to HA"));

  SenderClass sender;
  sender.add("VersionSW", versionSW);
  sender.add("Napeti",  ESP.getVcc());
  sender.add("HeartBeat", heartBeat++);
  sender.add("RSSI", WiFi.RSSI());
  DEBUG_PRINTLN(F("Calling MQTT"));
  
  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);
  digitalWrite(BUILTIN_LED, HIGH);
  return true;
}


#ifdef time
/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  //IPAddress ntpServerIP; // NTP server's ip address
  IPAddress ntpServerIP = IPAddress(195, 113, 144, 201);

  while (EthernetUdp.parsePacket() > 0) ; // discard any previously received packets
  DEBUG_PRINTLN("Transmit NTP Request");
  // get a random server from the pool
  //WiFi.hostByName(ntpServerName, ntpServerIP);
  DEBUG_PRINT(ntpServerName);
  DEBUG_PRINT(": ");
  DEBUG_PRINTLN(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = EthernetUdp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      DEBUG_PRINTLN("Receive NTP Response");
      EthernetUdp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      DEBUG_PRINT("Seconds since Jan 1 1900 = " );
      DEBUG_PRINTLN(secsSince1900);

      // now convert NTP time into everyday time:
      DEBUG_PRINT("Unix time = ");
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      unsigned long epoch = secsSince1900 - seventyYears;
      // print Unix time:
      DEBUG_PRINTLN(epoch);
	  
      TimeChangeRule *tcr;
      time_t utc;
      utc = epoch;
      
      return CE.toLocal(utc, &tcr);
      //return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  DEBUG_PRINTLN("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  EthernetUdp.beginPacket(address, 123); //NTP requests are to port 123
  EthernetUdp.write(packetBuffer, NTP_PACKET_SIZE);
  EthernetUdp.endPacket();
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding
  // colon and leading 0
  DEBUG_PRINT(":");
  if(digits < 10)
    DEBUG_PRINT('0');
  DEBUG_PRINT(digits);
}

#endif

void printSystemTime(){
#ifdef time
  DEBUG_PRINT(day());
  DEBUG_PRINT(".");
  DEBUG_PRINT(month());
  DEBUG_PRINT(".");
  DEBUG_PRINT(year());
  DEBUG_PRINT(" ");
  DEBUG_PRINT(hour());
  printDigits(minute());
  printDigits(second());
#endif
}
