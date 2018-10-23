//BMP085   - pressure sensor
//DS18B20  - temperature sensor
//SI7021   - temperature and humidity sensor

//Pinout NODEMCU 1.0
//D4 - DS18B20
//D5 - SCL
//D6 - SDA
//3.3
//GND

#include <Wire.h>
//#include "i2c.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <TimeLib.h>
#include <Timezone.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include "DoubleResetDetector.h" // https://github.com/datacute/DoubleResetDetector
#include <WiFiManager.h> 
#include <FS.h>          //this needs to be first
#include <Ticker.h>

WiFiManager wifiManager;

//for LED status
Ticker ticker;

void tick()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}

#define verbose
#ifdef verbose
 #define DEBUG_PRINT(x)         Serial.print (x)
 #define DEBUG_PRINTDEC(x)      Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)       Serial.println (x)
 #define DEBUG_PRINTF(x, y)     Serial.printf (x, y)
 #define PORTSPEED 115200
 #define SERIAL_BEGIN           Serial.begin(PORTSPEED)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x)
 #define DEBUG_PRINTF(x, y)
 #define SERIAL_BEGIN
#endif 

bool isDebugEnabled()
{
#ifdef verbose
  return true;
#endif // verbose
  return false;
}


static const char ntpServerName[] = "tik.cesnet.cz";
//const int timeZone = 2;     // Central European Time
//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET = {"CET", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET);

WiFiClient client;
#define SDAPIN D6 //- GPI12 on ESP-201 module
#define SCLPIN D5 //- GPI14 on ESP-201 module
char static_ip[16] = "192.168.1.132";
char static_gw[16] = "192.168.1.1";
char static_sn[16] = "255.255.255.0";
ESP8266WebServer server(80);
WiFiUDP EthernetUdp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
time_t getNtpTime();
#include "SI7021.h"
SI7021 si7021;

//#define TEMPERATURE_DIVIDOR 100

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT       1
// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS       0

DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

#define WIFIENADDR        1
#define RTCVALIDFLAG      0xCAFEBABE
#define CFGFILE "/config.json"

char mqtt_server[40]    = "192.168.1.56";
char mqtt_port[6]       = "1883";
char mqtt_username[40];
char mqtt_key[20];
char mqtt_base[60];

bool shouldSaveConfig = false;


#define AIO_SERVER      "192.168.1.56"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "datel"
#define AIO_KEY         "hanka12"

float                 humidity, tempSI7021, dewPoint;
bool                  SI7021Present        = false;

#define PORTALTIMEOUT 30

String my_ssid;
String my_psk;


Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

#define MQTTBASE "/home/MeteoTest/"

/****************************** Feeds ***************************************/
Adafruit_MQTT_Publish _temperature             = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "Temperature");
Adafruit_MQTT_Publish _pressure                = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "Press");
Adafruit_MQTT_Publish _temperature085          = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "Temp085");
Adafruit_MQTT_Publish _humidity                = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "Humidity");
Adafruit_MQTT_Publish _tempSI7021              = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "Temp7021");
Adafruit_MQTT_Publish _dewpoint                = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "DewPoint");
Adafruit_MQTT_Publish _versionSW               = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "VersionSW");
Adafruit_MQTT_Publish _napeti                  = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "Napeti");
Adafruit_MQTT_Subscribe restart                = Adafruit_MQTT_Subscribe(&mqtt, MQTTBASE "restart");
Adafruit_MQTT_Publish _hb                      = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "HeartBeat");


#include <OneWire.h>
#include <DallasTemperature.h>
#define               ONE_WIRE_BUS          D7 //D4
OneWire onewire(ONE_WIRE_BUS); // pin for onewire DALLAS bus
DallasTemperature dsSensors(&onewire);
DeviceAddress tempDeviceAddress;
#define               NUMBER_OF_DEVICES     1
const unsigned long   measTime            = 750; //in ms
float                 temperature         = 0.f;
const unsigned long   measDelay           = 5000; //in ms
unsigned long         lastMeas            = 0;
bool                  DS18B20Present      = false;

const unsigned long   sendDelay           = 60000; //in ms
unsigned long         lastSend            = 0;

#include <Adafruit_BMP085.h> 
Adafruit_BMP085 bmp;
float                 high_above_sea      = 369.0;
float                 pressure            = 0.f;
float                 temperature085      = 0.f;
bool                  BMP085Present       = false;

unsigned long milisLastRunMinOld          = 0;

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

ADC_MODE(ADC_VCC);

byte status=0;
float versionSW                           = 1.70;
char versionSWString[]                    = "METEO v"; //SW name & version
uint32_t heartBeat                        = 10;

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

void setup() {
  SERIAL_BEGIN;
  DEBUG_PRINT(versionSWString);
  DEBUG_PRINTLN(versionSW);
  //set led pin as output
  pinMode(BUILTIN_LED, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);
  
  DEBUG_PRINTLN(ESP.getResetReason());
  if (ESP.getResetReason()=="Software/System restart") {
    heartBeat=1;
  } else if (ESP.getResetReason()=="Power on") {
    heartBeat=2;
  } else if (ESP.getResetReason()=="External System") {
    heartBeat=3;
  } else if (ESP.getResetReason()=="Hardware Watchdog") {
    heartBeat=4;
  } else if (ESP.getResetReason()=="Exception") {
    heartBeat=5;
  } else if (ESP.getResetReason()=="Software Watchdog") {
    heartBeat=6;
  } else if (ESP.getResetReason()=="Deep-Sleep Wake") {
    heartBeat=7;
  }
 
  
  if (shouldStartConfig()) {
    // uint32_t tmp;
    // ESP.rtcUserMemoryRead(WIFIENADDR, &tmp, sizeof(tmp));

    // // DIRTY hack to keep track of WAKE_RF_DEFAULT --> find a way to read WAKE_RF_*
    // if (tmp != RTCVALIDFLAG) {
      // drd.setRecentlyResetFlag();
      // tmp = RTCVALIDFLAG;
      // ESP.rtcUserMemoryWrite(WIFIENADDR, &tmp, sizeof(tmp));
      // Serial.println(F("reboot RFCAL"));
      // ESP.deepSleep(100000, WAKE_RFCAL);
      // delay(500);
    // } else {
      // tmp = 0;
      // ESP.rtcUserMemoryWrite(WIFIENADDR, &tmp, sizeof(tmp));
    // }

    ticker.attach(1, tick);

    // rescue if wifi credentials lost because of power loss
    if (!startConfiguration())
    {
      // test if ssid exists
      if (WiFi.SSID() == "" &&
          my_ssid != "" && my_psk != "")
      {
        connectBackupCredentials();
      }
    }
    // uint32_t left2sleep = 0;
    // ESP.rtcUserMemoryWrite(RTCSLEEPADDR, &left2sleep, sizeof(left2sleep));

    ticker.detach();
  }
  // to make sure we wake up with STA but AP
  WiFi.mode(WIFI_STA);

  uint8_t wait = 0;
  while (WiFi.status() == WL_DISCONNECTED)
  {
    delay(100);
    wait++;
    if (wait > 50)
      break;
  }
  DEBUG_PRINTLN(WiFi.status());

  if (WiFi.status() == WL_CONNECTED) {
    DEBUG_PRINT("IP: ");
    DEBUG_PRINTLN(WiFi.localIP());
    delay(100); // workaround for https://github.com/esp8266/Arduino/issues/2750
  }
  // else
  // {
    // connectBackupCredentials();
    // SerialOut("failed to connect");
  // }

  
  Wire.begin();
  
  DEBUG_PRINT("Probe SI7021: ");
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
  
  mqtt.subscribe(&restart);


  server.on ( "/", handleRoot );
  server.begin();
  DEBUG_PRINTLN ( "HTTP server started!!" );
  
  DEBUG_PRINTLN("Setup TIME");
  EthernetUdp.begin(localPort);
  DEBUG_PRINT("Local port: ");
  DEBUG_PRINTLN(EthernetUdp.localPort());
  DEBUG_PRINTLN("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  
  // DEBUG_PRINT("Time Status after setSyncProvider:");
  // DEBUG_PRINTLN(timeStatus());

  if (timeStatus()== timeNotSet) {
    //ESP.reset();
  }
 
  //while(timeStatus()== timeNotSet)
     ; // wait until the time is set by the sync provider
  printSystemTime();
  
  //OTA
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("meteo");

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

  DEBUG_PRINTLN(" Ready");
  //DEBUG_PRINT("IP address: ");
  //DEBUG_PRINTLN(WiFi.localIP());
  
  ticker.detach();
  //keep LED on
  digitalWrite(BUILTIN_LED, LOW);
}

void loop() {
  server.handleClient();
  if (millis() - lastMeas >= measDelay) {
    digitalWrite(BUILTIN_LED, LOW);
    lastMeas = millis();
    
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
  }

  if (millis() - lastSend >= sendDelay) {
    lastSend = millis();
    sendDataHA();
  }

  if (millis() - milisLastRunMinOld > 60000) {
    milisLastRunMinOld = millis();
    if (MQTT_connect()) {
      if (! _hb.publish(heartBeat)) {
        DEBUG_PRINTLN("Send HB failed");
      } else {
        DEBUG_PRINTLN("Send HB OK!");
      }
      if (! _versionSW.publish(versionSW)) {
        DEBUG_PRINTLN(F("Send verSW failed!"));
      } else {
        DEBUG_PRINTLN(F("Send verSW OK!"));
      }
      if (! _napeti.publish(ESP.getVcc())) {
        DEBUG_PRINTLN("Send napeti failed");
      } else {
        DEBUG_PRINTLN("Send napeti OK!");
      }
    }
    heartBeat++;
  }

  if (MQTT_connect()) {
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(2000))) {
      if (subscription == &restart) {
        char *pNew = (char *)restart.lastread;
        uint32_t pPassw=atol(pNew); 
        if (pPassw==650419) {
          DEBUG_PRINT(F("Restart ESP now!"));
          ESP.restart();
         } else {
          DEBUG_PRINT(F("Wrong password."));
        }
      }
    }
  }
  ArduinoOTA.handle();
}


bool shouldStartConfig() { 
// we make sure that configuration is properly set and we are not woken by
  // RESET button
  // ensure this was called

  rst_info *_reset_info = ESP.getResetInfoPtr();
  uint8_t _reset_reason = _reset_info->reason;

  // The ESP reset info is sill buggy. see http://www.esp8266.com/viewtopic.php?f=32&t=8411
  // The reset reason is "5" (woken from deep-sleep) in most cases (also after a power-cycle)
  // I added a single reset detection as workaround to enter the config-mode easier
  DEBUG_PRINT("Boot-Mode: ");
  DEBUG_PRINTLN(_reset_reason);
  
  bool _poweredOnOffOn = _reset_reason == REASON_DEFAULT_RST || _reset_reason == REASON_EXT_SYS_RST;
  if (_poweredOnOffOn) {
    DEBUG_PRINTLN("Power-cycle or reset detected, config mode");
  }
  _poweredOnOffOn = false;
  
  bool _dblreset = drd.detectDoubleReset();
  if (_dblreset) {
    DEBUG_PRINTLN("\Double Reset detected");
  }
  
  bool _validConf = readConfig();
  if (!_validConf) {
    DEBUG_PRINTLN("ERROR config corrupted");
  }
  
  bool _wifiCred = (WiFi.SSID() != "");
  uint8_t c = 0;
  if (!_wifiCred)
    WiFi.begin();
  while (!_wifiCred) {
    if (c > 10)
      break;
    DEBUG_PRINTLN('.');
    delay(100);
    c++;
    _wifiCred = (WiFi.SSID() != "");
  }
  if (!_wifiCred) {
    DEBUG_PRINTLN("ERROR no Wifi credentials");
  }

  // DEBUG_PRINTLN(_validConf);
  // DEBUG_PRINTLN(_dblreset);
  // DEBUG_PRINTLN(_wifiCred);
  // DEBUG_PRINTLN(_poweredOnOffOn);
  
  if (_validConf && !_dblreset && _wifiCred && !_poweredOnOffOn) {
    DEBUG_PRINTLN(F("Normal mode"));
    return false;
  } // config mode
  else {
    DEBUG_PRINTLN(F("Going to Config Mode"));
    return true;
  }
}

void sendDataHA() {
  printSystemTime();
  DEBUG_PRINTLN(" I am sending data from Meteo unit to HomeAssistant");
  if (MQTT_connect()) {
    if (! _temperature.publish(temperature)) {
      DEBUG_PRINTLN("Temperature failed");
    } else {
      DEBUG_PRINTLN("Temperature OK!");
    }  
    if (! _pressure.publish(pressure)) {
      DEBUG_PRINTLN("Pressure failed");
    } else {
      DEBUG_PRINTLN("Pressure OK!");
    }  
    if (! _temperature085.publish(temperature085)) {
      DEBUG_PRINTLN("Temperature085 failed");
    } else {
      DEBUG_PRINTLN("Temperature085 OK!");
    }  
    if (! _humidity.publish(humidity)) {
      DEBUG_PRINTLN("Humidity failed");
    } else {
      DEBUG_PRINTLN("Humidity OK!");
    }  
    if (! _tempSI7021.publish(tempSI7021)) {
      DEBUG_PRINTLN("failed");
    } else {
      DEBUG_PRINTLN("OK!");
    }  
    if (! _dewpoint.publish(dewPoint)) {
      DEBUG_PRINTLN("DewPoint failed");
    } else {
      DEBUG_PRINTLN("DewPoint OK!");
    }  
  }
}

 
bool MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return true;
  }

  DEBUG_PRINT("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
     DEBUG_PRINTLN(mqtt.connectErrorString(ret));
     DEBUG_PRINTLN("Retrying MQTT connection in 5 seconds...");
     mqtt.disconnect();
     delay(5000);  // wait 5 seconds
     retries--;
     if (retries == 0) {
       return false;
     }
  }
  DEBUG_PRINTLN("MQTT Connected!");
  return true;
}

void generateHTML() {
  DEBUG_PRINTLN("new client");
  // an http request ends with a blank line
  boolean currentLineIsBlank = true;
  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      Serial.write(c);
      // if you've gotten to the end of the line (received a newline
      // character) and the line is blank, the http request has ended,
      // so you can send a reply
      if (c == '\n' && currentLineIsBlank) {
        // send a standard http response header
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Connection: close");  // the connection will be closed after completion of the response
        client.println("Refresh: 5");  // refresh the page automatically every 5 sec
        client.println();
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        // output the value of each analog input pin
        for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
          int sensorReading = analogRead(analogChannel);
          client.print("analog input ");
          client.print(analogChannel);
          client.print(" is ");
          client.print(sensorReading);
          client.println("<br />");
        }
        client.println("</html>");
        break;
      }
      if (c == '\n') {
        // you're starting a new line
        currentLineIsBlank = true;
      } else if (c != '\r') {
        // you've gotten a character on the current line
        currentLineIsBlank = false;
      }
    }
  }
  // give the web browser time to receive the data
  delay(1);
  // close the connection:
  client.stop();
  DEBUG_PRINTLN("client disconnected");
}

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

void printSystemTime(){
  DEBUG_PRINT(day());
  DEBUG_PRINT(".");
  DEBUG_PRINT(month());
  DEBUG_PRINT(".");
  DEBUG_PRINT(year());
  DEBUG_PRINT(" ");
  DEBUG_PRINT(hour());
  printDigits(minute());
  printDigits(second());
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding
  // colon and leading 0
  DEBUG_PRINT(":");
  if(digits < 10)
    DEBUG_PRINT('0');
  DEBUG_PRINT(digits);
}

float calcDewPoint (float humidity, float temperature)  
{  
    float logEx;  
    logEx = 0.66077 + (7.5 * temperature) / (237.3 + temperature)  
            + (log10(humidity) - 2);  
    return (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);  
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
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());

        if (json.success()) {
          DEBUG_PRINTLN(F("Parsed json"));

          // if (json.containsKey("Name"))
            // strcpy(my_name, json["Name"]);
          // if (json.containsKey("Token"))
            // strcpy(my_token, json["Token"]);
          // if (json.containsKey("Server"))
            // strcpy(my_server, json["Server"]);
          // if (json.containsKey("Sleep"))
            // my_sleeptime = json["Sleep"];
          // if (json.containsKey("API"))
            // my_api = json["API"];
          // if (json.containsKey("Port"))
            // my_port = json["Port"];
          // if (json.containsKey("URL"))
            // strcpy(my_url, json["URL"]);
          // if (json.containsKey("Vfact"))
            // my_vfact = json["Vfact"];

          if (json.containsKey("SSID")) {
            my_ssid = (const char *)json["SSID"];
          }
          if (json.containsKey("PSK")) {
            my_psk = (const char *)json["PSK"];
          }
          DEBUG_PRINTLN(F("Parsed config:"));
          if (isDebugEnabled)
            json.printTo(Serial);
          return true;
        }
        else {
          DEBUG_PRINTLN(F("ERROR: failed to load json config"));
          return false;
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


bool startConfiguration()
{

  WiFiManager wifiManager;

  wifiManager.setConfigPortalTimeout(PORTALTIMEOUT);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setBreakAfterConfig(true);

                                   
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
  WiFiManagerParameter custom_mqtt_uname("uname", "mqtt username", mqtt_username, 40);
  WiFiManagerParameter custom_mqtt_key("key", "mqtt password", mqtt_key, 20);
  WiFiManagerParameter custom_mqtt_base("base", "mqtt topic", mqtt_base, 60);

  //set static ip
  IPAddress _ip,_gw,_sn;
  _ip.fromString(static_ip);
  _gw.fromString(static_gw);
  _sn.fromString(static_sn);

  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_uname);
  wifiManager.addParameter(&custom_mqtt_key);
  wifiManager.addParameter(&custom_mqtt_base);


  // wifiManager._ssid = my_ssid;
  // wifiManager._pass = my_psk;

  DEBUG_PRINTLN(F("started Portal"));
  wifiManager.startConfigPortal("Meteo", "password");

  // save the custom parameters to FS
  if (shouldSaveConfig) {
    // Wifi config
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);

    return saveConfig();
  }
  return false;
}

// callback notifying us of the need to save config
void saveConfigCallback()
{
  DEBUG_PRINTLN(F("Should save config"));
  // WiFi.setAutoReconnect(true);
  shouldSaveConfig = true;
}

bool connectBackupCredentials()
{
  WiFi.disconnect();
  WiFi.begin(my_ssid.c_str(), my_psk.c_str());
  DEBUG_PRINTLN(F("Rescue Wifi credentials"));
  delay(100);
}


bool saveConfig()
{
  DEBUG_PRINTLN(F("saving config..."));

  // if SPIFFS is not usable
  if (!SPIFFS.begin() || !SPIFFS.exists(CFGFILE) ||
      !SPIFFS.open(CFGFILE, "w"))
  {
    DEBUG_PRINTLN(F("\nneed to format SPIFFS: "));
    SPIFFS.end();
    SPIFFS.begin();
    DEBUG_PRINTLN(SPIFFS.format());
  }

  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();

  // json["Name"] = my_name;
  // json["Token"] = my_token;
  // json["Sleep"] = my_sleeptime;
  // // first reboot is for test
  // my_sleeptime = 1;
  // json["Server"] = my_server;
  // json["API"] = my_api;
  // json["Port"] = my_port;
  // json["URL"] = my_url;
  // json["Vfact"] = my_vfact;

  // Store current Wifi credentials
  json["SSID"] = WiFi.SSID();
  json["PSK"] = WiFi.psk();

  File configFile = SPIFFS.open(CFGFILE, "w+");
  if (!configFile)
  {
    DEBUG_PRINTLN(F("failed to open config file for writing"));
    SPIFFS.end();
    return false;
  }
  else
  {
    if (isDebugEnabled)
      json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    SPIFFS.end();
    DEBUG_PRINTLN(F("saved successfully"));
    return true;
  }
}
