//BMP085   - pressure sensor
//DS18B20  - temperature sensor
//SI7021   - temperature and humidity sensor

//Pinout NODEMCU 1.0
//D4 - DS18B20
//D5 - SCL
//D6 - SDA
//3.3
//GND

#include "Configuration.h"

SI7021 si7021;

#ifdef serverHTTP
ESP8266WebServer server(81);
#endif

#ifdef time
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

//#define CFGFILE "/config.json"

float                 humidity, tempSI7021, dewPoint;
bool                  SI7021Present        = false;

uint32_t heartBeat                          = 0;

WiFiClient espClient;
PubSubClient client(espClient);

ADC_MODE(ADC_VCC);

bool isDebugEnabled() {
#ifdef verbose
  return true;
#endif // verbose
  return false;
}

//for LED status
Ticker ticker;

#define SDAPIN D6
#define SCLPIN D5

auto timer = timer_create_default(); // create a timer with default settings
Timer<> default_timer; // save as above

OneWire onewire(ONE_WIRE_BUS); // pin for onewire DALLAS bus
DallasTemperature dsSensors(&onewire);
DeviceAddress tempDeviceAddress;
#define               NUMBER_OF_DEVICES     1
float                 temperature         = 0.f;
bool                  DS18B20Present      = false;

Adafruit_BMP085 bmp;
float                 high_above_sea      = 369.0;
float                 pressure            = 0.f;
float                 temperature085      = 0.f;
bool                  BMP085Present       = false;

DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

#ifdef serverHTTP
void handleRoot() {
	char temp[600];
  DEBUG_PRINT("Web client request...");
  digitalWrite(BUILTIN_LED, LOW);
  
  // temperature = -4.625;
  // humidity=96.5f;
  // pressure=102370f;
  // dewPoint=1.5f;
  
  snprintf(temp, 600, "<html><head><meta charset='UTF-8'></head><body>\
           T2899BDCF02000076,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%s%d.%02d<br />\
           Humidity,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%d.00<br />\
           Press,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%d.00<br />\
           DewPoint,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%s%d.%02d<br />\
        </body>\
      </html>", year(), month(), day(), hour(), minute(), second(),
      temperature<0 && temperature>-1 ? "-":"",
      (int)temperature, (int)abs((temperature - (int)temperature) * 100),
      year(), month(), day(), hour(), minute(), second(),
      (int)humidity,
      year(), month(), day(), hour(), minute(), second(),
      (int)(round(pressure/100)),
      year(), month(), day(), hour(), minute(), second(),
      dewPoint<0 && dewPoint>-1 ? "-":"",
      (int)dewPoint, (int)abs((dewPoint - (int)dewPoint) * 100)
  );
	server.send ( 200, "text/html", temp );
  DEBUG_PRINTLN("done.");
  digitalWrite(BUILTIN_LED, HIGH);
}
#endif

void tick() {
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}
  

//MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
  char * pEnd;
  String val =  String();
  DEBUG_PRINT("\nMessage arrived [");
  DEBUG_PRINT(topic);
  DEBUG_PRINT("] ");
  for (int i=0;i<length;i++) {
    DEBUG_PRINT((char)payload[i]);
    val += (char)payload[i];
  }
  DEBUG_PRINTLN();
  
  if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_restart)).c_str())==0) {
    DEBUG_PRINT("RESTART");
    ESP.restart();
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_topic_netinfo)).c_str())==0) {
    sendNetInfoMQTT();
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_config_portal)).c_str())==0) {
    startConfigPortal();
  } else if (strcmp(topic, (String(mqtt_base) + "/" + String(mqtt_config_portal_stop)).c_str())==0) {
    stopConfigPortal();
  }
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

WiFiManager wifiManager;

void setup() {
  // put your setup code here, to run once:
  SERIAL_BEGIN;
  DEBUG_PRINT(F(SW_NAME));
  DEBUG_PRINT(F(" "));
  DEBUG_PRINTLN(F(VERSION));
 
  pinMode(BUILTIN_LED, OUTPUT);
  ticker.attach(1, tick);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT);
  wifiManager.setConnectTimeout(CONNECT_TIMEOUT);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.setWiFiAutoReconnect(true);

  if (drd.detectDoubleReset()) {
    drd.stop();
    DEBUG_PRINTLN("Double reset detected, starting config portal...");
    if (!wifiManager.startConfigPortal(HOSTNAMEOTA)) {
      DEBUG_PRINTLN("Failed to connect. Use ESP without WiFi.");
    }
  }

  rst_info *_reset_info = ESP.getResetInfoPtr();
  uint8_t _reset_reason = _reset_info->reason;
  DEBUG_PRINT("Boot-Mode: ");
  DEBUG_PRINTLN(_reset_reason);
  heartBeat = _reset_reason;

 
 /*
 REASON_DEFAULT_RST             = 0      normal startup by power on 
 REASON_WDT_RST                 = 1      hardware watch dog reset 
 REASON_EXCEPTION_RST           = 2      exception reset, GPIO status won't change 
 REASON_SOFT_WDT_RST            = 3      software watch dog reset, GPIO status won't change 
 REASON_SOFT_RESTART            = 4      software restart ,system_restart , GPIO status won't change 
 REASON_DEEP_SLEEP_AWAKE        = 5      wake up from deep-sleep 
 REASON_EXT_SYS_RST             = 6      external system reset 
  */

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  if (!wifiManager.autoConnect(AUTOCONNECTNAME, AUTOCONNECTPWD)) { 
    DEBUG_PRINTLN("Autoconnect failed connect to WiFi. Use ESP without WiFi.");
  }   

  WiFi.printDiag(Serial);
  
  sendNetInfoMQTT();

#ifdef serverHTTP
  server.on("/", handleRoot);
  server.begin();
  DEBUG_PRINTLN ("HTTP server started!!");
#endif

#ifdef time
  DEBUG_PRINTLN("Setup TIME");
  EthernetUdp.begin(localPort);
  DEBUG_PRINT("Local port: ");
  DEBUG_PRINTLN(EthernetUdp.localPort());
  DEBUG_PRINTLN("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(3600);
#endif

#ifdef ota
  ArduinoOTA.setHostname(HOSTNAMEOTA);

  ArduinoOTA.onStart([]() {
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
  timer.every(2000, isActive);
  timer.every(MEAS_DELAY, meass);
  timer.every(SEND_DELAY, sendDataMQTT);
  timer.every(SENDSTAT_DELAY, sendStatisticMQTT);
  timer.every(CONNECT_DELAY, reconnect);

  void * a;
  sendStatisticMQTT(a);
  reconnect(a);
  
  DEBUG_PRINTLN(" Ready");
 
  ticker.detach();
  //keep LED on
  digitalWrite(BUILTIN_LED, HIGH);

  drd.stop();

  DEBUG_PRINTLN(F("SETUP END......................."));
}



void loop() {
  timer.tick(); // tick the timer
#ifdef serverHTTP
  server.handleClient();
#endif
  client.loop();
  wifiManager.process();
#ifdef ota
  ArduinoOTA.handle();
#endif
}

void startConfigPortal(void) {
  DEBUG_PRINTLN("START config portal");
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.startConfigPortal(HOSTNAMEOTA);
}

void stopConfigPortal(void) {
  DEBUG_PRINTLN("STOP config portal");
  wifiManager.stopConfigPortal();
}

bool isActive(void *) {
  DEBUG_PRINTLN(wifiManager.getConfigPortalActive());
  return true;
}



bool meass(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  
  if (DS18B20Present) {
    dsSensors.requestTemperatures(); // Send the command to get temperatures
    delay(MEAS_TIME);
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
  
  if ((SI7021Present && humidity == 0) || (BMP085Present && (pressure == 0 || pressure > 106000))) {
    void * a;
    if (SI7021Present && humidity == 0) {
      heartBeat = 998;
    } else if (BMP085Present) {
      if (pressure == 0) {
        heartBeat = 997;
      } else if (pressure > 106000) {
        heartBeat = 996;
      } else {
        heartBeat = 995;
      }
    }
    sendStatisticMQTT(a);

    DEBUG_PRINT("RESTART");
    ESP.restart();
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

bool sendDataMQTT(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  DEBUG_PRINTLN(F("Data"));
  
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

bool sendStatisticMQTT(void *) {
  digitalWrite(BUILTIN_LED, LOW);
  DEBUG_PRINTLN(F("Statistic"));

  SenderClass sender;
  sender.add("VersionSW", VERSION);
  sender.add("Napeti",  ESP.getVcc());
  sender.add("HeartBeat", heartBeat++);
  sender.add("RSSI", WiFi.RSSI());
  DEBUG_PRINTLN(F("Calling MQTT"));
  
  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);
  digitalWrite(BUILTIN_LED, HIGH);
  return true;
}

void sendNetInfoMQTT() {
  digitalWrite(BUILTIN_LED, LOW);
  DEBUG_PRINTLN(F("Net info"));

  SenderClass sender;
  sender.add("IP",              WiFi.localIP().toString().c_str());
  sender.add("MAC",             WiFi.macAddress());
  sender.add("AP name",         WiFi.SSID());
  
  DEBUG_PRINTLN(F("Calling MQTT"));
  
  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);
  digitalWrite(BUILTIN_LED, HIGH);
  return;
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

#endif

bool reconnect(void *) {
  if (!client.connected()) {
    DEBUG_PRINT("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_base, mqtt_username, mqtt_key)) {
      client.subscribe((String(mqtt_base) + "/#").c_str());
      DEBUG_PRINTLN("connected");
    } else {
      DEBUG_PRINT("failed, rc=");
      DEBUG_PRINTLN(client.state());
    }
  }
  return true;
}