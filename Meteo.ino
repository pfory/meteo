#include <Wire.h>
//#include "i2c.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoOTA.h>

#define verbose

#define WIFI
#ifdef WIFI
  #include <WiFiClient.h>
  #include <ESP8266WiFi.h>
  #include <WiFiManager.h> 
  #include <TimeLib.h>
  #include <WiFiUdp.h>
  WiFiClient client;
  #define SDAPIN 12 // D6 - GPI12 on ESP-201 module
  #define SCLPIN 14 // D5 - GPI14 on ESP-201 module
  IPAddress _ip = IPAddress(192, 168, 1, 103);
  IPAddress _gw = IPAddress(192, 168, 1, 1);
  IPAddress _sn = IPAddress(255, 255, 255, 0);
  ESP8266WebServer server(80);
  #define TEMPERATURE_DIVIDOR 100
  static const char ntpServerName[] = "tik.cesnet.cz";
  const int timeZone = 1;     // Central European Time
  WiFiUDP Udp;
  unsigned int localPort = 8888;  // local port to listen for UDP packets
  time_t getNtpTime();
  WiFiManager wifiManager;
#else
 #include <SPI.h>
 #include <Ethernet.h>
  EthernetClient client;
  byte mac[] = { 0x00, 0xE0, 0x07D, 0xCE, 0xC6, 0x6F };
  IPAddress ip(192,168,1,103);
  EthernetServer server(80);
  #define watchdog //enable this only on board with UNO bootloader
  #ifdef watchdog
  #include <avr/wdt.h>
  #endif
#endif 

#define AIO_SERVER      "192.168.1.56"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "datel"
#define AIO_KEY         "hanka12"

#include "SI7021.h"
SI7021 si7021;
float                 humidity, tempSI7021, dewPoint;
bool                  SI7021Present        = false;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/
Adafruit_MQTT_Publish _temperature             = Adafruit_MQTT_Publish(&mqtt, "/home/Meteo/Temperature");
Adafruit_MQTT_Publish _pressure                = Adafruit_MQTT_Publish(&mqtt, "/home/Meteo/Press");
Adafruit_MQTT_Publish _temperature085          = Adafruit_MQTT_Publish(&mqtt, "/home/Meteo/Temp085");
Adafruit_MQTT_Publish _humidity                = Adafruit_MQTT_Publish(&mqtt, "/home/Meteo/Humidity");
Adafruit_MQTT_Publish _temperatureDHT          = Adafruit_MQTT_Publish(&mqtt, "/home/Meteo/TempDHT");
  

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS A0
OneWire onewire(ONE_WIRE_BUS); // pin for onewire DALLAS bus
DallasTemperature dsSensors(&onewire);
DeviceAddress tempDeviceAddress;
#define NUMBER_OF_DEVICES 1
const unsigned long   measTime            = 750; //in ms
float                 temperature         = 0.f;
const unsigned long   measDelay           = 60000; //in ms
unsigned long         lastMeas            = measDelay;
bool                  DS18B20Present      = false;

#include <Adafruit_BMP085.h> 
Adafruit_BMP085 bmp;
float                 high_above_sea      =369.0;
float                 pressure            = 0;
float                 temperature085      = 0;
bool                  BMP085Present       = false;


byte status=0;
float versionSW=1.4;
char versionSWString[] = "METEO Simple v"; //SW name & version

#define verbose
/*
DewPoint,2016-07-20T05:30:39.196504Z,13.6
Press,2018-01-03T18:56:46.618138Z,99717.00
T2899BDCF02000076,2018-01-03T18:56:46.618138Z,5.19
WindDir,2016-06-21T14:53:33.227881Z,199
WindSpeed,2016-07-20T05:30:26.842469Z,2.2
*/
void handleRoot() {
	char temp[600];
  // Serial.print(year());
  // Serial.print(month());
  // Serial.print(day());
  // Serial.print(hour());
  // Serial.print(minute());
  // Serial.print(second());


	snprintf ( temp, 200,
      "<html>\
        <head>\
          <meta charset='UTF-8'>\
        </head>\
        <body>\
          T2899BDCF02000076,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%3d.%02d<br />\
          Humidity,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%3d<br />\
          Press,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%6d<br />\
          DewPoint,%4d-%02d-%02dT%02d:%02d:%02d.000000Z,%3d.%02d<br />\
        </body>\
      </html>",
      year(), month(), day(), hour(), minute(), second(),
      (int)(temperature/TEMPERATURE_DIVIDOR), (abs((int)temperature))%TEMPERATURE_DIVIDOR,
      year(), month(), day(), hour(), minute(), second(),
      (int)humidity,
      year(), month(), day(), hour(), minute(), second(),
      (int)pressure,
      year(), month(), day(), hour(), minute(), second(),
      (int)dewPoint/TEMPERATURE_DIVIDOR, (abs((int)dewPoint))%TEMPERATURE_DIVIDOR
	);
	server.send ( 200, "text/html", temp );
}


void setup() {
#ifdef verbose
  Serial.begin(115200);
#endif
#ifdef WIFI
  //WiFi.config(ip); 
  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  //WiFi.begin(ssid, password);
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect, we should reset as see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
#ifdef verbose    
		Serial.print(".");
#endif
	}
#ifdef verbose
	Serial.println("");
	// Serial.print("Connected to ");
	// Serial.println(ssid);
	// Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
#endif
#else
  Ethernet.begin(mac, ip);
  Serial.println(Ethernet.localIP());
  server.begin();
#ifdef verbose
  Serial.print("Mask:");
  Serial.println(Ethernet.subnetMask());
  Serial.print("Gateway:");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("DNS:");
  Serial.println(Ethernet.dnsServerIP());
  Serial.println();
#endif
#endif
  Wire.begin();
  
#ifdef verbose
  Serial.print("Probe SI7021: ");
#endif
#ifdef WIFI  
  if (si7021.begin(SDAPIN, SCLPIN)) {
    SI7021Present = true;
  }
#else
  if (si7021.begin()) {
    SI7021Present = true;
  }
#endif

#ifdef verbose
  if (SI7021Present == true) {
    Serial.println("Sensor found.");
  } else {
    Serial.println("Sensor missing!!!!");
  }
#endif

#ifdef verbose
  Serial.print("Probe DS18B20: ");
#endif
  dsSensors.begin(); 
  if (dsSensors.getDeviceCount()>0) {
#ifdef verbose
    Serial.println("Sensor found.");
#endif
    DS18B20Present = true;
    dsSensors.setResolution(12);
    dsSensors.setWaitForConversion(false);
  } else {
#ifdef verbose
    Serial.println("Sensor missing!!!!");
#endif
  }

#ifdef verbose
  Serial.print("Probe BMP085: ");
#endif
  if (bmp.begin()==1) {
    BMP085Present = true;
#ifdef verbose
    Serial.println("Sensor found.");
#endif
    } else {
#ifdef verbose
    Serial.println("Sensor missing!!!");
#endif
  }
  
  server.on ( "/", handleRoot );
  server.begin();
  Serial.println ( "HTTP server started" );
  
  Udp.begin(localPort);
#ifdef verbose  
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
#endif
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  
  // Serial.print("Time Status after setSyncProvider:");
  // Serial.println(timeStatus());

  if (timeStatus()== timeNotSet) {
    ESP.reset();
  }

  
  //while(timeStatus()== timeNotSet)
     ; // wait until the time is set by the sync provider
  digitalClockDisplay();
  
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
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  
}

void loop() {
#ifdef watchdog
	wdt_reset();
#endif  
#ifdef WIFI
  server.handleClient();
#endif
  if (millis() - lastMeas >= measDelay) {
    lastMeas = millis();
    
    if (DS18B20Present) {
      dsSensors.requestTemperatures(); // Send the command to get temperatures
      delay(measTime);
      if (dsSensors.getCheckForConversion()==true) {
        temperature = dsSensors.getTempCByIndex(0) * TEMPERATURE_DIVIDOR;
      }
#ifdef verbose
      Serial.println("-------------");
      Serial.print("Temperature DS18B20: ");
      Serial.print(temperature); 
      Serial.println(" *C");
#endif
    } else {
      temperature = -22.2 * TEMPERATURE_DIVIDOR; //dummy
    }
    
    if (SI7021Present) {
      humidity=si7021.getHumidityPercent();
      tempSI7021=si7021.getCelsiusHundredths() / 100;
      //si7021.triggerMeasurement();

      if (humidity>100) {
        humidity = 100;
      }
#ifdef verbose
      Serial.print("Temperature SI7021: ");
      Serial.print(tempSI7021);
      Serial.println(" *C");
      Serial.print("Humidity SI7021: ");
      Serial.print(humidity);
      Serial.println(" %Rh");
#endif
    } else {
      humidity = 55.5;    //dummy
      tempSI7021 = 33.3;  //dummy
    }
    
    if (BMP085Present) {
#ifdef verbose
      Serial.print("Temperature BMP085: ");
#endif
      temperature085 = bmp.readTemperature();
      pressure = bmp.readSealevelPressure(high_above_sea);
#ifdef verbose
      Serial.print(temperature085);
      Serial.println(" *C");
      Serial.print("Pressure: ");
      Serial.print(pressure);
      Serial.println(" Pa");
#endif    
    } else {
      temperature085 = 77.7;  //dummy
      pressure = 123456;     //Pa - dummy
    }
    
    dewPoint = calcDewPoint(humidity, temperature);
    
    sendDataHA();

#ifdef WIFI
#else    
    EthernetClient client = server.available();
    if (client) {
      generateHTML();
    }
#endif
  }
  ArduinoOTA.handle();
}

void sendDataHA() {
  Serial.println("I am sending data from Meteo unit to HomeAssistant");
  
  MQTT_connect();
  if (! _temperature.publish(temperature/TEMPERATURE_DIVIDOR)) {
    Serial.println("Temperature failed");
  } else {
    Serial.println("Temperature OK!");
  }  
  if (! _pressure.publish(pressure)) {
    Serial.println("Pressure failed");
  } else {
    Serial.println("Pressure OK!");
  }  
  if (! _temperature085.publish(temperature085)) {
    Serial.println("Temperature085 failed");
  } else {
    Serial.println("Temperature085 OK!");
  }  
  if (! _humidity.publish(humidity)) {
    Serial.println("Humidity failed");
  } else {
    Serial.println("Humidity OK!");
  }  
  // if (! _temperatureDHT.publish(temperatureDHT)) {
    // Serial.println("failed");
  // } else {
    // Serial.println("OK!");
  // }  
}

 
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

uint8_t retries = 3;
while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
     Serial.println(mqtt.connectErrorString(ret));
     Serial.println("Retrying MQTT connection in 5 seconds...");
     mqtt.disconnect();
     delay(5000);  // wait 5 seconds
     retries--;
     if (retries == 0) {
       // basically die and wait for WDT to reset me
       while (1);
     }
}
  Serial.println("MQTT Connected!");
}

void generateHTML() {
#ifdef verbose
  Serial.println("new client");
#endif  
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
#ifdef verbose
  Serial.println("client disconnected");
#endif
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  //IPAddress ntpServerIP; // NTP server's ip address
  IPAddress ntpServerIP = IPAddress(195, 113, 144, 201);

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  //WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
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
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void digitalClockDisplay(){
#ifdef verbose
  // digital clock display of the time
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  Serial.print(" ");
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println(" ");
#endif
}

#ifdef verbose
void printDigits(int digits){
  // utility function for digital clock display: prints preceding
  // colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
#endif

float calcDewPoint (int humidity, int temperature)  
{  
    float logEx;  
    logEx = 0.66077 + (7.5 * temperature) / (237.3 + temperature)  
            + (log10(humidity) - 2);  
    return (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);  
}