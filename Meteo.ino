#include <Wire.h>
#include "i2c.h"

#define watchdog //enable this only on board with UNO bootloader
#ifdef watchdog
#include <avr/wdt.h>
#endif


#include "i2c_SI7021.h"
SI7021 si7021;
float                 humidity, tempSI7021;
bool                  SI7021Present        = false;

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

#include <SPI.h>
#include <Ethernet.h>
byte mac[] = { 0x00, 0xE0, 0x07D, 0xCE, 0xC6, 0x6F };
EthernetClient client;
IPAddress ip(192,168,1,102);

//XIVELY
#define xively
#ifdef xively
char server[] = "api.cosm.com";   // name address for cosm API
#include <Xively.h>
#include <HttpClient.h>


char xivelyKey[] 			= "q1PY6QqB9jvSHGKhmCQNBRdCofeSAKxpKzliaHJGWUc5UT0g";

#define xivelyFeed 				63310

char VersionID[]	 	    = "V";
char StatusID[]	 	      = "H";
char TempID[]	 	        = "T2899BDCF02000076";
char HumidityID[]	 	    = "Humidity";
char PressID[]	 	      = "Press";
char Temp085ID[]	 	    = "Temp085";
char TempSI7021ID[]	 	  = "TempSI7021";

XivelyDatastream datastreams[] = {
	XivelyDatastream(VersionID, 		    strlen(VersionID), 	      DATASTREAM_FLOAT),
	XivelyDatastream(StatusID, 		      strlen(StatusID), 		    DATASTREAM_INT),
	XivelyDatastream(TempID,  		      strlen(TempID), 		      DATASTREAM_FLOAT),
	XivelyDatastream(HumidityID, 		    strlen(HumidityID), 		  DATASTREAM_FLOAT),
	XivelyDatastream(PressID, 		      strlen(PressID), 		      DATASTREAM_FLOAT),
	XivelyDatastream(Temp085ID, 		    strlen(Temp085ID), 		    DATASTREAM_FLOAT),
	XivelyDatastream(TempSI7021ID, 		  strlen(TempSI7021ID), 		DATASTREAM_FLOAT)
};

XivelyFeed feed(xivelyFeed, 			datastreams, 			7);
XivelyClient xivelyclient(client);
#endif

#define openhab
#ifdef openhab
#include <PubSubClient.h>
//IPAddress serverPubSub(88, 146, 202, 186);
IPAddress serverPubSub(192, 168, 1, 56);
// Callback function header
void callback(char* topic, byte* payload, unsigned int length);

EthernetClient ethClient;
PubSubClient clientPubSub(serverPubSub, 1883, callback, ethClient);
char charBuf[15];

// Callback function
void callback(char* topic, byte* payload, unsigned int length) {
}
#endif

byte status=0;
float versionSW=1.3;
char versionSWString[] = "METEO Simple v"; //SW name & version

#define verbose

void setup() {
#ifdef verbose
  Serial.begin(115200);
#endif
  Ethernet.begin(mac, ip);
  Serial.println(Ethernet.localIP());
#ifdef verbose
  Serial.print("Mask:");
  Serial.println(Ethernet.subnetMask());
  Serial.print("Gateway:");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("DNS:");
  Serial.println(Ethernet.dnsServerIP());
  Serial.println();
#endif
  Wire.begin();
#ifdef verbose
  Serial.print("Probe SI7021: ");
#endif
  if (si7021.initialize()) {
#ifdef verbose
    Serial.println("Sensor found.");
#endif
    SI7021Present = true;
  } else {
#ifdef verbose
    Serial.println("Sensor missing!!!!");
#endif
  }
  
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
}

void loop() {
#ifdef watchdog
	wdt_reset();
#endif  
  if (millis() - lastMeas >= measDelay) {
    lastMeas = millis();
    
    if (DS18B20Present) {
      dsSensors.requestTemperatures(); // Send the command to get temperatures
      delay(measTime);
      if (dsSensors.getCheckForConversion()==true) {
        temperature = dsSensors.getTempCByIndex(0);
      }
#ifdef verbose
      Serial.println("-------------");
      Serial.print("Temperature DS18B20: ");
      Serial.print(temperature); 
      Serial.println(" *C");
#endif
    }
    
    if (SI7021Present) {
      si7021.getHumidity(humidity);
      si7021.getTemperature(tempSI7021);
      si7021.triggerMeasurement();

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
    }
    
    
#ifdef xively
    sendData();
#endif
#ifdef openhab
    sendDataOpenHAB();
#endif   
  }
}

#ifdef xively
void sendData() {
  datastreams[0].setFloat(versionSW);  
  datastreams[1].setInt(status);  
  if (DS18B20Present) {
    datastreams[2].setFloat(temperature);         //temperature DS18B20
  }
  if (BMP085Present) {
    datastreams[4].setFloat(pressure);            //pressure BMP085
    datastreams[5].setFloat(temperature085);      //temperature BMP085
  }
  if (SI7021Present) {
    datastreams[6].setFloat(tempSI7021);          //temperature SI7021
    datastreams[3].setFloat(humidity);            //humidity SI7021
  }
  
//#ifdef verbose
  Serial.println("Uploading data to Xively");
//#endif
  #ifdef watchdog
	wdt_disable();
#endif

  int ret = xivelyclient.put(feed, xivelyKey);
  
  if (ret==200) {
    if (status==0) status=1; else status=0;
#ifdef verbose
    Serial.print("Xively OK:");
#endif
  } else {
#ifdef verbose
    Serial.print("Xively err: ");
#endif
  }
#ifdef verbose
  Serial.println(ret);
#endif
  
#ifdef watchdog
	wdt_enable(WDTO_8S);
#endif
}
#endif

#ifdef openhab
void sendDataOpenHAB() {
#ifdef verbose
  Serial.print("Sending data to OpenHAB");
#endif
  if (clientPubSub.connect("Meteostanice", "datel", "hanka12")) {
    if (DS18B20Present) {
      clientPubSub.publish("/home/Meteo/esp02/Temperature",floatToString(temperature));
    }
    if (BMP085Present) {
      clientPubSub.publish("/home/Meteo/esp02/Press",floatToString(pressure));
      clientPubSub.publish("/home/Meteo/esp02/Temp085",floatToString(temperature085));
    }
    if (SI7021Present) {
      clientPubSub.publish("/home/Meteo/esp02/Humidity",floatToString(humidity));
      clientPubSub.publish("/home/Meteo/esp02/TempDHT",floatToString(tempSI7021));
    }
#ifdef verbose
    Serial.println("...OK");
#endif
    //    if (heartBeat==0) heartBeat=1;
//    else heartBeat=0;
  }
}


char* floatToString(float f) {
  dtostrf(f, 1, 2, charBuf);
  return charBuf;
}
#endif