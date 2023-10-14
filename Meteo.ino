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

float                 humidity, tempSI7021, dewPoint;
bool                  SI7021Present        = false;

ADC_MODE(ADC_VCC);

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

// #ifdef serverHTTP
// ESP8266WebServer server(81);
// #endif

#ifdef serverHTTP
void handleRoot() {
	char temp[600];
  DEBUG_PRINT("Web client request...");
  digitalWrite(LED_BUILTIN, LOW);
  
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
  digitalWrite(LED_BUILTIN, HIGH);
}
#endif

//MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
  // char * pEnd;
  String val =  String();
  DEBUG_PRINT("\nMessage arrived [");
  DEBUG_PRINT(topic);
  DEBUG_PRINT("] ");
  for (unsigned int i=0;i<length;i++) {
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


void setup() {
  preSetup();
  client.setCallback(callback);
  
#ifdef serverHTTP
  server.on ( "/", handleRoot );
  server.begin();
  DEBUG_PRINTLN ( "HTTP server started!!" );
#endif

  DEBUG_PRINT("\nProbe SI7021: ");
  if (si7021.begin(SDAPIN, SCLPIN)) {
    SI7021Present = true;
  }

  if (SI7021Present == true) {
    DEBUG_PRINTLN("Sensor found.");
  } else {
    DEBUG_PRINTLN("Sensor missing!!!!");
  }

  Wire.begin();
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

#ifdef timers
  //setup timers
  timer.every(MEAS_DELAY, meass);
  timer.every(SEND_DELAY, sendDataMQTT);
  timer.every(CONNECT_DELAY, reconnect);
#endif

  void * a;
  reconnect(a);
  meass(a);
  sendDataMQTT(a);

  ticker.detach();
  //keep LED on
  digitalWrite(LED_BUILTIN, HIGH);

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

bool meass(void *) {
  digitalWrite(LED_BUILTIN, LOW);
  
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
  
  digitalWrite(LED_BUILTIN, HIGH);

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
  digitalWrite(LED_BUILTIN, LOW);
  DEBUG_PRINT(F("Send data..."));

  client.publish((String(mqtt_base) + "/Temperature").c_str(), String(temperature).c_str());
  client.publish((String(mqtt_base) + "/Press").c_str(), String(pressure).c_str());
  client.publish((String(mqtt_base) + "/Temp085").c_str(), String(temperature085).c_str());
  client.publish((String(mqtt_base) + "/Humidity").c_str(), String(humidity).c_str());
  client.publish((String(mqtt_base) + "/Temp7021").c_str(), String(tempSI7021).c_str());
  client.publish((String(mqtt_base) + "/DewPoint").c_str(), String(dewPoint).c_str());

  digitalWrite(LED_BUILTIN, HIGH);
  DEBUG_PRINTLN(F("DONE!"));
  return true;
}

bool reconnect(void *) {
  if (!client.connected()) {
    DEBUG_PRINT("Attempting MQTT connection...");
    if (client.connect(mqtt_base, mqtt_username, mqtt_key, (String(mqtt_base) + "/LWT").c_str(), 2, true, "offline", true)) {
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_restart)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_netinfo)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_config_portal)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_config_portal_stop)).c_str());
      client.publish((String(mqtt_base) + "/LWT").c_str(), "online", true);
    } else {
      DEBUG_PRINT("disconected.");
      DEBUG_PRINT(" Wifi status:");
      DEBUG_PRINT(WiFi.status());
      DEBUG_PRINT(" Client status:");
      DEBUG_PRINTLN(client.state());
    }
  }
  return true;
}