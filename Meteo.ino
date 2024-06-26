//BMP085   - pressure sensor
//DS18B20  - temperature sensor
//SI7021   - temperature and humidity sensor do v2.32
//SHT40    - temperature and humidity sensor do v2.40

//Pinout NODEMCU 1.0
//D4 - DS18B20
//D5 - SCL
//D6 - SDA
//3.3
//GND

#include "Configuration.h"

float                 dewPoint;
float                 srazkyOdPulnoci = 0.f;
float                 srazkyPosledniHodina = 0.f;
float                 vitrPrumerPoslednich10Minut = 0.f;
float                 vitrMaxPoslednich30Minut = 0.f;
float                 vitrSmerPoslednich30Minut = 0.f;


#ifdef humSI7021
SI7021 sensorHumiditySI7021;

float                 tempSI7021;
float                 humiditySI7021;
bool                  SI7021Present        = false;
#endif

#ifdef humSHT40
Adafruit_SHT4x sensorHumiditySHT40 = Adafruit_SHT4x();
sensors_event_t       tempSHT40;
sensors_event_t       humiditySHT40;
bool                  SHT40Present        = false;
#endif

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

#ifdef serverHTTP
void handleRoot() {
	char temp[200];
  DEBUG_PRINT("Web client request...");
//  digitalWrite(LED_BUILTIN, LOW);
  int h;
#ifdef humSI7021      
  h = (int)humiditySI7021;
#endif
#ifdef humSHT40
  h = (int)humiditySHT40.relative_humidity;
#endif
  
  // temperature = -4.625f;
  // humidity=96.5f;
  // pressure=102378f;
  // dewPoint=1.5f;
  // srazkyOdPulnoci=1.4f

  
  //|11.09.11|21:17|22.9|69|1010.2|1.3|207|0.0|2.1|0.0|, 
  //|datum|čas|teplota|vlhkost|tlak (přepočtený na hladinu moře, relativní)|rychlost větru v m/s (maximální náraz za 30 minut)
  //|směr ve stupních|dnešní srážky (od půlnoci)|průměrná rychlost větru za 10 minut v m/s|aktuální intenzita deště v mm/h).
  
  snprintf(temp, 200, "<html><head><meta charset='UTF-8'></head><body>\
            |%2d.%02d.%02d|%02d:%02d|%2.1f|%d|%4.1f|%2.1f|%3.0f|%3.1f|%2.1f|%3.1f|</body></html>",
            day(), month(), year(), hour(), minute(),
            temperature,
            h,
            pressure/100,
            vitrMaxPoslednich30Minut,
            vitrSmerPoslednich30Minut,
            srazkyOdPulnoci,
            vitrPrumerPoslednich10Minut,
            srazkyPosledniHodina
  );


	server.send ( 200, "text/html", temp );
  //client.publish((String(mqtt_base) + "/DataInPocasi").c_str(), temp);

  DEBUG_PRINTLN("done.");
  //digitalWrite(LED_BUILTIN, HIGH);
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
  } else if (strcmp(topic, (String(mqtt_base) + "/SrazkyOdPulnoci").c_str())==0) {
    DEBUG_PRINT("Srazky");
    DEBUG_PRINTLN(val.toFloat());
    srazkyOdPulnoci = val.toFloat();
  } else if (strcmp(topic, (String(mqtt_base) + "/SrazkyPosledniHodina").c_str())==0) {
    DEBUG_PRINT("Srazky");
    DEBUG_PRINTLN(val.toFloat());
    srazkyPosledniHodina = val.toFloat();
  } else if (strcmp(topic, (String(mqtt_base) + "/VitrPrumerPoslednich10Minut").c_str())==0) {
    DEBUG_PRINT("Vitr");
    DEBUG_PRINTLN(val.toFloat());
    vitrPrumerPoslednich10Minut = val.toFloat();
  } else if (strcmp(topic, (String(mqtt_base) + "/VitrMaxPoslednich30Minut").c_str())==0) {
    DEBUG_PRINT("Vitr");
    DEBUG_PRINTLN(val.toFloat());
    vitrMaxPoslednich30Minut = val.toFloat();
  } else if (strcmp(topic, (String(mqtt_base) + "/VitrSmerPoslednich30Minut").c_str())==0) {
    DEBUG_PRINT("Vitr");
    DEBUG_PRINTLN(val.toFloat());
    vitrSmerPoslednich30Minut = val.toFloat();
  } else if (strcmp(topic, (String(mqtt_base) + "/Press").c_str())==0) {
    pressure = val.toFloat();
  }
}


void setup() {
  preSetup();
  Wire.begin(SDAPIN, SCLPIN);
  
#ifdef serverHTTP
  server.on ( "/", handleRoot );
  server.begin();
  DEBUG_PRINTLN ( "HTTP server started!!" );
#endif

#ifdef humSI7021
  DEBUG_PRINT("\nProbe SI7021: ");
  if (sensorHumiditySI7021.begin(SDAPIN, SCLPIN)) {
    SI7021Present = true;
  }

  if (SI7021Present == true) {
    DEBUG_PRINTLN("Sensor found.");
  } else {
    DEBUG_PRINTLN("Sensor missing!!!!");
  }
#endif

#ifdef humSHT40
  if (!sensorHumiditySHT40.begin()) {
    DEBUG_PRINTLN("Couldn't find SHT4x");
    SHT40Present = false;
  } else {
    DEBUG_PRINTLN("Found SHT4x sensor");
    DEBUG_PRINT("Serial number 0x");
    DEBUG_PRINTHEX(sensorHumiditySHT40.readSerial());
    DEBUG_PRINTLN();
    sensorHumiditySHT40.setPrecision(SHT4X_HIGH_PRECISION);
    sensorHumiditySHT40.setHeater(SHT4X_NO_HEATER);
    SHT40Present = true;
  }
#endif

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

  void * a = 0;
  reconnect(a);
  meass(a);
  sendDataMQTT(a);
  sendNetInfoMQTT();  

  ticker.detach();
  digitalWrite(LED_BUILTIN, HIGH);

  drd->stop();

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
  drd->loop();
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

#ifdef humSI7021  
  if (SI7021Present) {
    humiditySI7021=sensorHumiditySI7021.getHumidityPercent();
    tempSI7021=sensorHumiditySI7021.getCelsiusHundredths() / 100;
    //sensorHumiditySI7021.triggerMeasurement();

    if (humiditySI7021>100) {
      humiditySI7021 = 100;
    }
    DEBUG_PRINT("Temperature SI7021: ");
    DEBUG_PRINT(tempSI7021);
    DEBUG_PRINTLN(" *C");
    DEBUG_PRINT("Humidity SI7021: ");
    DEBUG_PRINT(humiditySI7021);
    DEBUG_PRINTLN(" %Rh");
  } else {
    humiditySI7021 = 0.0;    //dummy
    tempSI7021 = 0.0;  //dummy
  }
#endif  

#ifdef humSHT40
  if (SHT40Present) {
    sensorHumiditySHT40.getEvent(&humiditySHT40, &tempSHT40);// populate temp and humidity objects with fresh data
    DEBUG_PRINT("Temperature SHT40:");
    DEBUG_PRINT(tempSHT40.temperature);
    DEBUG_PRINTLN(" *C");
    DEBUG_PRINT("Humidity SHT40:");
    DEBUG_PRINT(humiditySHT40.relative_humidity);
    DEBUG_PRINTLN(" %Rh");
  } else {
    humiditySHT40.relative_humidity = 0.0;    //dummy
    tempSHT40.temperature = 0.0;   //dummy
  }
#endif

  // if (BMP085Present) {
    // DEBUG_PRINT("Temperature BMP085: ");
    // temperature085 = bmp.readTemperature();
    // pressure = bmp.readSealevelPressure(high_above_sea);
    // DEBUG_PRINT(temperature085);
    // DEBUG_PRINTLN(" *C");
    // DEBUG_PRINT("Pressure: ");
    // DEBUG_PRINT(pressure);
    // DEBUG_PRINTLN(" Pa");
  // } else {
    // temperature085 = 0.0;  //dummy
    // pressure = 0;     //Pa - dummy
  // }
  
  //if ((SI7021Present && humidity == 0) || (BMP085Present && (pressure == 0 || pressure > 106000))) {
    // void * a;
    // if (SI7021Present && humidity == 0) {
      // heartBeat = 998;
    // } else if (BMP085Present) {
      // if (pressure == 0) {
        // heartBeat = 997;
      // } else if (pressure > 106000) {
        // heartBeat = 996;
      // } else {
        // heartBeat = 995;
      // }
    // }
    // sendStatisticMQTT(a);

    // DEBUG_PRINT("RESTART");
    //ESP.restart();
  //}
// #ifdef humSI7021
  // dewPoint = calcDewPoint(humiditySI7021, temperature);
// #endif
// #ifdef humSHT40
  // dewPoint = calcDewPoint(humiditySHT40.relative_humidity, tempSHT40.temperature);
// #endif
  digitalWrite(LED_BUILTIN, HIGH);

  return true;
}

// float calcDewPoint (float humidity, float temperature)  
// {  
    // float logEx;  
    // logEx = 0.66077 + (7.5 * temperature) / (237.3 + temperature)  
            // + (log10(humidity) - 2);  
    // return (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);  
// }

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
  //client.publish((String(mqtt_base) + "/Press").c_str(), String(pressure).c_str());
  //client.publish((String(mqtt_base) + "/Temp085").c_str(), String(temperature085).c_str());
// #ifdef humSI7021
  // client.publish((String(mqtt_base) + "/Temp7021").c_str(), String(tempSI7021).c_str());
  // client.publish((String(mqtt_base) + "/HumiditySI7021").c_str(), String(humiditySI7021).c_str());
// #endif
 #ifdef humSHT40
   client.publish((String(mqtt_base) + "/TempSHT40").c_str(), String(tempSHT40.temperature).c_str());
   client.publish((String(mqtt_base) + "/Humidity").c_str(), String(humiditySHT40.relative_humidity).c_str());
 #endif

  //client.publish((String(mqtt_base) + "/DewPoint").c_str(), String(dewPoint).c_str());

  digitalWrite(LED_BUILTIN, HIGH);
  DEBUG_PRINTLN(F("DONE!"));
  return true;
}

bool reconnect(void *) {
  if (!client.connected()) {
    DEBUG_PRINTLN("Attempting MQTT connection...");
    if (client.connect(mqtt_base, mqtt_username, mqtt_key, (String(mqtt_base) + "/LWT").c_str(), 2, true, "offline", true)) {
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_restart)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_netinfo)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_config_portal)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_config_portal_stop)).c_str());
      client.subscribe((String(mqtt_base) + "/SrazkyOdPulnoci").c_str());
      client.subscribe((String(mqtt_base) + "/SrazkyPosledniHodina").c_str());
      client.subscribe((String(mqtt_base) + "/VitrPrumerPoslednich10Minut").c_str());
      client.subscribe((String(mqtt_base) + "/VitrMaxPoslednich30Minut").c_str());
      client.subscribe((String(mqtt_base) + "/VitrSmerPoslednich30Minut").c_str());
      client.subscribe((String(mqtt_base) + "/Press").c_str());
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