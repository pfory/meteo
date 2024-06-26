#include "Configuration.h"

unsigned int          srazkyPulseCount            = 0;
unsigned long         lastPulseSrazkyMillis       = 0;
unsigned int          vitrPulseCount              = 0;
//unsigned int          vitrPulseCountLast          = 0;
unsigned long         lastSend                    = 0;

float                 dewPoint;
float                 srazkyOdPulnoci             = 0.f;
float                 srazkyPosledniHodina        = 0.f;
float                 vitrPrumerPoslednich10Minut = 0.f;
float                 vitrMaxPoslednich30Minut    = 0.f;
float                 vitrSmerPoslednich30Minut   = 0.f;

int                   smerVetru = 0;

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

//ADC_MODE(ADC_VCC);

OneWire onewireA(ONE_WIRE_BUSA); // pin for onewire DALLAS bus
DallasTemperature dsSensorsA(&onewireA);
//DeviceAddress tempDeviceAddress;
float                 temperatureA        = 0.f;
bool                  DS18B20PresentA     = false;

OneWire onewireB(ONE_WIRE_BUSB); // pin for onewire DALLAS bus
DallasTemperature dsSensorsB(&onewireB);
//DeviceAddress tempDeviceAddress;
float                 temperatureB        = 0.f;
bool                  DS18B20PresentB     = false;

#ifdef BMP085
Adafruit_BMP085 bmp;
float                 high_above_sea      = 369.0;
float                 pressure            = 0.f;
float                 temperature085      = 0.f;
bool                  BMP085Present       = false;
#endif

#ifdef BMP280
Adafruit_BMP280 bmp;
float                 high_above_sea      = 369.0;
float                 pressure            = 0.f;
float                 pressureSeaLevel    = 0.f;
float                 temperature280      = 0.f;
bool                  BMP280Present       = false;
#endif

#ifdef VEML7700
Adafruit_VEML7700 veml = Adafruit_VEML7700();
float                 als;
float                 white;
float                 lux;
bool                  VEML7700Present = false;
#endif

#ifdef BH1750light
float                 lux;
BH1750                lightMeter;
#endif


void ARDUINO_ISR_ATTR vitrEvent() {
  // DEBUG_PRINTLN("Vítr puls");
  vitrPulseCount++;
}

//1 puls = 0.2794mm
void ARDUINO_ISR_ATTR srazkyEvent() {
  // DEBUG_PRINTLN("Srážkoměr puls");
   if (millis() - lastPulseSrazkyMillis > 1000) {
    lastPulseSrazkyMillis = millis();
    srazkyPulseCount++;
  }
}


#ifdef serverHTTP
#ifdef ESP32
  WebServer server(80);
#else
  ESP8266WebServer server(80);
#endif

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
            temperatureA,
            h,
#ifdef BMP280            
            pressureSeaLevel/100,
#endif
#ifdef BMP085
            pressure/100,
#endif
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
    DEBUG_PRINTLN("NET INFO");
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
  }
}


void setup() {
  preSetup();
  
#ifdef serverHTTP
// kontrola funkčnosti MDNS
  if (MDNS.begin("espwebserver")) {
    Serial.println("MDNS responder je zapnuty.");
  }
  server.on ( "/", handleRoot );
  server.begin();
  DEBUG_PRINTLN ( "HTTP server started!!" );
#endif


#ifdef VEML7700
  if (veml.begin()) {
    VEML7700Present = true;
    DEBUG_PRINTLN("VEML7700 sensor found");
    DEBUG_PRINT(F("Gain: "));
    switch (veml.getGain()) {
      case VEML7700_GAIN_1: DEBUG_PRINTLN("1"); break;
      case VEML7700_GAIN_2: DEBUG_PRINTLN("2"); break;
      case VEML7700_GAIN_1_4: DEBUG_PRINTLN("1/4"); break;
      case VEML7700_GAIN_1_8: DEBUG_PRINTLN("1/8"); break;
    }

    DEBUG_PRINT(F("Integration Time (ms): "));
    switch (veml.getIntegrationTime()) {
      case VEML7700_IT_25MS: DEBUG_PRINTLN("25"); break;
      case VEML7700_IT_50MS: DEBUG_PRINTLN("50"); break;
      case VEML7700_IT_100MS: DEBUG_PRINTLN("100"); break;
      case VEML7700_IT_200MS: DEBUG_PRINTLN("200"); break;
      case VEML7700_IT_400MS: DEBUG_PRINTLN("400"); break;
      case VEML7700_IT_800MS: DEBUG_PRINTLN("800"); break;
    }

    veml.setLowThreshold(10000);
    veml.setHighThreshold(20000);
    veml.interruptEnable(true);
  } else {
    VEML7700Present = false;
    DEBUG_PRINTLN("VEML7700 sensor not found");
  }
#endif

#ifdef BH1750light
  DEBUG_PRINT("\nBH1750:");
  Wire.begin();
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
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

#ifdef BMP085
  DEBUG_PRINT("Probe BMP085: ");
  if (bmp.begin()==1) {
    BMP085Present = true;
    DEBUG_PRINTLN("Sensor found.");
    } else {
    DEBUG_PRINTLN("Sensor missing!!!");
  }
#endif

#ifdef BMP280
  DEBUG_PRINT("Probe BMP280: ");
  if (bmp.begin()) {
    BMP280Present = true;
    DEBUG_PRINTLN("Sensor found.");
    /* Default settings from datasheet.  */ 
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
    Adafruit_BMP280::STANDBY_MS_500);  /* Standby time. */
  } else {
    DEBUG_PRINTLN("Sensor missing!!!");
  }
#endif

  DEBUG_PRINT("Probe DS18B20 on bus A: ");
  dsSensorsA.begin(); 
  if (dsSensorsA.getDeviceCount()>0) {
    DEBUG_PRINTLN("Sensor found.");
    DS18B20PresentA = true;
    dsSensorsA.setResolution(12);
    dsSensorsA.setWaitForConversion(false);
  } else {
    DEBUG_PRINTLN("Sensor missing!!!!");
  }

  DEBUG_PRINT("Probe DS18B20 on bus B: ");
  dsSensorsB.begin(); 
  if (dsSensorsB.getDeviceCount()>0) {
    DEBUG_PRINTLN("Sensor found.");
    DS18B20PresentB = true;
    dsSensorsB.setResolution(12);
    dsSensorsB.setWaitForConversion(false);
  } else {
    DEBUG_PRINTLN("Sensor missing!!!!");
  }

 
  //pinMode(vitrSmerPin, INPUT);
  analogReadResolution(13);
  //analogSetAttenuation(ADC_2_5db);
  
  pinMode(srazkyPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(srazkyPin), srazkyEvent, FALLING);
  pinMode(vitrPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(vitrPin), vitrEvent, FALLING);

#ifdef timers
  //setup timers
  //timer.every(5000, vypis);
  timer.every(SEND_DELAY,       sendDataMeteoMQTT);
  timer.every(SEND_DELAY_ANEMO, sendDataAnemoMQTT);
  timer.every(MEAS_DELAY, meass);
  timer.every(CONNECT_DELAY, reconnect);
#endif
  
  void * a=0;
  reconnect(a);
  postSetup();
}

void loop() {
  timer.tick(); // tick the timer
#ifdef ota
  ArduinoOTA.handle();
#endif
#ifdef serverHTTP
  server.handleClient();
#endif
  client.loop();
  wifiManager.process();
  drd->loop();
}

bool meass(void *) {
  //digitalWrite(LED_BUILTIN, HIGH);
 
  if (DS18B20PresentA) {
    dsSensorsA.requestTemperatures(); // Send the command to get temperatures
    delay(MEAS_TIME);
    if (dsSensorsA.getCheckForConversion()==true) {
      temperatureA = dsSensorsA.getTempCByIndex(0) + korekceTeploty;
    }
    DEBUG_PRINTLN("-------------");
    DEBUG_PRINT("Temperature DS18B20 on bus A: ");
    DEBUG_PRINT(temperatureA); 
    DEBUG_PRINTLN(" *C");
  } else {
    temperatureA = 0.0; //dummy
  }

  if (DS18B20PresentB) {
    dsSensorsB.requestTemperatures(); // Send the command to get temperatures
    delay(MEAS_TIME);
    if (dsSensorsB.getCheckForConversion()==true) {
      temperatureB = dsSensorsB.getTempCByIndex(0);
    }
    DEBUG_PRINT("Temperature DS18B20 on bus B: ");
    DEBUG_PRINT(temperatureB); 
    DEBUG_PRINTLN(" *C");
  } else {
    temperatureB = 0.0; //dummy
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

#ifdef BMP085
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
#endif

#ifdef BMP280
  if (BMP280Present) {
    DEBUG_PRINT("Temperature BMP280: ");
    temperature280 = bmp.readTemperature();
    pressure = bmp.readPressure();
    pressureSeaLevel = pressure / pow(1 - ((0.0065 *high_above_sea) / (temperature280 + (0.0065 *high_above_sea) + 273.15)), 5.257);
    //pressureSeaLevel = (pressure/pow(1-(high_above_sea/44330), 5.255));
    DEBUG_PRINT(temperature280);
    DEBUG_PRINTLN(" *C");
    DEBUG_PRINT("Pressure: ");
    DEBUG_PRINT(pressure);
    DEBUG_PRINTLN(" Pa");
  } else {
    temperature280 = 0.0;  //dummy
    pressure = 0;     //Pa - dummy
  }
#endif


// #ifdef humSI7021
  // dewPoint = calcDewPoint(humiditySI7021, temperatureA);
// #endif
// #ifdef humSHT40
  // dewPoint = calcDewPoint(humiditySHT40.relative_humidity, temperatureA);
// #endif
  
#ifdef VEML7700
  if (VEML7700Present) {
    als = veml.readALS();
    white = veml.readWhite();
    lux = veml.readLux();
    DEBUG_PRINT("VEML7700 ");
    DEBUG_PRINT("als: ");
    DEBUG_PRINT(als);
    DEBUG_PRINT(" white: ");
    DEBUG_PRINT(white);
    DEBUG_PRINT(" lux: ");
    DEBUG_PRINTLN(lux);
    

    // uint16_t irq = veml.interruptStatus();
    // if (irq & VEML7700_INTERRUPT_LOW) {
      // DEBUG_PRINTLN("** Low threshold");
    // }
    // if (irq & VEML7700_INTERRUPT_HIGH) {
      // DEBUG_PRINTLN("** High threshold");
    // }
  }
#endif

#ifdef BH1750light
  lux = lightMeter.readLightLevel();
  DEBUG_PRINT("Light: ");
  DEBUG_PRINT(lux);
  DEBUG_PRINTLN(" lx");
  if (lux < 0) {
      DEBUG_PRINTLN(F("Error condition detected"));
    } else {
      if (lux > 40000.0) {
        // reduce measurement time - needed in direct sun light
        if (lightMeter.setMTreg(32)) {
          DEBUG_PRINTLN(F("Setting MTReg to low value for high light environment"));
        } else {
          DEBUG_PRINTLN(F("Error setting MTReg to low value for high light environment"));
        }
      } else {
        if (lux > 10.0) {
          // typical light environment
          if (lightMeter.setMTreg(69)) {
            DEBUG_PRINTLN(F("Setting MTReg to default value for normal light environment"));
          } else {
            DEBUG_PRINTLN(F("Error setting MTReg to default value for normal light environment"));
          }
        } else {
          if (lux <= 10.0) {
            // very low light environment
            if (lightMeter.setMTreg(138)) {
              DEBUG_PRINTLN(
                  F("Setting MTReg to high value for low light environment"));
            } else {
              DEBUG_PRINTLN(F("Error setting MTReg to high value for low light environment"));
            }
          }
        }
      }
    }
#endif

  int smerTemp = 0;
  for (unsigned int i=0;i<100;i++) {
    smerTemp += analogRead(vitrSmerPin);
  }
  int smer = smerTemp/100;
  DEBUG_PRINT("Vítr ");
  DEBUG_PRINT("smer: ");
  DEBUG_PRINT(smer);
  DEBUG_PRINT(" - ");
  // char buff[4] = "xxx";
  // if (smer>135 && smer<165)   strcpy(buff,"VJV");
  // if (smer>171 && smer<212)   strcpy(buff,"VSV");
  // if (smer>213 && smer<230)   strcpy(buff,"V");
  // if (smer>270 && smer<330)   strcpy(buff,"JJV");
  // if (smer>414 && smer<506)   strcpy(buff,"JV");
  // if (smer>576 && smer<700)   strcpy(buff,"JJV");
  // if (smer>701 && smer<858)   strcpy(buff,"J");
  // if (smer>1125 && smer<1365) strcpy(buff,"SSV");
  // if (smer>1366 && smer<1661) strcpy(buff,"SV");
  // if (smer>2115 && smer<2499) strcpy(buff,"ZJZ");
  // if (smer>2500 && smer<2800) strcpy(buff,"JZ");
  // if (smer>2916 && smer<3564) strcpy(buff,"SSZ");
  // if (smer>3798 && smer<4500) strcpy(buff,"S");
  // if (smer>4501 && smer<5313) strcpy(buff,"ZSZ");
  // if (smer>5346 && smer<6534) strcpy(buff,"SZ");
  // if (smer>6624 && smer<8096) strcpy(buff,"Z");
  // DEBUG_PRINT(buff);
  // DEBUG_PRINT(" - ");
  smerVetru = getSmerStupne(smer);
  DEBUG_PRINTLN(smerVetru);
  //DEBUG_PRINTLN(analogReadMilliVolts(vitrSmerPin));

  return true;
}


int getSmerStupne(int s) {
  if (s>=0    && s<=170)   return 112;
  if (s>=171  && s<=209)   return 67;
  if (s>=210  && s<=250)   return 90;
  if (s>=251  && s<=380)   return 157;
  if (s>=381  && s<=550)   return 135;
  if (s>=551  && s<=700)   return 202;
  if (s>=701  && s<=950)   return 180;
  if (s>=951  && s<=1365)  return 22;
  if (s>=1366 && s<=1950)  return 45;
  if (s>=1951 && s<=2499)  return 247;
  if (s>=2500 && s<=2900)  return 225;
  if (s>=2901 && s<=3650)  return 337;
  if (s>=3651 && s<=4500)  return 0;
  if (s>=4501 && s<=5330)  return 292;
  if (s>=5331 && s<=6600)  return 315;
  if (s>=6601 && s<=8096)  return 270;
  return -s;
}


bool sendDataMeteoMQTT(void *) {
  digitalWrite(LED_BUILTIN, HIGH);
  DEBUG_PRINT(F("Send data..."));

  client.publish((String(mqtt_base) + "/Temperature").c_str(), String(temperatureA).c_str());
  client.publish((String(mqtt_base) + "/Temperature0").c_str(), String(temperatureB).c_str());
#ifdef BMP085
  client.publish((String(mqtt_base) + "/Press").c_str(), String(pressure).c_str());
  client.publish((String(mqtt_base) + "/Temp085").c_str(), String(temperature085).c_str());
#endif
#ifdef BMP280
  client.publish((String(mqtt_base) + "/Press").c_str(), String(pressureSeaLevel).c_str());
  client.publish((String(mqtt_base) + "/PressLevel").c_str(), String(pressure).c_str());
  client.publish((String(mqtt_base) + "/Temp280").c_str(), String(temperature280).c_str());
#endif
#ifdef humSI7021
  client.publish((String(mqtt_base) + "/Temp7021").c_str(), String(13.6f).c_str());
  client.publish((String(mqtt_base) + "/HumiditySI7021").c_str(), String(88).c_str());
#endif
#ifdef humSHT40
  client.publish((String(mqtt_base) + "/TempSHT40").c_str(), String(tempSHT40.temperature).c_str());
  client.publish((String(mqtt_base) + "/Humidity").c_str(), String(humiditySHT40.relative_humidity).c_str());
#endif

#ifdef VEML7700
  client.publish((String(mqtt_base) + "/als").c_str(), String(als).c_str());
  client.publish((String(mqtt_base) + "/white").c_str(), String(white).c_str());
  client.publish((String(mqtt_base) + "/lux").c_str(), String(lux).c_str());
#endif  

#ifdef BH1750light
  client.publish((String(mqtt_base) + "/lux").c_str(), String(lux).c_str());
#endif

  //DEBUG_PRINTLN(srazkyPulseCount);
  if (srazkyPulseCount>0) {
    client.publish((String(mqtt_base) + "/srazkyPulse").c_str(), String(srazkyPulseCount).c_str());
    srazkyPulseCount = 0;
  }

  digitalWrite(LED_BUILTIN, LOW);
  DEBUG_PRINTLN(F("DONE!"));
  return true;
}

bool sendDataAnemoMQTT(void *) {
  digitalWrite(LED_BUILTIN, HIGH);
  DEBUG_PRINT(F("Send data..."));

  client.publish((String(mqtt_base) + "/smerVetru").c_str(), String(smerVetru).c_str());

  //DEBUG_PRINTLN(vitrPulseCount);
  float pc = (float)vitrPulseCount/((millis() - lastSend) / 1000); //pocet pulsu za sekundu
  vitrPulseCount = 0;
  lastSend = millis();
  pc = pc * windSpeed; //1 puls 2,4km/h
  //if (abs(pc - vitrPulseCountLast) < PULSECOUNTDIF) {
    client.publish((String(mqtt_base) + "/rychlostVetru").c_str(), String(pc).c_str());
  //}

  //vitrPulseCountLast = pc;

  digitalWrite(LED_BUILTIN, LOW);
  DEBUG_PRINTLN(F("DONE!"));
  return true;
}


bool reconnect(void *) {
  if (!client.connected()) {
    DEBUG_PRINT("Attempting MQTT connection...");
    if (client.connect((String(mqtt_base)).c_str(), mqtt_username, mqtt_key, (String(mqtt_base) + "/LWT").c_str(), 2, true, "offline", true)) {
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_restart)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_topic_netinfo)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_config_portal_stop)).c_str());
      client.subscribe((String(mqtt_base) + "/" + String(mqtt_config_portal)).c_str());
      client.subscribe((String(mqtt_base) + "/SrazkyOdPulnoci").c_str());
      client.subscribe((String(mqtt_base) + "/SrazkyPosledniHodina").c_str());
      client.subscribe((String(mqtt_base) + "/VitrPrumerPoslednich10Minut").c_str());
      client.subscribe((String(mqtt_base) + "/VitrMaxPoslednich30Minut").c_str());
      client.subscribe((String(mqtt_base) + "/VitrSmerPoslednich30Minut").c_str());
     client.publish((String(mqtt_base) + "/LWT").c_str(), "online", true);
      client.publish((String(mqtt_base) + "/connect").c_str(), "1", true);
      DEBUG_PRINTLN("connected");
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