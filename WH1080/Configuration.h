#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//SW name & version
#define     VERSION                          "0.10"
#define     SW_NAME                          "MeteoNew"


#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <DallasTemperature.h>
#include <OneWireNg.h>

//sensor tlaku
//#define BMP085
#ifdef BMP085
#include <Adafruit_BMP085.h> 
#endif

//sensor tlaku
#define BMP280
#ifdef BMP280
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#endif


//sensor vlhkosti
//#define humSI7021
#ifdef humSI7021
#include <SI7021.h>
#endif

//sensor vlhkosti
#define humSHT40
#ifdef humSHT40
#include "Adafruit_SHT4x.h"
#endif


//sensor osvetleni
//#define VEML7700
#ifdef VEML7700
#include "Adafruit_VEML7700.h"
#endif

//sensor osvetleni
#define BH1750light
#ifdef BH1750light
#include <BH1750.h>
#endif

#define ota
#define verbose
#define cas
#define timers

//#define serverHTTP
#define serverHTTP32
#if defined(serverHTTP)
#include <ESP8266WebServer.h>
#endif


static const char* const      mqtt_server                    = "192.168.1.56";
static const uint16_t         mqtt_port                      = 1883;
static const char* const      mqtt_username                  = "datel";
static const char* const      mqtt_key                       = "hanka12";
static const char* const      mqtt_base                      = "/home/MeteoTest";
static const char* const      mqtt_base_old                  = "/home/Meteo";
static const char* const      mqtt_topic_restart             = "restart";
static const char* const      mqtt_topic_netinfo             = "netinfo";
static const char* const      mqtt_config_portal             = "config";
static const char* const      mqtt_config_portal_stop        = "disconfig";


#define ONE_WIRE_BUSA                       16 //Dallas
#define ONE_WIRE_BUSB                       18 //Dallas
#define SDAPIN                              33
#define SCLPIN                              35


const byte srazkyPin                        = 12;
const byte vitrPin                          = 11;
const byte vitrSmerPin                      = 3;

float korekceTeploty                        = 0.0;  //korekce teplotniho cidla proti minulemu

//The cup-type anemometer measures wind speed by closing a contact as a magnet moves past a
//switch. A wind speed of 2.4km/h causes the switch to close once per second.
const float windSpeed                       = 2.4;

#define SEND_DELAY                           15000  //prodleva mezi poslanim dat v ms
#define SEND_DELAY_ANEMO                     5000  //prodleva mezi poslanim dat v ms
#define SENDSTAT_DELAY                       60000  //poslani statistiky kazdou minutu
#define CONNECT_DELAY                        5000 //ms
#define MEAS_DELAY                           5000   //mereni


#define MEAS_TIME                            750 //in ms

#include <fce.h>

#endif
