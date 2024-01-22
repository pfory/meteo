#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define humSHT40
#define humSI7021

#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <DallasTemperature.h>
#include <Adafruit_BMP085.h> 
#include <OneWire.h>
#include <ESP8266WebServer.h>

#ifdef humSI7021
#include <SI7021.h>
#endif

#ifdef humSHT40
#include "Adafruit_SHT4x.h"
#endif

//SW name & version
#define     VERSION                          "2.44"
#define     SW_NAME                          "Meteo"

#define ota
#define verbose
#define cas
#define timers
#define serverHTTP

static const char* const      mqtt_server                    = "192.168.1.56";
static const uint16_t         mqtt_port                      = 1883;
static const char* const      mqtt_username                  = "datel";
static const char* const      mqtt_key                       = "hanka12";
static const char* const      mqtt_base                      = "/home/Meteo";
static const char* const      mqtt_topic_restart             = "restart";
static const char* const      mqtt_topic_netinfo             = "netinfo";
static const char* const      mqtt_config_portal             = "config";
static const char* const      mqtt_config_portal_stop        = "disconfig";


//All of the IO pins have interrupt/pwm/I2C/one-wire support except D0.
#define ONE_WIRE_BUS                        D4 //Dallas

#define SDAPIN                              D6
#define SCLPIN                              D5

#define SEND_DELAY                           15000  //prodleva mezi poslanim dat v ms
#define SENDSTAT_DELAY                       60000  //poslani statistiky kazdou minutu
#define MEAS_DELAY                           5000   //mereni
#define CONNECT_DELAY                        5000 //ms

#define MEAS_TIME                            750 //in ms

#include <fce.h>

#endif
