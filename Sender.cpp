#include "Sender.h"
#include <PubSubClient.h>

SenderClass::SenderClass() {
    _jsonVariant = _jsonBuffer.createObject();
}
void SenderClass::add(String id, float value) {
    _jsonVariant[id] = value;
}
void SenderClass::add(String id, String value) {
    _jsonVariant[id] = value;
}
void SenderClass::add(String id, uint32_t value) {
    _jsonVariant[id] = value;
}
void SenderClass::add(String id, int32_t value) {
    _jsonVariant[id] = value;
}

bool SenderClass::sendMQTT(String server, uint16_t port, String username, String password, String name) {
    _mqttClient.setClient(_client);
    _mqttClient.setServer(server.c_str(), port);
    _mqttClient.setCallback([this](char *topic, byte *payload, unsigned int length) { this->mqttCallback(topic, payload, length); });

    byte i = 0;

    while (!_mqttClient.connected() && (i < 3)) {
        DEBUG_PRINTLN(F("Attempting MQTT connection"));
        // Attempt to connect
        if (_mqttClient.connect(name.c_str(), username.c_str(), password.c_str())) {
            DEBUG_PRINTLN(F("Connected to MQTT"));
        } else {
            DEBUG_PRINTLN(F("Failed MQTT connection, return code:"));

            int Status = _mqttClient.state();

            switch (Status) {
            case -4:
              DEBUG_PRINTLN(F("Connection timeout"));
              break;

            case -3:
              DEBUG_PRINTLN(F("Connection lost"));
              break;

            case -2:
              DEBUG_PRINTLN(F("Connect failed"));
              break;

            case -1:
              DEBUG_PRINTLN(F("Disconnected"));
              break;

            case 1:
              DEBUG_PRINTLN(F("Bad protocol"));
              break;

            case 2:
              DEBUG_PRINTLN(F("Bad client ID"));
              break;

            case 3:
              DEBUG_PRINTLN(F("Unavailable"));
              break;

            case 4:
              DEBUG_PRINTLN(F("Bad credentials"));
              break;

            case 5:
              DEBUG_PRINTLN(F("Unauthorized"));
              break;
            }
            DEBUG_PRINTLN(F("Retrying MQTT connection in 5 seconds"));
            // Wait 5 seconds before retrying
            i++;
            delay(5000);
        }
    }
    //MQTT publish values
    for (const auto &kv : _jsonVariant.as<JsonObject>()) {
        DEBUG_PRINTLN("MQTT publish: " + name + "/" + kv.key + "/" + kv.value.as<String>());
        _mqttClient.publish((name + "/" + kv.key).c_str(), kv.value.as<String>().c_str());
        _mqttClient.loop(); //This should be called regularly to allow the client to process incoming messages and maintain its connection to the server.
    }

    DEBUG_PRINTLN(F("Closing MQTT connection"));
    _mqttClient.disconnect();
    delay(100); // allow gracefull session close
    return true;
}

void SenderClass::mqttCallback(char *topic, byte *payload, unsigned int length) {
    DEBUG_PRINTLN(F("MQTT message arrived ["));
    DEBUG_PRINTLN(topic);
    DEBUG_PRINTLN(F("] "));
    for (unsigned int i = 0; i < length; i++) {
        DEBUG_PRINTLN((char)payload[i]);
    }
}