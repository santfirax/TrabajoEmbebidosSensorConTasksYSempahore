#ifndef UBIDOTS_CONNECTION_H
#define UBIDOTS_CONNECTION_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>

class UbidotsConnection {
private:
    String apiKey;
    String baseUrl;

public:
    UbidotsConnection(const String &apiKey);
    bool sendData(const String &deviceLabel, const String &variableLabel, float value);
};

#endif