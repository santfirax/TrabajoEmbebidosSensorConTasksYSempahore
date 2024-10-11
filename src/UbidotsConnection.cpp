#include "UbidotsConnection.h"

UbidotsConnection::UbidotsConnection(const String &apiKey) : apiKey(apiKey), baseUrl("http://industrial.api.ubidots.com/api/v1.6/devices/") {}

bool UbidotsConnection::sendData(const String &deviceLabel, const String &variableLabel, float value) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected");
        return false;
    }

    HTTPClient http;
    String url = baseUrl + deviceLabel + "/" + variableLabel + "/values";
    http.begin(url);

    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Auth-Token", apiKey);

    String payload = "{\"value\":" + String(value) + "}";

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode == 201) {
        Serial.println("Data sent successfully");
        http.end();
        return true;
    } else {
        Serial.print("Error sending data. HTTP response code: ");
        Serial.println(httpResponseCode);
        http.end();
        return false;
    }
}