#include <WiFi.h>
#include <HTTPClient.h>
#include "DHT.h"

#define DHTPIN 14
#define DHTTYPE DHT22

#define LDRPIN 34 // chân Analog đọc LDR

#define TRIGPIN 12
#define ECHOPIN 13

const char* ssid = "Wokwi-GUEST";
const char* password = "";

String GAS_URL = "https://script.google.com/macros/s/AKfycbx3WvT-GBCP7CEMOzF185cAM3W4nDijUFHtDQHQW7X8ax-lK80cCNo-v_T2tZYAHXFaCQ/exec";

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");
}

float readDistanceCM() {
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  int duration = pulseIn(ECHOPIN, HIGH);
  return duration * 0.034 / 2;
}

void loop() {
  delay(10000);

  float temp = dht.readTemperature();
  float humi = dht.readHumidity();
  int light = analogRead(LDRPIN);
  float distance = readDistanceCM();

  if (isnan(temp) || isnan(humi)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.printf("Temp: %.2f°C, Humi: %.2f%%, Light: %d, Distance: %.2fcm\n", temp, humi, light, distance);

  if (WiFi.status() == WL_CONNECTED) {
    delay(2000);
    HTTPClient http;
    String url = GAS_URL + "?temp=" + String(temp, 2) + "%C2%B0C" +
                            "&humi=" + String(humi, 2) + "%25" +
                            "&light=" + String(light) + "%20lux" +
                            "&distance=" + String(distance, 2) + "%20cm";

    Serial.println("Sending data: " + url);
    http.begin(url);
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      Serial.println("Response: " + http.getString());
    } else {
      Serial.println("Error sending data!");
    }
    http.end();
  }
}
