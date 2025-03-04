#include "esp_camera.h"
#include "DHTesp.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <FirebaseESP32.h>

// WiFi and MQTT configuration
const char* ssid = "towmiess";
const char* password = "12345678";
const char* mqtt_server = "c6d333eb510a4e1eb70c3b83e56f0c05.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "hxc+123";
const char* mqtt_password = "Dodinhcuong2003";

#define FIREBASE_HOST "https://nhom5-iot-9b213-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "AIzaSyBX-bncTHdaEGGCjo3NrRXacJc89r4KNHs"

// GPIO pin definitions
#define DHT_PIN 16
#define RELAY_MOTOR 14
#define RELAY_LOCK_BUZZER 2
#define LIGHT_PIN 12

DHTesp dht;
WiFiClientSecure espClient;
PubSubClient client(espClient);
FirebaseData firebaseData;
unsigned long lastMsg = 0;
unsigned long timeUpdate = millis();

void setup_wifi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientID = "ESPClient-" + String(random(0xffff), HEX);
        if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT");
            client.subscribe("esp32/control");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" trying again in 5 seconds");
            delay(5000);
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (int i = 0; i < length; i++) message += (char)payload[i];
    Serial.println("Message received [" + String(topic) + "]: " + message);

    if (message == "open") {
        digitalWrite(RELAY_LOCK_BUZZER, HIGH);
        digitalWrite(LIGHT_PIN, HIGH);
        delay(3000);
        digitalWrite(RELAY_LOCK_BUZZER, LOW);
    } else if (message == "light_on") {
        digitalWrite(LIGHT_PIN, HIGH);
    } else if (message == "light_off") {
        digitalWrite(LIGHT_PIN, LOW);
    } else if (message == "motor_on") {
        digitalWrite(RELAY_MOTOR, HIGH);
    } else if (message == "motor_off") {
        digitalWrite(RELAY_MOTOR, LOW);
    }
}

void publishMessage(const char* topic, String payload) {
    client.publish(topic, payload.c_str(), true);
    Serial.println("Published: " + payload);
}

void setup() {
    Serial.begin(115200);
    pinMode(RELAY_MOTOR, OUTPUT);
    pinMode(RELAY_LOCK_BUZZER, OUTPUT);
    pinMode(LIGHT_PIN, OUTPUT);
    dht.setup(DHT_PIN, DHTesp::DHT11);
    setup_wifi();
    espClient.setInsecure();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
        setup_wifi();
    }
    if (!client.connected()) reconnect();
    client.loop();

    // if (millis() - timeUpdate > 2000) {
    //     float h = dht.getHumidity();
    //     float t = dht.getTemperature();
    //     DynamicJsonDocument doc(1024);
    //     doc["humidity"] = h;
    //     doc["temperature"] = t;
    //     char mqtt_message[128];
    //     serializeJson(doc, mqtt_message);
    //     publishMessage("esp32/dht11", mqtt_message);
    //     timeUpdate = millis();
    //     if (t > 30) digitalWrite(RELAY_MOTOR, HIGH);
    //     else digitalWrite(RELAY_MOTOR, LOW);
    // }
    if (millis() - timeUpdate > 2000) {
    float h = dht.getHumidity();
    float t = dht.getTemperature();

    // 🖥 In dữ liệu lên Serial Monitor
    Serial.print("🌡️ Nhiệt độ: ");
    Serial.print(t);
    Serial.print(" °C, 💧 Độ ẩm: ");
    Serial.print(h);
    Serial.println(" %");

    // Gửi lên MQTT
    DynamicJsonDocument doc(1024);
    doc["humidity"] = h;
    doc["temperature"] = t;
    char mqtt_message[128];
    serializeJson(doc, mqtt_message);
    publishMessage("esp32/dht11", mqtt_message);

    // Điều khiển quạt theo nhiệt độ
    if (t > 30) {
        digitalWrite(RELAY_MOTOR, HIGH);
        Serial.println("🔥 Quạt bật do nhiệt độ cao!");
    } else {
        digitalWrite(RELAY_MOTOR, LOW);
        Serial.println("❄️ Quạt tắt do nhiệt độ thấp!");
    }

    timeUpdate = millis();
}

}