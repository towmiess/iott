#define BLYNK_TEMPLATE_ID "TMPL6MqMFPbAp"
#define BLYNK_TEMPLATE_NAME "IOTN5 ESP32 DHT11 FAN"
#define BLYNK_AUTH_TOKEN "npjZsmVBGbYBMZv14oLN1tcFthiGBG8c"

#define BLYNK_PRINT Serial
#define APP_DEBUG

#include <WiFi.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <FirebaseESP32.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <ArduinoJson.h> 

// MQTT config
#define MQTT_SERVER "e4d6461e44b845fdbc9a32917b240fa3.s1.eu.hivemq.cloud"
#define MQTT_PORT 8883
#define MQTT_USER "IOTN5"
#define MQTT_PASSWORD "Nhom5IOT"
#define MQTT_TOPIC "esp32/sensor/data"

// Firebase
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// Cảm biến & Relay
const int soilMoisturePin = 35;
const int relayPin = 27;   // Máy bơm
const int relayPin1 = 19;  // Quạt
#define DHT_PIN 15
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// Blynk
bool isManualMode = false;
bool manualRelayState = false;
bool manualFanState = false;
BlynkTimer timer;

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// === Gửi lên Firebase ===
void sendToFirebase(int soilMoisture) {
  String path = "/sensor_data/soil_moisture";
  if (Firebase.setInt(firebaseData, path, soilMoisture)) {
    Serial.println(" Đã gửi độ ẩm lên Firebase!");
  } else {
    Serial.print(" Lỗi Firebase (độ ẩm): ");
    Serial.println(firebaseData.errorReason());
  }
}

void sendTempToFirebase(float temp) {
  String path = "/sensor_data/temperature";
  if (Firebase.setFloat(firebaseData, path, temp)) {
    Serial.println(" Đã gửi nhiệt độ lên Firebase!");
  } else {
    Serial.print(" Lỗi Firebase (nhiệt độ): ");
    Serial.println(firebaseData.errorReason());
  }
}

// === MQTT Callback ===
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  Serial.print(" Nhận từ MQTT: ");
  Serial.println(topic);

  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print(" Dữ liệu MQTT: ");
  Serial.println(message);

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.print(" Lỗi parse JSON MQTT: ");
    Serial.println(error.f_str());
    return;
  }

  if (doc.containsKey("soil")) {
    int soilMoisture = doc["soil"];
    Serial.print(" Độ ẩm từ MQTT: ");
    Serial.println(soilMoisture);
    Blynk.virtualWrite(V2, soilMoisture);
    sendToFirebase(soilMoisture);

    if (!isManualMode) {
      if (soilMoisture < 30 && digitalRead(relayPin) == HIGH) {
        Serial.println("TỰ ĐỘNG: Bật máy bơm...");
        digitalWrite(relayPin, LOW);
      } else if (soilMoisture >= 30 && digitalRead(relayPin) == LOW) {
        Serial.println("TỰ ĐỘNG: Tắt máy bơm...");
        digitalWrite(relayPin, HIGH);
      }
    }
  }

  if (doc.containsKey("temp")) {
    float temperature = doc["temp"];
    Serial.print(" Nhiệt độ từ MQTT: ");
    Serial.println(temperature);
    Blynk.virtualWrite(V5, temperature);
    sendTempToFirebase(temperature);

    if (!isManualMode) {
      if (temperature >= 27 && digitalRead(relayPin1) == HIGH) {
        Serial.println("TỰ ĐỘNG: Bật quạt...");
        digitalWrite(relayPin1, LOW);
      } else if (temperature < 27 && digitalRead(relayPin1) == LOW) {
        Serial.println("TỰ ĐỘNG: Tắt quạt...");
        digitalWrite(relayPin1, HIGH);
      }
    }
  }
}

// === Kết nối MQTT ===
void connectMQTT() {
  espClient.setInsecure();
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(callbackMQTT);
  mqttClient.setBufferSize(512);

  while (!mqttClient.connected()) {
    Serial.print(" Kết nối MQTT...");
    if (mqttClient.connect("ESP32_Client", MQTT_USER, MQTT_PASSWORD)) {
      Serial.println(" OK!");
      mqttClient.subscribe(MQTT_TOPIC);
    } else {
      Serial.print(" Lỗi MQTT: ");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
}

// === Kết nối WiFi ===
void connectWiFi() {
  WiFi.begin("towmiess", "12345678");
  Serial.print(" Kết nối WiFi...");
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++attempt > 30) {
      Serial.println("\n Không kết nối được WiFi! Reset...");
      ESP.restart();
    }
  }
  Serial.println("\n WiFi kết nối thành công!");
}

// === Gửi sensor định kỳ & publish MQTT ===
void sendSensorData() {
  int soilMoisture = analogRead(soilMoisturePin);
  soilMoisture = map(soilMoisture, 0, 4095, 100, 0);
  float temperature = dht.readTemperature();

  Serial.print("Độ ẩm đất: ");
  Serial.print(soilMoisture);
  Serial.println("%");
  Serial.print("Nhiệt độ: ");
  Serial.print(temperature);
  Serial.println("°C");

  // Gửi MQTT (JSON)
  StaticJsonDocument<200> doc;
  doc["soil"] = soilMoisture;
  doc["temp"] = temperature;
  String jsonPayload;
  serializeJson(doc, jsonPayload);
  mqttClient.publish(MQTT_TOPIC, jsonPayload.c_str());
}

// === Blynk điều khiển ===
BLYNK_WRITE(V1) {
  manualRelayState = param.asInt();
  if (isManualMode) {
    digitalWrite(relayPin, manualRelayState ? LOW : HIGH);
  }
}

BLYNK_WRITE(V4) {
  manualFanState = param.asInt();
  if (isManualMode) {
    digitalWrite(relayPin1, manualFanState ? LOW : HIGH);
  }
}

BLYNK_WRITE(V3) {
  isManualMode = param.asInt();
  if (isManualMode) {
    Serial.println("Chế độ: THỦ CÔNG");
  } else {
    Serial.println("Chế độ: TỰ ĐỘNG");
    digitalWrite(relayPin, HIGH);
    digitalWrite(relayPin1, HIGH);
  }
}

void updateConnectionStatus() {
  Blynk.virtualWrite(V0, WiFi.status() == WL_CONNECTED ? 255 : 0);
}

// === SETUP ===
void setup() {
  Serial.begin(115200);
  pinMode(relayPin, OUTPUT);
  pinMode(relayPin1, OUTPUT);
  digitalWrite(relayPin, HIGH);   // Tắt máy bơm ban đầu
  digitalWrite(relayPin1, HIGH);  // Tắt quạt ban đầu

  connectWiFi();
  Blynk.begin(BLYNK_AUTH_TOKEN, WiFi.SSID().c_str(), WiFi.psk().c_str());
  dht.begin();

  config.host = "https://dht11-125c7-default-rtdb.firebaseio.com/";
  config.signer.tokens.legacy_token = "gcvJ7k7KyjJYpLawuXZAEBymwm4aW6yFgZY56Zcq";
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  connectMQTT();
  timer.setInterval(5000L, sendSensorData);  // Gửi sensor mỗi 5s
}

// === LOOP ===
void loop() {
  mqttClient.loop();
  Blynk.run();
  timer.run();
  updateConnectionStatus();

  if (!mqttClient.connected()) {
    connectMQTT();
  }
}
