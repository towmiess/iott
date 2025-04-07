#define BLYNK_TEMPLATE_ID "TMPL6vcy_-5sj"
#define BLYNK_TEMPLATE_NAME "RELAY MOLSUTURE"
#define BLYNK_AUTH_TOKEN "1kj1ZPzHix86IgCU3A_4nD-_arw7TK42"

#define BLYNK_PRINT Serial
#define APP_DEBUG

#include <WiFi.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <FirebaseESP32.h>
#include <BlynkSimpleEsp32.h>

// MQTT cấu hình
#define MQTT_SERVER "e644ce5e89354e76856d7f082f98aff4.s1.eu.hivemq.cloud"
#define MQTT_PORT 8883
#define MQTT_USER "NTV_N5_IOT"
#define MQTT_PASSWORD "Viet0000@"
#define MQTT_TOPIC "tuoi_cay/soil_moisture"

// Firebase
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// Cảm biến & Relay
const int soilMoisturePin = 35;
const int relayPin = 27;
unsigned long lastMillis = 0;
const int interval = 5000;

// Blynk
bool isManualMode = false;        // Mặc định là Auto
bool manualRelayState = false;

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// Gửi dữ liệu lên Firebase
void sendToFirebase(int soilMoisture) {
  String path = "/sensor_data/soil_moisture";
  if (Firebase.setInt(firebaseData, path, soilMoisture)) {
    Serial.println(" Đã gửi dữ liệu lên Firebase!");
  } else {
    Serial.print(" Lỗi Firebase: ");
    Serial.println(firebaseData.errorReason());
  }
}

// Nhận dữ liệu từ MQTT
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  Serial.print(" Nhận từ MQTT: ");
  Serial.println(topic);

  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  int soilMoisture = message.toInt();
  Serial.print(" Độ ẩm từ MQTT: ");
  Serial.println(soilMoisture);

  sendToFirebase(soilMoisture);
}

// Kết nối MQTT
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

// Kết nối WiFi
void connectWiFi() {
  WiFi.begin("DUY PHUC", "ngoquocdat");
  Serial.print(" Kết nối WiFi...");
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempt++;
    if (attempt > 30) {
      Serial.println("\n Không kết nối được WiFi! Reset...");
      ESP.restart();
    }
  }
  Serial.println("\n WiFi kết nối thành công!");
}

// BLYNK: Điều khiển relay thủ công (V1)
BLYNK_WRITE(V1) {
  manualRelayState = param.asInt();
  if (isManualMode) {
    digitalWrite(relayPin, manualRelayState ? LOW : HIGH);
    // Serial.println(manualRelayState ? "THỦ CÔNG: Bật máy bơm" : "THỦ CÔNG: Tắt máy bơm");
  }
}

// BLYNK: Hiển thị trạng thái kết nối LED (V0)
// Tượng trưng =))) 
void updateConnectionStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V0, 255);
  } else {
    Blynk.virtualWrite(V0, 0);
  }
}

// BLYNK: Chuyển chế độ Auto <-> Manual (V3)
BLYNK_WRITE(V3) {
  isManualMode = param.asInt();
  if (isManualMode) {
    Serial.println("Chế độ: THỦ CÔNG");
  } else {
    Serial.println("Chế độ: TỰ ĐỘNG");
    // Khi chuyển sang tự động, tắt relay nếu không cần thiết
    digitalWrite(relayPin, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Tắt máy bơm ban đầu

  connectWiFi();
  Blynk.begin(BLYNK_AUTH_TOKEN, WiFi.SSID().c_str(), WiFi.psk().c_str());

  // Firebase
  config.host = "nhom5-iot-9b213-default-rtdb.asia-southeast1.firebasedatabase.app";
  config.signer.tokens.legacy_token = "krTZyZmVADrnpIB3h1R4ztoRpic61RM5lc0UnS8k";
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  connectMQTT();
}

void loop() {
  mqttClient.loop();
  Blynk.run();

  // Cập nhật trạng thái kết nối WiFi
  updateConnectionStatus();
  
  if (!mqttClient.connected()) {
    connectMQTT();
  }

  if (millis() - lastMillis > interval) {
    int soilMoisture = analogRead(soilMoisturePin);
    soilMoisture = map(soilMoisture, 0, 4095, 100, 0);

    Serial.print("Độ ẩm đất: ");
    Serial.print(soilMoisture);
    Serial.println("%");

    // Gửi về Blynk
    Blynk.virtualWrite(V2, soilMoisture);

    // Điều khiển tự động nếu đang ở chế độ Auto
    if (!isManualMode) {
      if (soilMoisture < 30 && digitalRead(relayPin) == HIGH) {
        Serial.println("TỰ ĐỘNG: Bật máy bơm...");
        digitalWrite(relayPin, LOW);
      } else if (soilMoisture >= 30 && digitalRead(relayPin) == LOW) {
        Serial.println("TỰ ĐỘNG: Tắt máy bơm...");
        digitalWrite(relayPin, HIGH);
      }
    }

    // Gửi MQTT
    mqttClient.publish(MQTT_TOPIC, String(soilMoisture).c_str());

    // Gửi Firebase
    sendToFirebase(soilMoisture);

    lastMillis = millis();
  }
}
