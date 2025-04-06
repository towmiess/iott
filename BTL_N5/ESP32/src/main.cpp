// 🔹 Cấu hình Blynk
#define BLYNK_TEMPLATE_ID "TMPL6MqMFPbAp"
#define BLYNK_TEMPLATE_NAME "IOTN5 ESP32 DHT11 FAN"
#define BLYNK_AUTH_TOKEN "npjZsmVBGbYBMZv14oLN1tcFthiGBG8c"

#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <BlynkSimpleEsp32.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <FirebaseESP32.h>

// 🔹 Cấu hình WiFi
#define WIFI_SSID "towmiess"
#define WIFI_PASSWORD "12345678"

// 🔹 Cấu hình Firebase
#define FIREBASE_HOST "https://dht11-125c7-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "gcvJ7k7KyjJYpLawuXZAEBymwm4aW6yFgZY56Zcq"
FirebaseData firebaseData;
FirebaseAuth firebaseAuth;
FirebaseConfig firebaseConfig;

// 🔹 Cấu hình DHT11
#define DHT_PIN 15
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// 🔹 Cấu hình Relay (Quạt)
#define RELAY_FAN_PIN 14
bool fanState = false;
bool autoFanControl = true;

// 🔹 Cấu hình MQTT (HiveMQ)
#define MQTT_SERVER "e4d6461e44b845fdbc9a32917b240fa3.s1.eu.hivemq.cloud"
#define MQTT_PORT 8883
#define MQTT_USER "IOTN5"
#define MQTT_PASSWORD "Nhom5IOT"
#define MQTT_PUBLISH_TOPIC "esp32/sensor/data"
#define MQTT_SUBSCRIBE_TOPIC "esp32/fan/control"

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

SemaphoreHandle_t wifiMutex;
SemaphoreHandle_t tempMutex;
float lastTemperature = 0.0;

// 📡 Kết nối WiFi
void WiFiTask(void *pvParameters) {
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.print("🔄 Đang kết nối WiFi...");
            
            // 🛑 Giữ quyền truy cập WiFi
            xSemaphoreTake(wifiMutex, portMAX_DELAY);

            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            int attempt = 0;
            while (WiFi.status() != WL_CONNECTED && attempt < 10) {
                delay(1000);
                Serial.print(".");
                attempt++;
            }

            // ✅ Trả quyền truy cập WiFi
            xSemaphoreGive(wifiMutex);

            Serial.println(WiFi.status() == WL_CONNECTED ? "✅ WiFi Kết nối!" : "❌ Lỗi WiFi!");
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}


// 📡 Nhận dữ liệu từ MQTT
void callbackMQTT(char *topic, byte *payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    if (String(topic) == MQTT_SUBSCRIBE_TOPIC) {
        bool command = (message == "ON");
        if (autoFanControl) {
            Serial.println("⛔ Không thể thao tác thủ công khi Auto bật!");
        } else {
            fanState = command;
            digitalWrite(RELAY_FAN_PIN, fanState ? LOW : HIGH);
            Blynk.virtualWrite(V1, fanState);
            Serial.printf("🔹 Quạt %s\n", fanState ? "BẬT" : "TẮT");
        }
    }
}
// 📡 Kết nối MQTT
void connectMQTT() {
    espClient.setInsecure(); // Bỏ qua kiểm tra chứng chỉ SSL (dùng cho MQTTs nếu cần)
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT); // Thiết lập máy chủ MQTT
    mqttClient.setCallback(callbackMQTT); // Định nghĩa hàm xử lý khi nhận dữ liệu từ MQTT

    while (!mqttClient.connected()) {
        Serial.print("🔄 Kết nối MQTT...");
        if (mqttClient.connect("ESP32_Client", MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("✅ MQTT Kết nối!");
            mqttClient.subscribe(MQTT_SUBSCRIBE_TOPIC); // Đăng ký chủ đề để nhận lệnh
        } else {
            Serial.println("❌ Lỗi MQTT, thử lại...");
            vTaskDelay(5000 / portTICK_PERIOD_MS); // Đợi 5 giây trước khi thử lại
        }
    }
}

// 📡 Xử lý MQTT
void MQTTTask(void *pvParameters) {
    connectMQTT(); // Kết nối MQTT khi khởi động
    while (true) {
        if (!mqttClient.connected()) {
            connectMQTT(); // Nếu mất kết nối, thử kết nối lại
        }
        mqttClient.loop(); // Lắng nghe và xử lý dữ liệu MQTT
        vTaskDelay(100 / portTICK_PERIOD_MS); // Chờ 100ms trước khi kiểm tra lại
    }
}



// 📡 Đọc cảm biến DHT11
void DHTTask(void *pvParameters) {
    while (true) {
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();

        if (!isnan(temperature) && !isnan(humidity)) {
            // 🛑 Giữ quyền truy cập dữ liệu nhiệt độ
            xSemaphoreTake(tempMutex, portMAX_DELAY);// Chiếm quyền truy cập biến nhiệt độ
            lastTemperature = temperature;// Lưu nhiệt độ đọc được
            xSemaphoreGive(tempMutex);//✅ Trả quyền truy cập dữ liệu nhiệt độ

            Serial.printf("🌡 Nhiệt độ: %.1f°C, Độ ẩm: %.1f%%\n", temperature, humidity);
            Blynk.virtualWrite(V2, temperature);
            Blynk.virtualWrite(V3, humidity);
            // Tạo chuỗi JSON để gửi dữ liệu lên MQTT
            String mqttMessage = "{\"temperature\": " + String(temperature) + ", \"humidity\": " + String(humidity) + "}";
            mqttClient.publish(MQTT_PUBLISH_TOPIC, mqttMessage.c_str());// Gửi dữ liệu lên MQTT
        } else {
            Serial.println("❌ Lỗi cảm biến!");
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

// 📡 Điều khiển quạt tự động
void FanControlTask(void *pvParameters) {
    while (true) {
        // 🛑 Đọc giá trị nhiệt độ từ biến lastTemperature
        // Sử dụng mutex để đảm bảo rằng chỉ một task có thể truy cập biến này tại một thời điểm
        xSemaphoreTake(tempMutex, portMAX_DELAY);// Chiếm quyền truy cập biến nhiệt độ
        float temp = lastTemperature; // Lưu giá trị nhiệt độ hiện tại
        xSemaphoreGive(tempMutex);// Giải phóng quyền truy cập biến nhiệt độ
        // 🔹 Nếu chế độ tự động bật, điều khiển quạt theo nhiệt độ
        if (autoFanControl) {
            if (temp > 25.0 && !fanState) {
                fanState = true;
                digitalWrite(RELAY_FAN_PIN, LOW);
                Blynk.virtualWrite(V1, 1);
                Serial.println("🌀 Quạt bật (Tự động)");
            } else if (temp <= 25.0 && fanState) {
                fanState = false;
                digitalWrite(RELAY_FAN_PIN, HIGH);
                Blynk.virtualWrite(V1, 0);
                Serial.println("❄️ Quạt tắt (Tự động)");
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// 📡 Gửi dữ liệu lên Firebase
void FirebaseTask(void *pvParameters) {
    // Cấu hình Firebase
    firebaseConfig.database_url = FIREBASE_HOST;
    firebaseAuth.user.email = "";
    firebaseAuth.user.password = "";
    firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;

    Firebase.begin(&firebaseConfig, &firebaseAuth);
    Firebase.reconnectWiFi(true);

    while (true) {
        if (WiFi.status() == WL_CONNECTED && Firebase.ready()) {
            // 🔐 Đọc nhiệt độ từ biến dùng mutex
            xSemaphoreTake(tempMutex, portMAX_DELAY);
            float temp = lastTemperature;
            xSemaphoreGive(tempMutex);

            // Gửi nhiệt độ và độ ẩm lên Firebase
            String path = "/esp32";
            Firebase.setFloat(firebaseData, path + "/temperature", temp);
            Firebase.setFloat(firebaseData, path + "/humidity", dht.readHumidity());
        } else {
            Serial.println("⚠️ Firebase chưa sẵn sàng hoặc WiFi mất kết nối!");
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Gửi mỗi 5 giây
    }
}

// 📡 Nhận lệnh từ Blynk
BLYNK_WRITE(V4) {
    // 🛠️ Nhận dữ liệu từ Blynk để bật/tắt chế độ quạt tự động
    autoFanControl = param.asInt(); // Lưu trạng thái của chế độ tự động
    Serial.printf("⚙️ Chế độ tự động: %s\n", autoFanControl ? "BẬT" : "TẮT");
}

BLYNK_WRITE(V1) {
    // 🔹 Nếu chế độ tự động đang tắt, cho phép điều khiển quạt thủ công
    if (!autoFanControl) {
        fanState = param.asInt(); // Lưu trạng thái mới của quạt
        digitalWrite(RELAY_FAN_PIN, fanState ? LOW : HIGH); // Điều khiển relay
        Serial.printf("🔹 Quạt %s\n", fanState ? "BẬT" : "TẮT");
    } else {
        // ⛔ Nếu chế độ tự động đang bật, không cho phép điều khiển thủ công
        Serial.println("⛔ Không thể thao tác thủ công khi Auto bật!");
        Blynk.virtualWrite(V1, fanState); // Cập nhật trạng thái hiển thị trên Blynk
    }
}

// 🔹 Cấu hình ESP32
void setup() {
    Serial.begin(115200); // Khởi động Serial Monitor với baud rate 115200
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Kết nối WiFi

    dht.begin(); // Khởi tạo cảm biến DHT11
    pinMode(RELAY_FAN_PIN, OUTPUT); // Thiết lập chân relay của quạt là OUTPUT
    digitalWrite(RELAY_FAN_PIN, HIGH); // Mặc định tắt quạt khi khởi động

    Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD); // Kết nối Blynk

    // Tạo mutex để quản lý truy cập WiFi và biến nhiệt độ
    wifiMutex = xSemaphoreCreateMutex();
    tempMutex = xSemaphoreCreateMutex();

    // Khởi tạo các task FreeRTOS để chạy song song
    xTaskCreate(WiFiTask, "WiFiTask", 4096, NULL, 1, NULL);
    xTaskCreate(MQTTTask, "MQTTTask", 4096, NULL, 1, NULL);
    xTaskCreate(DHTTask, "DHTTask", 4096, NULL, 1, NULL);
    xTaskCreate(FanControlTask, "FanControlTask", 4096, NULL, 1, NULL);
    xTaskCreate(FirebaseTask, "FirebaseTask", 4096, NULL, 1, NULL);

}

// 🔄 Loop chính, chỉ chạy Blynk
void loop() {
    Blynk.run();
}
