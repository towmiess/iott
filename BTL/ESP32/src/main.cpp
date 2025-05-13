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
//🔹 Cấu hình Cảm biến đất
#define SOIL_SENSOR_PIN 35

// 🔹 Cấu hình Relay (Quạt)
#define RELAY_FAN_PIN 14
bool fanState = false;
bool autoFanControl = false;

// 🔹 Cấu hình Relay (Máy Bơm)
#define RELAY_PUMP_PIN 27
bool PumpState = false;
bool autoPumpControl = false;

// 🔹 Cảm biến khí gas (ví dụ MQ2)
#define GAS_SENSOR_PIN 34
#define BUZZER_PIN 26
#define RELAY_LED_PIN     21  // Chân điều khiển relay của đèn siêu âm (IN1)
#define RELAY_BUZZER_PIN  22  // Chân điều khiển relay của còi khí gas (IN2)
int gasThreshold = 400; // Giá trị mặc định, có thể thay đổi từ Blynk
// Biến lưu giá trị cảm biến khí gas
int lastGasValue = 0;
bool gasAlertActive = false;
unsigned long gasAlertStartTime = 0;


// 🔹 Cảm biến siêu âm
#define TRIG_PIN 12
#define ECHO_PIN 13
#define LED_PIN 25
int distanceThreshold = 20; // Ngưỡng khoảng cách bật đèn (cm)
// Biến lưu khoảng cách đo được từ cảm biến siêu âm
long lastDistance = 0;

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
int lastSoilMoisture = 0; // Giá trị độ ẩm đất
SemaphoreHandle_t soilMutex; 
SemaphoreHandle_t xGasMutex;
SemaphoreHandle_t xUltrasonicMutex;


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

int readSoilMoisture() {
  int analogValue = analogRead(SOIL_SENSOR_PIN);
  // Tùy theo cảm biến, bạn có thể map về phần trăm như sau:
  int moisturePercent = map(analogValue, 0, 4095, 100, 0);
  return moisturePercent;
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

void SoilTask(void *pvParameters) {
  while (true) {
      int rawSoil = analogRead(35); // Đọc giá trị từ cảm biến
      int soilPercent = map(rawSoil, 0, 4095, 100, 0); // Chuyển đổi sang % (tuỳ cảm biến)

      // Giữ quyền truy cập biến soil
      xSemaphoreTake(soilMutex, portMAX_DELAY);
      lastSoilMoisture = soilPercent;
      xSemaphoreGive(soilMutex);

      Serial.printf("🌱 Độ ẩm đất: %d%% (raw: %d)\n", soilPercent, rawSoil);

      // Gửi lên Blynk (ví dụ Virtual Pin V4)
      Blynk.virtualWrite(V5, soilPercent);

      // Tạo JSON gửi MQTT
      String mqttMessage = "{\"soil\": " + String(soilPercent) + "}";
      mqttClient.publish(MQTT_PUBLISH_TOPIC, mqttMessage.c_str());

      vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay 5s
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

// 📡 Điều khiển máy bơm tự động
void PumpControlTask(void *pvParameters) {
    while (true) {
        // Đọc độ ẩm đất an toàn
        xSemaphoreTake(soilMutex, portMAX_DELAY);
        int soil = lastSoilMoisture;
        xSemaphoreGive(soilMutex);

        // 🔄 Nếu chế độ tự động bật, điều khiển bơm theo độ ẩm đất
        if (autoPumpControl) {
            if (soil < 30 && !PumpState) {
                PumpState = true;
                digitalWrite(RELAY_PUMP_PIN, LOW);  // LOW để bật relay
                Blynk.virtualWrite(V7, 1);
                Serial.println("💧 Bơm BẬT (Tự động)");
            } else if (soil >= 50 && PumpState) {
                PumpState = false;
                digitalWrite(RELAY_PUMP_PIN, HIGH); // HIGH để tắt relay
                Blynk.virtualWrite(V7, 0);
                Serial.println("💧 Bơm TẮT (Tự động)");
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// 📡 Đọc cảm biến khí gas
void GasSensorTask(void *pvParameters) {
    while (true) {
        // Đảm bảo chỉ một task có thể đọc cảm biến khí gas tại một thời điểm
        if (xSemaphoreTake(xGasMutex, portMAX_DELAY) == pdTRUE) {
            int gasValue = analogRead(GAS_SENSOR_PIN);
            Serial.printf("🚨 Giá trị khí gas: %d\n", gasValue);

            if (gasValue >= gasThreshold && !gasAlertActive) {
                digitalWrite(RELAY_BUZZER_PIN, LOW);  // Bật còi
                gasAlertActive = true;
                gasAlertStartTime = millis();
                Serial.println("🔊 Báo động khí gas!");
            } 
            if (gasAlertActive && millis() - gasAlertStartTime >= 3000) {
                digitalWrite(RELAY_BUZZER_PIN, HIGH); // Tắt còi
                gasAlertActive = false;
            }

            Blynk.virtualWrite(V8, gasValue); // Gửi giá trị gas về Blynk

            xSemaphoreGive(xGasMutex); // Giải phóng mutex sau khi hoàn thành
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Đọc lại mỗi 5 giây
    }
}


long readDistanceCM() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout 30ms
    long distance = duration * 0.034 / 2;
    return distance;
}

// 📡 Cảm biến siêu âm
void UltrasonicTask(void *pvParameters) {
    while (true) {
        // Đảm bảo chỉ một task có thể đọc cảm biến siêu âm tại một thời điểm
        if (xSemaphoreTake(xUltrasonicMutex, portMAX_DELAY) == pdTRUE) {
            long distance = readDistanceCM();
            Serial.printf("📏 Khoảng cách đo được: %ld cm\n", distance);

            if (distance > 0 && distance <= distanceThreshold) {
                digitalWrite(RELAY_LED_PIN, LOW);   // Bật đèn
                Serial.println("💡 Vật thể gần - Bật đèn");
            } else {
                digitalWrite(RELAY_LED_PIN, HIGH); // Tắt đèn
            }

            Blynk.virtualWrite(V9, distance); // Gửi dữ liệu về Blynk

            xSemaphoreGive(xUltrasonicMutex); // Giải phóng mutex sau khi hoàn thành
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Đọc lại mỗi 5 giây
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
            String path = "/DHT11";
            Firebase.setFloat(firebaseData, path + "/temperature", temp);
            Firebase.setFloat(firebaseData, path + "/humidity", dht.readHumidity());
            // Đọc độ ẩm đất từ biến dùng mutex
            xSemaphoreTake(soilMutex, portMAX_DELAY);
            int soil = lastSoilMoisture;
            xSemaphoreGive(soilMutex);

            // Gửi dữ liệu Soil Moisture
             String soilPath = "/sensor_data";
             Firebase.setInt(firebaseData, soilPath + "/soil_moisture", soil);

            // 🔐 Đọc giá trị khí gas từ biến dùng mutex
            xSemaphoreTake(xGasMutex, portMAX_DELAY);
            int gasValue = lastGasValue; // Sử dụng biến gasValue được cập nhật trong GasSensorTask
            xSemaphoreGive(xGasMutex);

            // Gửi dữ liệu khí gas lên Firebase
            String gasPath = "/sensor_data";
            Firebase.setInt(firebaseData, gasPath + "/gas", gasValue);

            // 🔐 Đọc khoảng cách siêu âm từ biến dùng mutex
            xSemaphoreTake(xUltrasonicMutex, portMAX_DELAY);
            long distance = lastDistance; // Sử dụng biến distance được cập nhật trong UltrasonicTask
            xSemaphoreGive(xUltrasonicMutex);

            // Gửi dữ liệu khoảng cách siêu âm lên Firebase
            String ultrasonicPath = "/sensor_data";
            Firebase.setFloat(firebaseData, ultrasonicPath + "/distance", distance);

             yield();
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
// ⚙️ Nút auto máy bơm - V6
BLYNK_WRITE(V6) {
    autoPumpControl = param.asInt();
    Serial.println(autoPumpControl ? "🟢 Auto Pump: BẬT" : "🔴 Auto Pump: TẮT");
}

// 👆 Nút điều khiển thủ công - V7
BLYNK_WRITE(V7) {
    if (!autoPumpControl) {
        PumpState = param.asInt();
        digitalWrite(RELAY_PUMP_PIN, PumpState ? LOW : HIGH);
        Serial.println(PumpState ? "🖐️ Bật bơm (Thủ công)" : "🖐️ Tắt bơm (Thủ công)");
    } else {
        Blynk.virtualWrite(V7, PumpState);  // Phản hồi lại trạng thái thật
        Serial.println("⛔ Không thể điều khiển máy bơm thủ công khi đang ở chế độ tự động.");
    }
}

// Ngưỡng khí gas (V10)
BLYNK_WRITE(V10) {
    gasThreshold = param.asInt();
    Serial.printf("⚙️ Ngưỡng khí gas mới: %d\n", gasThreshold);
}

// Ngưỡng khoảng cách siêu âm (V11)
BLYNK_WRITE(V11) {
    distanceThreshold = param.asInt();
    Serial.printf("⚙️ Ngưỡng khoảng cách mới: %d cm\n", distanceThreshold);
}

// 🔹 Cấu hình ESP32
void setup() {
    Serial.begin(115200); // Khởi động Serial Monitor với baud rate 115200
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Kết nối WiFi

    dht.begin(); // Khởi tạo cảm biến DHT11
    pinMode(RELAY_FAN_PIN, OUTPUT); // Thiết lập chân relay của quạt là OUTPUT
    digitalWrite(RELAY_FAN_PIN, HIGH); // Mặc định tắt quạt khi khởi động

    pinMode(RELAY_PUMP_PIN, OUTPUT);
    digitalWrite(RELAY_PUMP_PIN, HIGH);  // Tắt bơm ban đầu
    Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD); // Kết nối Blynk

    // Gas sensor & buzzer
    pinMode(GAS_SENSOR_PIN, INPUT);
    pinMode(RELAY_BUZZER_PIN, OUTPUT);
    digitalWrite(RELAY_BUZZER_PIN, HIGH);  // Tắt còi ban đầu (relay không cấp điện)

    // Ultrasonic sensor
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(RELAY_LED_PIN, OUTPUT);
    digitalWrite(RELAY_LED_PIN, HIGH);    // Tắt đèn ban đầu

    // Tạo mutex để quản lý truy cập WiFi và biến nhiệt độ
    wifiMutex = xSemaphoreCreateMutex();
    tempMutex = xSemaphoreCreateMutex();
    soilMutex = xSemaphoreCreateMutex();
    xGasMutex = xSemaphoreCreateMutex();
    xUltrasonicMutex = xSemaphoreCreateMutex();


    // Khởi tạo các task FreeRTOS để chạy song song
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(MQTTTask, "MQTTTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(DHTTask, "DHTTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(SoilTask, "Soil Task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(FanControlTask, "FanControlTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(PumpControlTask, "PumpControlTask",4096,NULL,1,NULL,0);
    xTaskCreatePinnedToCore(GasSensorTask, "GasSensorTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(UltrasonicTask, "UltrasonicTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(FirebaseTask, "FirebaseTask", 10240, NULL, 1, NULL, 1);

}

// 🔄 Loop chính, chỉ chạy Blynk
void loop() {
    Blynk.run();
}