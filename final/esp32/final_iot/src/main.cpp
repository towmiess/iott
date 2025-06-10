// ======= IOT + Google Assistant Voice Control with FreeRTOS (ESP32) =======

// -------------------- BLYNK CONFIGURATION --------------------
#define BLYNK_TEMPLATE_ID "TMPL6MqMFPbAp"
#define BLYNK_TEMPLATE_NAME "IOTN5 ESP32 DHT11 FAN"
#define BLYNK_AUTH_TOKEN "npjZsmVBGbYBMZv14oLN1tcFthiGB8c"

#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <BlynkSimpleEsp32.h>
#include <WiFiClientSecure.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// -------------------- WIFI CONFIGURATION --------------------
#define WIFI_SSID       "towmiess"
#define WIFI_PASSWORD   "12345678"

// -------------------- DHT SENSOR --------------------
#define DHT_PIN 15
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// -------------------- DEVICE PINS --------------------
#define RELAY_PUMP_PIN 27
#define FAN_PWM_PIN 14
#define FAN_IN1_PIN 12
#define FAN_IN2_PIN 13
#define TRIG_PIN 4
#define ECHO_PIN 5
#define LDR_PIN 34
#define LED1_PIN 16
#define LED2_PIN 17

// -------------------- GLOBAL STATES --------------------
bool pumpState = false, autoPumpControl = false;
bool led1State = false, autoLed1Control = false;
bool led2State = false, autoLed2Control = false;
uint8_t fanSpeed = 0;         // 0 - 3 (level)
bool autoFanControl = false;

// PWM levels tương ứng tốc độ quạt: 0%, 33%, 66%, 100%
const uint8_t fanSpeedLevels[4] = {0, 85, 170, 255};

// -------------------- MQTT CONFIGURATION --------------------
#define AIO_SERVER       "io.adafruit.com"
#define AIO_SERVERPORT   1883
#define AIO_USERNAME     "towmiess"
#define AIO_KEY          "aio_LZTG22O18i5U857MqAVGYNm38MYd"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe fanFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/fan");
Adafruit_MQTT_Subscribe pumpFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pump");
Adafruit_MQTT_Subscribe led1Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/led1");
Adafruit_MQTT_Subscribe led2Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/led2");
Adafruit_MQTT_Subscribe fanSpeedUpFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/fan_speed_up");
Adafruit_MQTT_Subscribe fanSpeedDownFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/fan_speed_down");

// -------------------- SEMAPHORES --------------------
SemaphoreHandle_t wifiMutex, tempMutex, lightMutex, distMutex;

float lastTemperature = 0.0, lastHumidity = 0.0, latestDistance = 0;
int latestLight = 0;

// -------------------- HARDWARE CONTROL FUNCTIONS --------------------

/**
 * Thiết lập tốc độ quạt dựa theo level từ 0 đến 3.
 * Level 0 là tắt quạt, 3 là tốc độ max.
 */
void setFanSpeed(uint8_t speed) {
  if (speed > 3) speed = 3;  // đảm bảo giới hạn
  digitalWrite(FAN_IN1_PIN, speed > 0 ? HIGH : LOW);
  digitalWrite(FAN_IN2_PIN, LOW);
  ledcWrite(0, fanSpeedLevels[speed]);
  fanSpeed = speed;
  Serial.printf("🌀 Quạt tốc độ %d (PWM=%d)\n", speed, fanSpeedLevels[speed]);

}

/**
 * Bật/tắt LED1 và cập nhật trạng thái toàn cục.
 */
void setLed1(bool state) {
  digitalWrite(LED1_PIN, state);
  led1State = state;
  Serial.printf("💡 LED1 %s\n", state ? "BẬT" : "TẮT");
}

/**
 * Bật/tắt LED2 và cập nhật trạng thái toàn cục.
 */
void setLed2(bool state) {
  digitalWrite(LED2_PIN, state);
  led2State = state;
  Serial.printf("💡 LED2 %s\n", state ? "BẬT" : "TẮT");
}

/**
 * Đọc khoảng cách từ cảm biến siêu âm.Trả về khoảng cách (cm), -1 nếu lỗi.
 */
float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return (duration == 0) ? -1 : duration * 0.034 / 2;
}

// -------------------- MQTT SUBSCRIPTION CALLBACK --------------------

/**
 * Xử lý dữ liệu nhận được từ MQTT.
 * Điều khiển thiết bị theo payload.
 */
void mqttCallback(char *data, uint16_t len) {
  // xử lý dữ liệu data có độ dài len
  Serial.print("MQTT message received: ");
  Serial.write(data, len);
  Serial.println();
  // Lấy topic
  String topic = String(data);

  // Lấy payload dạng char*
  String payload = String((char *)data);

  // Xử lý từng topic
  if (topic.endsWith("fan")) {
    if (!autoFanControl) {
      int speed = payload.toInt();
      if (speed >= 0 && speed <= 3) setFanSpeed(speed);
    }
  } else if (topic.endsWith("pump")) {
    pumpState = (payload.toInt() == 1);
    digitalWrite(RELAY_PUMP_PIN, pumpState ? LOW : HIGH);
  } else if (topic.endsWith("led1")) {
    setLed1(payload.toInt() == 1);
  } else if (topic.endsWith("led2")) {
    setLed2(payload.toInt() == 1);
  } else if (topic.endsWith("fan_speed_up")) {
    if (!autoFanControl && fanSpeed < 3) setFanSpeed(fanSpeed + 1);
  } else if (topic.endsWith("fan_speed_down")) {
    if (!autoFanControl && fanSpeed > 0) setFanSpeed(fanSpeed - 1);
  }
}


//

// -------------------- RTOS TASK DEFINITIONS --------------------

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
            // Blynk.virtualWrite(V2, temperature);
            // Blynk.virtualWrite(V3, humidity);
        } else {
            Serial.println("❌ Lỗi cảm biến!");
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}


/**
 * Task đọc dữ liệu cảm biến khoảng cách siêu âm.
 */
void UltrasonicTask(void *pv) {
  while (1) {
    // Đọc khoảng cách từ cảm biến siêu âm
    float dist = readUltrasonicDistance();

    // Nếu khoảng cách hợp lệ (> 0), lưu giá trị vào biến toàn cục
    if (dist > 0) {
      // 🛑 Chiếm quyền truy cập biến `latestDistance` bằng semaphore
      xSemaphoreTake(distMutex, portMAX_DELAY);
      latestDistance = dist; // Lưu giá trị khoảng cách
      xSemaphoreGive(distMutex); // ✅ Trả quyền truy cập biến `latestDistance`
    }

    // Delay để giảm tần suất đọc cảm biến (500ms)
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

/**
 * Task đọc giá trị cảm biến ánh sáng quang trở.
 */
void PhotoresistorTask(void *pv) {
  while (1) {
    int val = analogRead(LDR_PIN);
    xSemaphoreTake(lightMutex, portMAX_DELAY);
    latestLight = val;
    xSemaphoreGive(lightMutex);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

/**
 * Task tự động điều khiển LED1 dựa trên ánh sáng môi trường.
 */
void AutoLed1Task(void *pv) {
  while (1) {
    if (autoLed1Control) {
      xSemaphoreTake(lightMutex, portMAX_DELAY);
      setLed1(latestLight < 1000);
      Serial.println("💡 Tối - Bật đèn");

      xSemaphoreGive(lightMutex);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

/**
 * Task tự động điều khiển LED2 dựa trên khoảng cách.
 */
void AutoLed2Task(void *pv) {
  while (1) {
    if (autoLed2Control) {
      xSemaphoreTake(distMutex, portMAX_DELAY);
      setLed2(latestDistance < 15.0 && latestDistance > 0);
      Serial.println("💡 Vật thể gần - Bật đèn");
      xSemaphoreGive(distMutex);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

/**
 * 8.Task tự động điều khiển máy bơm dựa trên nhiệt độ Nếu nhiệt độ lớn hơn 30°C thì bật bơm, ngược lại tắt bơm.
 */
void AutoPumpTask(void *pv) {
  pinMode(RELAY_PUMP_PIN, OUTPUT);
  digitalWrite(RELAY_PUMP_PIN, HIGH); // mặc định OFF
  while (1) {
    xSemaphoreTake(tempMutex, portMAX_DELAY);
    float t = lastTemperature;
    xSemaphoreGive(tempMutex);
    if (autoPumpControl) {
      if (t > 30 && !pumpState) {
        pumpState = true;
        digitalWrite(RELAY_PUMP_PIN, LOW);
        Blynk.virtualWrite(V7, 1);
      } else if (t <= 30 && pumpState) {
        pumpState = false;
        digitalWrite(RELAY_PUMP_PIN, HIGH);
        Blynk.virtualWrite(V7, 0);
      }
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}
/**
 * Task tự động điều khiển quạt dựa trên nhiệt độ.
 */
void AutoFanTask(void *pv) {
  while (1) {
      // 🛑 Đọc giá trị nhiệt độ từ biến lastTemperature
      // Sử dụng mutex để đảm bảo rằng chỉ một task có thể truy cập biến này tại một thời điểm

    if (autoFanControl) {
      xSemaphoreTake(tempMutex, portMAX_DELAY);
      float temp = lastTemperature;
      xSemaphoreGive(tempMutex);

      if (temp >= 32) setFanSpeed(3);
      else if (temp >= 30) setFanSpeed(2);
      else if (temp >= 25) setFanSpeed(1);
      else setFanSpeed(0);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

/**
 * Task xử lý MQTT và nhận dữ liệu.
 */
// 📡 Kết nối MQTT
// Đã sử dụng Adafruit_MQTT_Client (mqtt) và đã subscribe/callback ở phần setup, không cần hàm connectMQTT riêng.

/**
 * Task xử lý kết nối và nhận dữ liệu từ MQTT.
 * Đảm bảo luôn duy trì kết nối, tự động reconnect nếu mất kết nối.
 */
void MQTTTask(void *pv) {
  mqtt.connect(); // Kết nối MQTT lần đầu
  while (1) {
    mqtt.processPackets(10000); // Xử lý các gói tin MQTT (timeout 10s)
    if (!mqtt.ping()) {         // Kiểm tra kết nối, nếu mất thì reconnect
      mqtt.disconnect();
      mqtt.connect();
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay 500ms trước lần lặp tiếp theo
  }
}

/**
 * Task gửi dữ liệu lên Blynk định kỳ.
 */
void BlynkSendTask(void *pv) {
  while (1) {
    xSemaphoreTake(tempMutex, portMAX_DELAY);
    Blynk.virtualWrite(V10, lastTemperature);
    Serial.println("Nhiệt độ: " + String(lastTemperature) + "°C");

    Blynk.virtualWrite(V11, lastHumidity);
    Serial.println("Độ ẩm: " + String(lastHumidity) + "%");
    xSemaphoreGive(tempMutex);
    
    xSemaphoreTake(distMutex, portMAX_DELAY);
    Blynk.virtualWrite(V12, latestDistance);
    Serial.println("Khoảng cách: " + String(latestDistance) + "cm");
    xSemaphoreGive(distMutex);

    xSemaphoreTake(lightMutex, portMAX_DELAY);
    Blynk.virtualWrite(V13, latestLight);
    Serial.println("Cường độ Ánh sáng trong phòng: " + String(latestLight));
    xSemaphoreGive(lightMutex);

    Blynk.virtualWrite(V14, pumpState);
    Serial.println(pumpState ? "💧 Bật bơm (Thủ công)" : "💧 Tắt bơm (Thủ công)");
    
    Blynk.virtualWrite(V15, fanSpeed);
    Serial.println("Tốc độ quạt: " + String(fanSpeed));
    
    Blynk.virtualWrite(V16, led1State);
    Serial.println("💡 LED1: " + String(led1State));
    
    Blynk.virtualWrite(V17, led2State);
    Serial.println("💡 LED2: " + String(led2State));

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// -------------------- BLYNK WRITE HANDLERS --------------------

/**
 * Xử lý nút bật/tắt chế độ tự động bơm nước (V0).
 */
BLYNK_WRITE(V0) {
  autoPumpControl = param.asInt();
  if (!autoPumpControl) {
    digitalWrite(RELAY_PUMP_PIN, pumpState ? LOW : HIGH);
    Serial.println(pumpState ? "💧 Bật bơm (Thủ công)" : "💧 Tắt bơm (Thủ công)");
  } 
}

/**
 * Xử lý nút bật/tắt bơm nước thủ công (V1).
 */
BLYNK_WRITE(V1) {
  if (!autoPumpControl) {
    pumpState = param.asInt();
    digitalWrite(RELAY_PUMP_PIN, pumpState ? LOW : HIGH);
    Serial.println(pumpState ? "💧 Bật bơm (Thủ công)" : "💧 Tắt bơm (Thủ công)");
  } else {
    Serial.println("💧 Tự động điều khiển bơm đang bật, không thể tắt thủ công");
  }
}

/**
 * Xử lý nút bật/tắt chế độ tự động quạt (V2).
 */
BLYNK_WRITE(V2) {
  autoFanControl = param.asInt();
  if (!autoFanControl) setFanSpeed(0);
}

/**
 * Xử lý thay đổi tốc độ quạt thủ công (V3).
 */
BLYNK_WRITE(V3) {
  if (!autoFanControl) {
    uint8_t speed = param.asInt();

    if (speed >= 0 && speed <= 3){
      setFanSpeed(speed);
      Serial.printf("🌀 Thay đổi tốc độ quạt: %d\n", speed);
    } else {
      Serial.println("❌ Tốc độ quạt không hợp lệ (0-3)");
    }
  }
}

/**
 * Xử lý nút bật/tắt chế độ tự động LED1 (V4).
 */
BLYNK_WRITE(V4) {
  autoLed1Control = param.asInt();
  if (!autoLed1Control) setLed1(false);
}

/**
 * Xử lý nút bật/tắt LED1 thủ công (V5).
 */
BLYNK_WRITE(V5) {
  if (!autoLed1Control) {
    setLed1(param.asInt());
    Serial.println(led1State ? "💡 Bật đèn (Thủ công)" : "💡 Tắt đèn (Thủ công)");
  }
  else {
    Serial.println("💡 Tự động điều khiển LED1 đang bật, không thể tắt thủ công");
  }
}

/**
 * Xử lý nút bật/tắt chế độ tự động LED2 (V6).
 */
BLYNK_WRITE(V6) {
  autoLed2Control = param.asInt();
  if (!autoLed2Control) setLed2(false);
}

/**
 * Xử lý nút bật/tắt LED2 thủ công (V7).
 */
BLYNK_WRITE(V7) {
  if (!autoLed2Control) {
    setLed2(param.asInt());
    Serial.println(led2State ? "💡 Bật đèn (Thủ công)" : "💡 Tắt đèn (Thủ công)");
  } else{
    Serial.println("💡 Tự động điều khiển LED2 đang bật, không thể tắt thủ công");
  }

}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  // Khởi tạo các chân
  pinMode(RELAY_PUMP_PIN, OUTPUT);
  digitalWrite(RELAY_PUMP_PIN, HIGH); // relay thường đóng (OFF)
  
  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(FAN_IN1_PIN, OUTPUT);
  pinMode(FAN_IN2_PIN, OUTPUT);
  setFanSpeed(0);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LDR_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  
  dht.begin();

  // PWM setup cho quạt
  ledcSetup(0, 12000, 8);
  ledcAttachPin(FAN_PWM_PIN, 0);

  // Khởi tạo semaphore
  wifiMutex = xSemaphoreCreateMutex();
  tempMutex = xSemaphoreCreateMutex();
  lightMutex = xSemaphoreCreateMutex();
  distMutex = xSemaphoreCreateMutex();

  // Kết nối WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  // Kết nối Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);

  // Kết nối MQTT và subscribe
  mqtt.subscribe(&fanFeed);
  mqtt.subscribe(&pumpFeed);
  mqtt.subscribe(&led1Feed);
  mqtt.subscribe(&led2Feed);
  mqtt.subscribe(&fanSpeedUpFeed);
  mqtt.subscribe(&fanSpeedDownFeed);

  // Đăng ký callback MQTT
  fanFeed.setCallback(mqttCallback);
  pumpFeed.setCallback(mqttCallback);
  led1Feed.setCallback(mqttCallback);
  led2Feed.setCallback(mqttCallback);
  fanSpeedUpFeed.setCallback(mqttCallback);
  fanSpeedDownFeed.setCallback(mqttCallback);


  // FreeRTOS tasks
  // CORE 0: Xử lý mạng
  xTaskCreatePinnedToCore(WiFiTask,            "WiFiTask",     2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(MQTTTask,            "MQTTTask",     4096, NULL, 3, NULL, 0);

  // CORE 1: Xử lý cảm biến + thiết bị
  xTaskCreatePinnedToCore(DHTTask,             "DHTTask",      4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(UltrasonicTask,  "UltrasonicRd", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(PhotoresistorTask,"PhotoRd",     2048, NULL, 2, NULL, 1);

  xTaskCreatePinnedToCore(AutoLed1Task,        "Led1Ctrl",     2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(AutoLed2Task,        "Led2Ctrl",     2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(AutoFanTask,         "FanCtrl",      2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(AutoPumpTask,        "PumpCtrl",     2048, NULL, 1, NULL, 1);

}

// -------------------- MAIN LOOP --------------------
void loop() {
  // Blynk chạy trong loop
  Blynk.run();
}
