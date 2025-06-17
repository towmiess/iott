// ======= IOT + Google Assistant Voice Control with FreeRTOS (ESP32) =======

// -------------------- BLYNK CONFIGURATION --------------------

#define BLYNK_TEMPLATE_ID "TMPL6tmfDJMdC"
#define BLYNK_TEMPLATE_NAME "Smart Home"
#define BLYNK_AUTH_TOKEN "Gv41R0taM-0ucwhaojbc99TZta-tAiL9"

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

// -------------------- DEVICE PINS --------------------
#define DHT_PIN 15
#define RELAY_PUMP_PIN 33
#define FAN_PWM_PIN 14 // ENA điều khiển tốc độ quạt 
#define FAN_IN1_PIN 26 // hướng quay quạt
#define FAN_IN2_PIN 27
#define TRIG_PIN 23
#define ECHO_PIN 22
#define LDR_PIN 34 // CHAN A0
#define LDR_D0_PIN 2 // chân đọc dữ liệu của LDR (quang trở)
#define LED1_PIN 25 // quang trở
#define LED2_PIN 32 // siêu âm 

#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
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
#define AIO_KEY          "aio_MLEH63Gd4ZhxTo7GmIVh43xSXULw"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Subscribe fanFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/speed-fan");
Adafruit_MQTT_Subscribe pumpFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pump");
Adafruit_MQTT_Subscribe led1Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/led1");
Adafruit_MQTT_Subscribe led2Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/led2");
Adafruit_MQTT_Publish distanceFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/distances");
Adafruit_MQTT_Publish lightFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/light");
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");

// -------------------- SEMAPHORES --------------------
SemaphoreHandle_t wifiMutex, tempMutex, lightMutex, distMutex;

float lastTemperature = 0.0, lastHumidity = 0.0, latestDistance = 0;
float prevTemperature = 0.0, prevHumidity = 0.0;
int latestLight = 0;
int lastState = 0; // trạng thái chân D0 của LDR


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

// -------------------- HARDWARE CONTROL FUNCTIONS --------------------

/**
 * Thiết lập tốc độ quạt dựa theo level từ 0 đến 3.
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
long readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);


  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms
  return duration * 0.034 / 2;
}

// -------------------- MQTT SUBSCRIPTION CALLBACK --------------------

/**
 * Xử lý dữ liệu nhận được từ MQTT. Điều khiển thiết bị theo payload. **/
void fanCallback(char *data, uint16_t len) {
  int speed = String(data).toInt();
  Serial.println("🌀 Nhận tốc độ quạt: " + String(speed));

  if (!autoFanControl) {
    if (speed >= 0 && speed <= 3) {
      setFanSpeed(speed);  // Hàm do bạn định nghĩa sẵn
      digitalWrite(FAN_IN1_PIN, speed > 0 ? HIGH : LOW);
      digitalWrite(FAN_IN2_PIN, LOW);
      fanSpeed = speed;
      Blynk.virtualWrite(V5, fanSpeed);
    } else {
      Serial.println("❌ Tốc độ quạt không hợp lệ (0-3)");
    }
  } else {
    Serial.println("⛔ Đang ở chế độ Auto Fan");
  }
}

void pumpCallback(char *data, uint16_t len) {
  int value = String(data).toInt();
  Serial.println("💧 Nhận lệnh bơm: " + String(value));

  if (!autoPumpControl) {
    if (value == 1) {
      pumpState = true;
      digitalWrite(RELAY_PUMP_PIN, LOW);
    } else if (value == 0) {
      pumpState = false;
      digitalWrite(RELAY_PUMP_PIN, HIGH);
    } else {
      Serial.println("❌ Lệnh không hợp lệ cho bơm");
    }
    Blynk.virtualWrite(V8, pumpState);
  } else {
    Serial.println("⛔ Đang ở chế độ Auto Pump");
  }
}

void led1Callback(char *data, uint16_t len) {
  int value = String(data).toInt();
  Serial.println("💡 Nhận lệnh LED1: " + String(value));

  if (!autoLed1Control) {
    if (value == 1) {
      led1State = true;
      digitalWrite(LED1_PIN, LOW);
    } else if (value == 0) {
      led1State = false;
      digitalWrite(LED1_PIN, HIGH);
    } else {
      Serial.println("❌ Lệnh LED1 không hợp lệ");
    }
    Blynk.virtualWrite(V6, led1State);
  } else {
    Serial.println("⛔ Đang ở chế độ Auto LED1");
  }
}

void led2Callback(char *data, uint16_t len) {
  int value = String(data).toInt();
  Serial.println("💡 Nhận lệnh LED2: " + String(value));

  if (!autoLed2Control) {
    if (value == 1) {
      led2State = true;
      digitalWrite(LED2_PIN, LOW);
    } else if (value == 0) {
      led2State = false;
      digitalWrite(LED2_PIN, HIGH);
    } else {
      Serial.println("❌ Lệnh LED2 không hợp lệ");
    }
    Blynk.virtualWrite(V7, led2State);
  } else {
    Serial.println("⛔ Đang ở chế độ Auto LED2");
  }
}

/**
 * Task xử lý kết nối và nhận dữ liệu từ MQTT.
 * Đảm bảo luôn duy trì kết nối, tự động reconnect nếu mất kết nối.
 */

void MQTTTask(void *pvParameters) {
  while (true) {
    if (!mqtt.connected()) {
      Serial.println("MQTT không kết nối, thử lại...");
      mqtt.connect();
      if (!mqtt.connected()) {
        Serial.println("Kết nối MQTT thất bại!");
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Chờ 5 giây trước khi thử lại
        continue;
      }
    }
    mqtt.processPackets(100); // Xử lý MQTT
    vTaskDelay(100 / portTICK_PERIOD_MS); // Chờ 100ms
  }
}



// -------------------- RTOS TASK DEFINITIONS --------------------


// 📡 Đọc cảm biến DHT11
void DHTTask(void *pvParameters) {
  while (true) {
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (!isnan(temperature) && !isnan(humidity)) {
      // 🛑 Giữ quyền truy cập dữ liệu nhiệt độ
      xSemaphoreTake(tempMutex, portMAX_DELAY); // Chiếm quyền truy cập biến nhiệt độ
      lastTemperature = temperature; // Lưu nhiệt độ đọc được
      lastHumidity = humidity; // Lưu độ ẩm đọc được
      xSemaphoreGive(tempMutex); // ✅ Trả quyền truy cập dữ liệu nhiệt độ

      Serial.printf("🌡 Nhiệt độ: %.1f°C, Độ ẩm: %.1f%%\n", temperature, humidity);
      Blynk.virtualWrite(V9, temperature);
      Blynk.virtualWrite(V10, humidity);

      // Gửi dữ liệu lên Adafruit IO
      if (abs(temperature - prevTemperature) >= 0.5 || abs(humidity - prevHumidity) >= 0.5) { // Thay đổi 0.5 độ/phần trăm
      Adafruit_MQTT_Publish temperaturePub(&mqtt, AIO_USERNAME "/feeds/temperature");
      Adafruit_MQTT_Publish humidityPub(&mqtt, AIO_USERNAME "/feeds/humidity");

      if (!temperaturePub.publish(temperature)) {
        Serial.println("❌ Lỗi gửi nhiệt độ lên Adafruit IO!");
      }
      if (!humidityPub.publish(humidity)) {
        Serial.println("❌ Lỗi gửi độ ẩm lên Adafruit IO!");
      }
      prevTemperature = temperature;
      prevHumidity = humidity;
      }
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
    
    Serial.printf("📏 Khoảng cách: %.2f cm\n", dist);
    Blynk.virtualWrite(V12, dist); // Gửi dữ liệu khoảng cách lên Blynk
    
    // Gửi dữ liệu khoảng cách lên Adafruit IO
    static float lastSentDistance = 0;
    if (fabs(dist - lastSentDistance) >= 0.5) {  // Chỉ gửi khi khoảng cách thay đổi vượt quá 0.5 cm
      Adafruit_MQTT_Publish distancePub(&mqtt, AIO_USERNAME "/feeds/distance");
      if (!distancePub.publish(dist)) {
        Serial.println("❌ Lỗi gửi khoảng cách lên Adafruit IO!");
      } else {
        lastSentDistance = dist;
      }
    }
    // Delay để giảm tần suất đọc cảm biến (10000ms)
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

/**
 * Task đọc giá trị cảm biến ánh sáng quang trở.
 */
void PhotoresistorTask(void *pv) {
  while (1) {
    int lightValue = analogRead(LDR_PIN); // đọc giá trị từ chân A0 (LDR) càng tối giá trị càng cao 
    int lightState = digitalRead(LDR_D0_PIN); // đọc chân D0 của LDR
    // Cập nhật biến toàn cục
    xSemaphoreTake(lightMutex, portMAX_DELAY);
    latestLight = lightValue;
    lastState = lightState; // cập nhật trạng thái chân D0
    xSemaphoreGive(lightMutex);

    // Nếu chân D0 là HIGH : tối, nếu là LOW : sáng
    if (lightValue >= 450) {
      Serial.println("💡 Tối");
    } if (lightValue < 450) {
      Serial.println("💡Sáng");
    }
    Serial.printf("💡 Cường độ tối: %d\n", lightValue);
    Blynk.virtualWrite(V11, lightValue);

    // Gửi dữ liệu ánh sáng lên Adafruit IO
    static int lastSentLight = -1;
    if (abs(lightValue - lastSentLight) > 100) { // chỉ gửi khi thay đổi lớn hơn 100 đơn vị
      Adafruit_MQTT_Publish lightPub(&mqtt, AIO_USERNAME "/feeds/light");
      if (!lightPub.publish(lightValue)) {
        Serial.println("❌ Lỗi gửi ánh sáng lên Adafruit IO!");
      } else {
        lastSentLight = lightValue;
      }
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

/**
 * Task tự động điều khiển LED1 dựa trên ánh sáng môi trường.
 */
void AutoLed1Task(void *pv) {
  while (1) {
    xSemaphoreTake(lightMutex, portMAX_DELAY);
    bool isDark = latestLight >= 450; // Nếu chân D0 là LOW và giá trị ánh sáng >= 600 thì coi là tối
    xSemaphoreGive(lightMutex);
    if (autoLed1Control) {
      if (isDark) {
        led1State = true;
        digitalWrite(LED1_PIN, LOW); // Bật LED1 (LOW = ON)
        Blynk.virtualWrite(V6, 1); // Đồng bộ trạng thái với thủ công
        Serial.println("💡 Tối - Bật đèn (auto)");
      } else {
        led1State = false;
        digitalWrite(LED1_PIN, HIGH); // Tắt LED1 (HIGH = OFF)
        Blynk.virtualWrite(V6, 0);
        Serial.println("💡 Sáng - Tắt đèn (auto)");
      }
      Blynk.virtualWrite(V1, 1); // Cập nhật chế độ auto bật
      Serial.println("⚙️ Chế độ tự động LED1: BẬT");
    } else {
      Blynk.virtualWrite(V1, 0); // Cập nhật chế độ auto tắt
      Serial.println("⚙️ Chế độ tự động LED1: TẮT");
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

/**
 * Task tự động điều khiển LED2 dựa trên khoảng cách.
 */
void AutoLed2Task(void *pv) {
  while (1) {
    if (autoLed2Control) {
      xSemaphoreTake(distMutex, portMAX_DELAY);
      float dist = latestDistance;
      xSemaphoreGive(distMutex);

      if (dist > 0 && dist <= 15.0) {
        led2State = true;
        digitalWrite(LED2_PIN, LOW); // Bật LED2
        Serial.println("💡 Vật thể gần - Bật LED2 (auto)");
      } else {
        led2State = false;
        digitalWrite(LED2_PIN, HIGH); // Tắt LED2
        Serial.println("💡 Không có vật thể gần - Tắt LED2 (auto)");
      }
      Blynk.virtualWrite(V7, led2State); // Cập nhật trạng thái LED2 trên Blynk
      Blynk.virtualWrite(V2, 1); // Cập nhật chế độ auto bật
      Serial.println("⚙️ Chế độ tự động LED2: BẬT");
    } else {
      Blynk.virtualWrite(V2, 0); // Cập nhật chế độ auto tắt
      Serial.println("⚙️ Chế độ tự động LED2: TẮT");
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

/**
 * Task tự động điều khiển máy bơm dựa trên nhiệt độ Nếu nhiệt độ lớn hơn 30°C thì bật bơm, ngược lại tắt bơm.
 */
void AutoPumpTask(void *pv) {
  while (1) {
    xSemaphoreTake(tempMutex, portMAX_DELAY);
    float t = lastTemperature;
    xSemaphoreGive(tempMutex);
    if (autoPumpControl) {
      if (t >= 25 && !pumpState) {
        pumpState = true;
        digitalWrite(RELAY_PUMP_PIN, LOW);
        Blynk.virtualWrite(V8, 1); // Cập nhật đồng bộ với thủ công
        Serial.printf("💧 Nhiệt độ cao >=25 - bật làm mát không khí (auto) %.1f°C\n", t);
      } else if (t < 25 && pumpState) {
        pumpState = false;
        digitalWrite(RELAY_PUMP_PIN, HIGH);// high là tắt relay
        Blynk.virtualWrite(V8, 0);
        Serial.printf("💧 Nhiệt độ thấp < 25 -  tắt làm mát không khí (auto) %.1f°C\n", t);
      }
      Blynk.virtualWrite(V3, autoPumpControl ? 1 : 0); // Cập nhật chế độ auto
    }
    Serial.printf("⚙️ Chế độ làm mát tự động: %s\n", autoPumpControl ? "BẬT" : "TẮT");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
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
      if (temp >= 25) {
        setFanSpeed(3);
        digitalWrite (FAN_IN1_PIN, HIGH);
        digitalWrite (FAN_IN2_PIN, LOW);
        Blynk.virtualWrite(V5, 3);
        Blynk.virtualWrite(V4, 1); // Cập nhật chế độ auto bật
        Serial.println("🌡 Nhiệt độ cao - Bật quạt tốc độ 3(auto)");

      } else if (temp >= 20 && temp < 25) {
        setFanSpeed(2);
        digitalWrite (FAN_IN1_PIN, HIGH);
        digitalWrite (FAN_IN2_PIN, LOW);
        Blynk.virtualWrite(V5, 2);
        Serial.println("🌡 Nhiệt độ cao - Bật quạt tốc độ 2(auto)");

      } else {
        autoFanControl = false;
        setFanSpeed(0);
        digitalWrite(FAN_IN1_PIN, LOW);
        digitalWrite(FAN_IN2_PIN, LOW);
        Blynk.virtualWrite(V5, 0);// dong bo voi thu cong 
        Serial.println("🌡 Nhiệt độ thấp - Tắt quạt(auto)");
      }
    }
    Blynk.virtualWrite(V4, autoFanControl ? 1 : 0); // Cập nhật chế độ auto
    Serial.printf("⚙️ Chế độ quạt tự động: %s\n", autoFanControl ? "BẬT" : "TẮT");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}


// -------------------- BLYNK WRITE HANDLERS --------------------

/**
 * Xử lý nút bật/tắt chế độ tự động bơm nước (V3).
 * 🛠️ Nhận dữ liệu từ Blynk để bật/tắt chế độ bơm nước tự động
 */
BLYNK_WRITE(V3) {
  autoPumpControl = param.asInt();
  pumpState = param.asInt(); // Lưu trạng thái mới của bơm
  Serial.printf("⚙️ Chế độ làm mát tự động: %s\n", autoPumpControl ? "BẬT" : "TẮT");
}

/**
 * Xử lý nút bật/tắt bơm nước thủ công (V8).
 */
BLYNK_WRITE(V8) {
  if (!autoPumpControl) {
    pumpState = param.asInt();
    digitalWrite(RELAY_PUMP_PIN, pumpState ? LOW : HIGH);
    Serial.println(pumpState ? "💧 Bật bơm (Thủ công)" : "💧 Tắt bơm (Thủ công)");
  } else {
    Serial.println("💧 Tự động điều khiển bơm đang bật, không thể tắt thủ công");
    // Cập nhật lại trạng thái bơm trên Blynk
    Blynk.virtualWrite(V8, pumpState);
  }
}

/**
 * Xử lý nút bật/tắt chế độ tự động quạt (V4).
 */
BLYNK_WRITE(V4) {
  autoFanControl = param.asInt();
  Serial.printf("⚙️ Chế độ quạt tự động: %s\n", autoFanControl ? "BẬT" : "TẮT");
}

/**
 * Xử lý thay đổi tốc độ quạt thủ công (V5).
 */

BLYNK_WRITE(V5) {
  if (!autoFanControl) {
    uint8_t speed = param.asInt();
    if (speed >= 0 && speed <= 3){
      setFanSpeed(speed);
      Serial.printf("🌀 Thay đổi tốc độ quạt: %d\n", speed);
    } else {
      Serial.println("❌ Tốc độ quạt không hợp lệ (0-3)");
      Blynk.virtualWrite(V5, fanSpeed); // Cập nhật lại tốc độ quạt trên Blynk
    }
  }
}

/** Xử lý nút bật/tắt chế độ tự động LED1 (V1). */
BLYNK_WRITE(V1) {
  autoLed1Control = param.asInt();
  led1State = param.asInt();
  Serial.printf("⚙️ Chế độ LED1 tự động: %s\n", autoLed1Control ? "BẬT" : "TẮT");
}

/** Xử lý nút bật/tắt LED1 thủ công (V6). */
BLYNK_WRITE(V6) {
  if (!autoLed1Control) {
    setLed1(param.asInt());
    digitalWrite(LED1_PIN, led1State ? LOW : HIGH);
    Serial.println(led1State ? "💡 Bật đèn (Thủ công)" : "💡 Tắt đèn (Thủ công)");
  }
  else {
    Serial.println("💡 Tự động điều khiển LED1 đang bật, không thể tắt thủ công");
    Blynk.virtualWrite(V6, led1State); // Cập nhật lại trạng thái LED1
  }
}

/**
 * Xử lý nút bật/tắt chế độ tự động LED2 (V2).
 */
BLYNK_WRITE(V2) {
  autoLed2Control = param.asInt();
  Serial.printf("⚙️ Chế độ LED2 tự động: %s\n", autoLed2Control ? "BẬT" : "TẮT");
}

/**
 * Xử lý nút bật/tắt LED2 thủ công (V7).
 */
BLYNK_WRITE(V7) {
  if (!autoLed2Control) {
    setLed2(param.asInt());
    digitalWrite(LED2_PIN, led2State ? LOW : HIGH);
    Serial.println(led2State ? "💡 Bật đèn (Thủ công)" : "💡 Tắt đèn (Thủ công)");
  } else{
    Serial.println("💡 Tự động điều khiển LED2 đang bật, không thể tắt thủ công");
    Blynk.virtualWrite(V7, led2State); // Cập nhật lại trạng thái LED2
  }

}

// -------------------- SETUP --------------------
void setup() {
  delay(5000); // Chờ 5 giây trước khi khởi động
  Serial.begin(115200);

  // Khởi tạo các chân
  pinMode(RELAY_PUMP_PIN, OUTPUT);
  digitalWrite(RELAY_PUMP_PIN, HIGH); // relay thường đóng (OFF)
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);

  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(FAN_IN1_PIN, OUTPUT);
  pinMode(FAN_IN2_PIN, OUTPUT);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LDR_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  
  dht.begin();

  // PWM setup cho quạt
  ledcSetup(0, 12000, 8);
  ledcAttachPin(FAN_PWM_PIN, 0);
  setFanSpeed(0);

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
  // mqtt.subscribe(&distanceFeed);
  // mqtt.subscribe(&lightFeed);
  // mqtt.subscribe(&temperatureFeed);

  // Đăng ký callback MQTT
  fanFeed.setCallback(fanCallback);
  pumpFeed.setCallback(pumpCallback);
  led1Feed.setCallback(led1Callback);
  led2Feed.setCallback(led2Callback);


  // FreeRTOS tasks
  // CORE 0: Kết nối WiFi, MQTT và đọc cảm biến ánh sáng, siêu âm
  xTaskCreatePinnedToCore(WiFiTask,            "WiFiTask",     2048, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(MQTTTask,            "MQTTTask",     4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(UltrasonicTask,      "UltrasonicRd", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(PhotoresistorTask,   "PhotoRd",      4096, NULL, 2, NULL, 0);

  // CORE 1: Xử lý cảm biến + thiết bị
  xTaskCreatePinnedToCore(DHTTask,             "DHTTask",      4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(AutoLed1Task,        "Led1Ctrl",     4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(AutoLed2Task,        "Led2Ctrl",     4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(AutoFanTask,         "FanCtrl",      4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(AutoPumpTask,        "PumpCtrl",     4096, NULL, 1, NULL, 1);


}

// -------------------- MAIN LOOP --------------------
void loop() {
  Blynk.run();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
  }

}
