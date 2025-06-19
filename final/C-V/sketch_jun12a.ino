#define BLYNK_TEMPLATE_ID "TMPL6xAi8fSD9"
#define BLYNK_TEMPLATE_NAME "MAY LOC KHONG KHI MINI"
#define BLYNK_AUTH_TOKEN "G19IwgpgRz6_87zbELjHFGHidVs0Iq2s"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>
#include <MQ135.h>

// Mobile hotspot
char ssid[] = "towmiess";
char pass[] = "12345678";

// Định nghĩa chân cho ESP8266
#define MQ135_PIN A0    // Cảm biến MQ135
#define BUZZER_PIN D7   // GPIO 13
#define LED_PIN D5      // GPIO 14
#define FAN_PWM_PIN D6  // GPIO 12 (PWM)
#define FAN_IN1 D1      // GPIO 5
#define FAN_IN2 D2      // GPIO 4
#define DHTPIN D4       // GPIO 2
#define DHTTYPE DHT22
#define OZONE_PIN D0    // GPIO 16

// Khởi tạo
DHT dht(DHTPIN, DHTTYPE);
MQ135 gasSensor = MQ135(MQ135_PIN, 90); // R0 = 150
BlynkTimer timer;

// Biến cho điều khiển ozone
unsigned long ozoneStartTime = 0;
bool ozoneActive = false;
bool ozoneCooldown = false;
const unsigned long OZONE_ON_TIME = 10000; // 10 giây
const unsigned long OZONE_OFF_TIME = 300000; // 5 phút

// Biến cho chế độ Auto/Thủ công
bool manualMode = false; // Mặc định là Auto (false)

void setup() {
  Serial.begin(115200);
  
  // Kết nối WiFi
  WiFi.begin(ssid, pass);
  int wifiTimeout = 10; // 10 giây timeout
  while (WiFi.status() != WL_CONNECTED && wifiTimeout > 0) {
    delay(1000);
    Serial.print(".");
    wifiTimeout--;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
  } else {
    Serial.println("Failed to connect to WiFi");
  }
  
  // Khởi tạo Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  // Khởi tạo DHT
  dht.begin();

  // Cấu hình chân
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN_IN1, OUTPUT);
  pinMode(FAN_IN2, OUTPUT);
  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(OZONE_PIN, OUTPUT);

  // Tắt tất cả thiết bị
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(FAN_IN1, LOW);
  digitalWrite(FAN_IN2, LOW);
  analogWrite(FAN_PWM_PIN, 0);
  digitalWrite(OZONE_PIN, LOW);

  // Thiết lập timer để đọc cảm biến và gửi dữ liệu mỗi 2 giây
  timer.setInterval(3000L, readAndSendData);
}

// Chuyển đổi chế độ Auto/Thủ công
BLYNK_WRITE(V6) {
  manualMode = param.asInt(); // V6 = 1: Thủ công, V6 = 0: Auto
  if (!manualMode) {
    digitalWrite(FAN_IN1, LOW);
    digitalWrite(FAN_IN2, LOW);
    analogWrite(FAN_PWM_PIN, 0);
    digitalWrite(OZONE_PIN, LOW);
    ozoneActive = false;
    ozoneCooldown = false;
  }
}

// Điều khiển quạt từ Blynk (chỉ ở chế độ Thủ công)
BLYNK_WRITE(V4) {
  if (manualMode) {
    int fanSpeed = param.asInt(); // 0-255 từ Blynk
    int pwmValue = map(fanSpeed, 0, 255, 0, 1023);
    analogWrite(FAN_PWM_PIN, pwmValue);
    digitalWrite(FAN_IN1, fanSpeed > 0 ? HIGH : LOW);
    digitalWrite(FAN_IN2, LOW);
  }
}

// Điều khiển Ozone từ Blynk (chỉ ở chế độ Thủ công)
BLYNK_WRITE(V5) {
  if (manualMode && param.asInt() && !ozoneActive && !ozoneCooldown) {
    digitalWrite(OZONE_PIN, HIGH);
    ozoneStartTime = millis();
    ozoneActive = true;
  }
}

// Hàm đọc cảm biến, gửi dữ liệu và điều khiển thiết bị
void readAndSendData() {
  // Đọc DHT22
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  // Đọc MQ135
  float ppm = gasSensor.getCorrectedPPM(temp, hum); // ppm đã bù nhiệt độ, độ ẩm

  // Gửi dữ liệu lên Blynk
  Blynk.virtualWrite(V0, ppm);
  Blynk.virtualWrite(V1, temp);
  Blynk.virtualWrite(V2, hum);

  // Xác định trạng thái chỉ dựa trên ppm
  int status = 0; // 0: Tốt, 1: Kém, 2: Nguy hiểm
  if (ppm > 3500) status = 2;
  else if (ppm > 800) status = 1;
  else if (ppm < 500) status = 0;
  Blynk.virtualWrite(V3, status);

  // Cảnh báo
  if (status == 1) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 1000, 500);
  } else if (status == 2) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Nhấp nháy
    tone(BUZZER_PIN, 1000);
  } else {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
  }

  // Điều khiển tự động (chỉ khi ở chế độ Auto)
  if (!manualMode) {
    // Điều khiển quạt tự động
    if (status == 1) {
      digitalWrite(FAN_IN1, HIGH);
      digitalWrite(FAN_IN2, LOW);
      analogWrite(FAN_PWM_PIN, 512); // Tốc độ 50%
    } else if (status == 2) {
      digitalWrite(FAN_IN1, HIGH);
      digitalWrite(FAN_IN2, LOW);
      analogWrite(FAN_PWM_PIN, 1023); // Tốc độ 100%
    } else {
      digitalWrite(FAN_IN1, LOW);
      digitalWrite(FAN_IN2, LOW);
      analogWrite(FAN_PWM_PIN, 0);
    }

    // Điều khiển Ozone tự động
    if (ppm > 3500 && !ozoneActive && !ozoneCooldown) {
      digitalWrite(OZONE_PIN, HIGH);
      ozoneStartTime = millis();
      ozoneActive = true;
    }
  }

  // In dữ liệu ra Serial
  Serial.print("Gas: ");
  Serial.print(ppm);
  Serial.print(" ppm, Temp: ");
  Serial.print(temp);
  Serial.print(" °C, Hum: ");
  Serial.print(hum);
  Serial.println(" %");
}

void loop() {
  if (Blynk.connected()) {
    Blynk.run();
  } else {
    Serial.println("Blynk disconnected");
  }
  timer.run();

  // Quản lý thời gian ozone
  if (ozoneActive && millis() - ozoneStartTime >= OZONE_ON_TIME) {
    digitalWrite(OZONE_PIN, LOW);
    ozoneStartTime = millis();
    ozoneActive = false;
    ozoneCooldown = true;
  }

  if (ozoneCooldown && millis() - ozoneStartTime >= OZONE_OFF_TIME) {
    ozoneCooldown = false; // Kết thúc thời gian nghỉ
  }
}