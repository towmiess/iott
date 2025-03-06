#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// Chọn model camera
#define CAMERA_MODEL_AI_THINKER
#define Relay 2
#define Light 4  // Điều khiển đèn

#include "camera_pins.h"

// ✅ Cấu hình WiFi
const char* ssid = "towmiess";
const char* password = "12345678";

boolean matchFace = false;
boolean activateRelay = false;
long prevMillis = 0;
int interval = 5000;

// ✅ Hàm khởi tạo camera
void startCameraServer();

// ✅ Cấu hình web server
httpd_handle_t server = NULL;

// ✅ Setup ESP32-CAM
void setup() {
  Serial.begin(115200);
  Serial.println();

  // Cấu hình GPIO cho Relay (khóa cửa)
  pinMode(Relay, OUTPUT);
  digitalWrite(Relay, LOW);

  // Cấu hình LED PWM cho đèn
  ledcSetup(LEDC_CHANNEL_0, 5000, 8);  // 5kHz, độ phân giải 8-bit
  ledcAttachPin(Light, LEDC_CHANNEL_0);
  ledcWrite(LEDC_CHANNEL_0, 0);  // Đèn tắt lúc khởi động

  // ✅ Cấu hình camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // ✅ Khởi động camera
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Lỗi khởi tạo camera!");
    return;
  }

  // ✅ Kết nối WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi đã kết nối!");
  Serial.print("Truy cập Camera tại: http://");
  Serial.println(WiFi.localIP());

  // ✅ Khởi động server
  startCameraServer();
}

// ✅ Hàm gửi ảnh từ camera qua HTTP
esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Không thể lấy ảnh từ camera!");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return ESP_OK;
}

// ✅ Khởi động server

// ✅ Xử lý mở cửa & đèn khi nhận diện khuôn mặt
void loop() {
  if (matchFace == true && activateRelay == false) {
    activateRelay = true;
    digitalWrite(Relay, HIGH);
    ledcWrite(LEDC_CHANNEL_0, 255);  // Đèn sáng nhất
    prevMillis = millis();
  }
  
  if (activateRelay == true && millis() - prevMillis > interval) {
    activateRelay = false;
    matchFace = false;
    digitalWrite(Relay, LOW);
    ledcWrite(LEDC_CHANNEL_0, 0);  // Tắt đèn
  }
}
