#define BLYNK_TEMPLATE_ID "TMPL6ejGwqClU"
#define BLYNK_TEMPLATE_NAME "SmartHome"
#define BLYNK_AUTH_TOKEN "L3rIUzp_eUawrCW8dd0z_YKds8hV7NMR"
#define VIRTUAL_PIN V0

#include <HTTPClient.h>
#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "base64.h"
#include <BlynkSimpleEsp32.h>

const char* auth = BLYNK_AUTH_TOKEN;
const char* ssid = "towmiess";
const char* password = "12345678";

BlynkTimer timer;
bool doorState = false;

#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Base64.h"

#define AIO_USERNAME    ""
#define AIO_KEY         ""
#define FEED_NAME       "captrue"

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

using namespace websockets;
WebsocketsServer socket_server;

camera_fb_t * fb = NULL;

long current_millis;
long last_detected_millis = 0;

#define relay_pin 15 // điều khiển relay

unsigned long door_opened_millis = 0;
long interval = 5000;           // mở cửa sau 5s
bool face_recognised = false;

void app_facenet_main();// khởi tạo nhận diện khuôn mặt
void app_httpserver_init();// khởi động http server

typedef struct
{
  uint8_t *image; //dữ liệu ảnh chụp được
  box_array_t *net_boxes; //danh sách khuôn mặt đã phát hiện
  dl_matrix3d_t *face_id; //đặc trưng để nhận diện
} http_img_process_result;

//hàm cấu hình bộ phát hiện khuôn mặt (face detection)
static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

httpd_handle_t camera_httpd = NULL;

//định nghĩa trạng thái hoạt động của hệ thống
typedef enum
{
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state;

//lưu tên khuôn mặt
typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  digitalWrite(relay_pin, HIGH);
  pinMode(relay_pin, OUTPUT);

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

  //cấu hình khung hình camera
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  app_httpserver_init();//Khởi tạo máy chủ HTTP.
  app_facenet_main();//Bắt đầu quá trình nhận diện khuôn mặt.
  socket_server.listen(82);//Lắng nghe kết nối client trên cổng 82 qua socket.

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  //khởi tạo task điều khiển trạng thái cửa trên blynk
  xTaskCreatePinnedToCore(
    blynkTask,
    "BlynkTask",
    4096,
    NULL,
    1,
    NULL,
    1
  );

}

//hàm trả về một trang HTML đã được nén bằng gzip khi có yêu cầu HTTP
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};

//khởi tạo và cấu hình máy chủ HTTP
void app_httpserver_init ()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}

void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);//Khởi tạo danh sách khuôn mặt
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);//Cấp phát bộ nhớ cho ảnh khuôn mặt
  read_face_id_from_flash_with_name(&st_face_list);//Đọc thông tin khuôn mặt đã lưu
}


//thực hiện quá trình đăng ký khuôn mặt
static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

//đồng bộ danh sách khuôn mặt đã đăng ký giữa ESP32 và trình duyệt
static esp_err_t send_face_list(WebsocketsClient &client)
{
  client.send("delete_faces"); // tell browser to delete all faces
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) // loop current faces
  {
    sprintf(add_face, "listface:%s", head->id_name);
    Serial.println(add_face);
    client.send(add_face); //send face to browser
    head = head->next;
  }
  return ESP_OK;
}

//xóa khuôn mặt
static esp_err_t delete_all_faces(WebsocketsClient &client)
{
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
  return ESP_OK;
}

//điều khiển trên blynk
BLYNK_WRITE(V0) {
  int value = param.asInt(); // 1 hoặc 0

  if (value == 1) {
    Serial.println("Mở cửa từ Blynk");
    digitalWrite(relay_pin, LOW);
    door_opened_millis = millis();
    timer.setTimeout(5000L, []() {
      digitalWrite(relay_pin, HIGH);
      Serial.println("Cửa đã đóng.");

      // Gửi lại trạng thái OFF về Blynk
      Blynk.virtualWrite(VIRTUAL_PIN, 0);
    });
    doorState = true;
  } else {
    Serial.println("Đóng cửa từ Blynk");
    digitalWrite(relay_pin, HIGH);
    doorState = false;
  }
}

void blynkTask(void * parameter) {
  Blynk.begin(auth, ssid, password);

  while (true) {
    Blynk.run();    // xử lý sự kiện
    timer.run();    // chạy timer nếu cần
    vTaskDelay(10 / portTICK_PERIOD_MS);  // delay nhỏ tránh chiếm CPU
  }
}


void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{
  if (msg.data() == "stream") {
    g_state = START_STREAM;
    client.send("STREAMING");
  }
  if (msg.data() == "detect") {
    g_state = START_DETECT;
    client.send("DETECTING");
  }
  //ghi nhận khuôn mặt mới
  if (msg.data().substring(0, 8) == "capture:") {
    g_state = START_ENROLL;
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("CAPTURING");
  }
  if (msg.data() == "recognise") {
    g_state = START_RECOGNITION;
    client.send("RECOGNISING");
  }
  if (msg.data().substring(0, 7) == "remove:") {
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client); // reset faces in the browser
  }
  if (msg.data() == "open_door") {
    digitalWrite(relay_pin, LOW); //close (energise) relay so door unlocks
    door_opened_millis = millis();
    client.send("Door Opened!");
  }
  if (msg.data() == "close_door") {
    digitalWrite(relay_pin, HIGH); //close (energise) relay so door unlocks
    client.send("Door Closed!");
  }
  if (msg.data() == "delete_all") {
    delete_all_faces(client);
  }
}

void open_door(WebsocketsClient &client) {
  if (digitalRead(relay_pin) == HIGH) {
    digitalWrite(relay_pin, LOW); //close (energise) relay so door unlocks
    Serial.println("Door Unlocked");
    client.send("door_open");
    door_opened_millis = millis(); // time relay closed and door opened
  }
}

void close_door() {
  digitalWrite(relay_pin, HIGH); // open relay (lock door)
}


void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi mất kết nối. Đang thử lại...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    delay(1000);
  }
  auto client = socket_server.accept();
  client.onMessage(handle_message);
  //Khởi tạo bộ nhớ ảnh, gửi danh sách khuôn mặt
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  http_img_process_result out_res = {0};
  out_res.image = image_matrix->item;
  send_face_list(client);
  client.send("STREAMING");//gửi trạng thái

  //Lặp khi client vẫn còn kết nối.
  while (client.available()) {
    client.poll();

    if (millis() - interval > door_opened_millis) {
      close_door(); // close the door (lock)
    }

    fb = esp_camera_fb_get();

    if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
    {
      out_res.net_boxes = NULL;
      out_res.face_id = NULL;

      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

      out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

      if (out_res.net_boxes)
      {
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK)
        {

          out_res.face_id = get_face_id(aligned_face);
          last_detected_millis = millis();
          if (g_state == START_DETECT) {
            client.send("FACE DETECTED");
          }

          if (g_state == START_ENROLL)
          {
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
            char enrolling_message[64];
            sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            client.send(enrolling_message);
            if (left_sample_face == 0)
            {
              ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
              g_state = START_STREAM;
              char captured_message[64];
              sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
              client.send(captured_message);
              send_face_list(client);

            }
          }

          if (g_state == START_RECOGNITION  && (st_face_list.count > 0))
          {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
            if (f)
            {
              char recognised_message[64];
              sprintf(recognised_message, "DOOR OPEN FOR %s", f->id_name);
              open_door(client);
              client.send(recognised_message);

              String base64Photo = Photo2Base64();

              // Gửi ảnh lên Adafruit
              sendToAdafruit(base64Photo);

              // Gửi ảnh + thông báo đến server
              sendImageToServer(fb->buf, fb->len, "Chào mừng " + String(f->id_name) + " trở về");
            }
            else
            {
              client.send("FACE NOT RECOGNISED");
              String base64Photo = Photo2Base64();

              // Gửi ảnh lên Adafruit
              sendToAdafruit(base64Photo);
              sendImageToServer(fb->buf, fb->len, "Cảnh báo: Có người lạ mở cửa!");
            }
          }
          dl_matrix3d_free(out_res.face_id);
        }

      }
      else
      {
        if (g_state != START_DETECT) {
          client.send("NO FACE DETECTED");
        }
      }

      if (g_state == START_DETECT && millis() - last_detected_millis > 500) { // Detecting but no face detected
        client.send("DETECTING");
      }

    }

    client.sendBinary((const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    fb = NULL;
  }
}


String Photo2Base64() {
  //chụp ảnh từ cam
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return "";
  }
  //chuyển về dạng base64
  String imageFile = "data:image/jpeg;base64,";
  char *input = (char *)fb->buf;
  char output[base64_enc_len(3)];
  for (int i = 0; i < fb->len; i++) {
    base64_encode(output, (input++), 3);
    if (i % 3 == 0) imageFile += urlencode(String(output));
  }

  esp_camera_fb_return(fb);

  return imageFile;
}
//hàm gửi ảnh lên Adafruit
void sendToAdafruit(String base64Image) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "https://io.adafruit.com/api/v2/" + String(AIO_USERNAME) + "/feeds/" + FEED_NAME + "/data";

    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-AIO-Key", AIO_KEY);

    String payload = "{\"value\":\"" + base64Image + "\"}";

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Adafruit IO Response: " + response);
    } else {
      Serial.printf("Error sending to Adafruit, code: %d\n", httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi Disconnected. Cannot send to Adafruit.");
  }
}

//Mã hóa url
String urlencode(String str) {
  const char *msg = str.c_str();
  const char *hex = "0123456789ABCDEF";
  String encodedMsg = "";
  while (*msg != '\0') {
    if (('a' <= *msg && *msg <= 'z') || ('A' <= *msg && *msg <= 'Z') || ('0' <= *msg && *msg <= '9') || *msg == '-' || *msg == '_' || *msg == '.' || *msg == '~') {
      encodedMsg += *msg;
    } else {
      encodedMsg += '%';
      encodedMsg += hex[(unsigned char) * msg >> 4];
      encodedMsg += hex[*msg & 0xf];
    }
    msg++;
  }
  return encodedMsg;
}

//Hàm gửi ảnh lên server
void sendImageToServer(uint8_t *imageData, size_t len, const String &message) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String serverUrl = "https://esp32cam-control.onrender.com/send";
    http.begin(serverUrl);

    String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
    String contentType = "multipart/form-data; boundary=" + boundary;
    http.addHeader("Content-Type", contentType);

    // Tạo phần đầu và cuối của multipart`
    String bodyStart = "--" + boundary + "\r\n";
    bodyStart += "Content-Disposition: form-data; name=\"image\"; filename=\"capture.jpg\"\r\n";
    bodyStart += "Content-Type: image/jpeg\r\n\r\n";

    String bodyMiddle = "\r\n--" + boundary + "\r\n";
    bodyMiddle += "Content-Disposition: form-data; name=\"message\"\r\n\r\n";
    bodyMiddle += message + "\r\n";

    String bodyEnd = "--" + boundary + "--\r\n";

    // Tính tổng độ dài
    size_t totalLen = bodyStart.length() + len + bodyMiddle.length() + bodyEnd.length();

    // Cấp phát buffer để gửi
    uint8_t *postData = (uint8_t *)malloc(totalLen);
    if (!postData) {
      Serial.println("Failed to allocate memory for POST");
      return;
    }

    // Copy dữ liệu
    size_t idx = 0;
    memcpy(postData + idx, bodyStart.c_str(), bodyStart.length());
    idx += bodyStart.length();

    memcpy(postData + idx, imageData, len);
    idx += len;

    memcpy(postData + idx, bodyMiddle.c_str(), bodyMiddle.length());
    idx += bodyMiddle.length();

    memcpy(postData + idx, bodyEnd.c_str(), bodyEnd.length());
    idx += bodyEnd.length();

    // Gửi yêu cầu
    int httpResponseCode = http.sendRequest("POST", postData, totalLen);
    free(postData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Server response: " + response);
    } else {
      Serial.printf("Failed to send image. HTTP error code: %d\n", httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi disconnected. Cannot send image.");
  }
}
