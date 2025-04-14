#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <uri/UriBraces.h>

#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
#define WIFI_CHANNEL 6

String UrlThingspeak = "https://api.thingspeak.com/update?api_key=XT0PIST17GN42U37&field1=0"
;

WebServer server(80);

const int RELAY1 = 22;
const int RELAY2 = 21;
const int RELAY3 = 19;
const int RELAY4 = 18;

bool relayState[4] = {false, false, false, false};

void sendHtml() {
  String response = R"(
    <!DOCTYPE html><html>
      <head>
        <title>ESP32 Relay Control</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
          html { font-family: sans-serif; text-align: center; }
          body { display: inline-flex; flex-direction: column; }
          h1 { margin-bottom: 1.2em; }
          div { display: grid; grid-template-columns: 1fr 1fr; grid-gap: 1em; }
          .btn { background-color: #5B5; border: none; color: #fff; padding: 0.5em 1em;
                 font-size: 2em; text-decoration: none }
          .btn.OFF { background-color: #333; }
        </style>
      </head>
      <body>
        <h1>ESP32 Relay Control</h1>
        <div>
          <h2>Relay 1</h2>
          <a href="/toggle/1" class="btn RELAY1_TEXT">RELAY1_TEXT</a>
          <h2>Relay 2</h2>
          <a href="/toggle/2" class="btn RELAY2_TEXT">RELAY2_TEXT</a>
          <h2>Relay 3</h2>
          <a href="/toggle/3" class="btn RELAY3_TEXT">RELAY3_TEXT</a>
          <h2>Relay 4</h2>
          <a href="/toggle/4" class="btn RELAY4_TEXT">RELAY4_TEXT</a>
        </div>
      </body>
    </html>
  )";
  response.replace("RELAY1_TEXT", relayState[0] ? "ON" : "OFF");
  response.replace("RELAY2_TEXT", relayState[1] ? "ON" : "OFF");
  response.replace("RELAY3_TEXT", relayState[2] ? "ON" : "OFF");
  response.replace("RELAY4_TEXT", relayState[3] ? "ON" : "OFF");
  server.send(200, "text/html", response);
}

void sendToThingSpeak() {
  String url = UrlThingspeak + "&field1=" + (relayState[0] ? "1" : "0") + 
               "&field2=" + (relayState[1] ? "1" : "0") + 
               "&field3=" + (relayState[2] ? "1" : "0") + 
               "&field4=" + (relayState[3] ? "1" : "0");

  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode > 0) {
    Serial.println("Data sent to ThingSpeak successfully.");
  } else {
    Serial.println("Failed to send data to ThingSpeak.");
  }

  http.end();
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Connecting to WiFi ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", sendHtml);

  server.on(UriBraces("/toggle/{}"), []() {
    int relayNum = server.pathArg(0).toInt();
    if (relayNum >= 1 && relayNum <= 4) {
      Serial.printf("Toggling Relay #%d\n", relayNum);
      relayState[relayNum - 1] = !relayState[relayNum - 1];
      digitalWrite((relayNum == 1) ? RELAY1 : (relayNum == 2) ? RELAY2 : (relayNum == 3) ? RELAY3 : RELAY4, relayState[relayNum - 1]);
      
      sendToThingSpeak();
    }
    sendHtml();
  });

  server.begin();
  Serial.println("HTTP server started (http://localhost:8180)");
}

void loop() {
  server.handleClient();
  delay(2);
}
