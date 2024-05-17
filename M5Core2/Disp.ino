#include "M5Core2.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid = "csse4011demo";
const char* password = "csse4011";
const char* mqtt_server = "csse4011-iot.zones.eait.uq.edu.au";
const char* mqtt_topic_general = "un46591300";

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (100)
char msg[MSG_BUFFER_SIZE];
float xValue = 1;
float yValue = 1;
float yawValue = 0;
int value = 0;
float linear_velocity = 0;
float ang_velocity = 0;

char receivedMessage[MSG_BUFFER_SIZE];

void setupWifi();
void callback(char* topic, byte* payload, unsigned int length);
void reConnect();

const int gridWidth = 200;
const int gridHeight = 200;
const int gridSize = min(M5.Lcd.width(), M5.Lcd.height()) / 2;
const int gridCellSize = gridSize / 3;

const int minXValue = 0;
const int maxXValue = 2;
const int minYValue = 0;
const int maxYValue = 2;

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

float mapValueToPixel(float value, float minValue, float maxValue, int minPixel, int maxPixel) {
  return mapFloat(value, minValue, maxValue, minPixel, maxPixel);
}

void setup() {
  Serial.begin(115200);
  M5.begin();
  setupWifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.subscribe(mqtt_topic_general);

  Draw_Grid();
}

void loop() {
  long duration, distance;

  if (!client.connected()) {
    reConnect();
  }
  client.loop();

  M5.Lcd.setCursor(220, 10);
  M5.Lcd.printf("Battery Life:");

  M5.Lcd.setCursor(220, 20);
  M5.Lcd.printf("X:");
  M5.Lcd.setCursor(235, 20);
  M5.Lcd.print(2 - yValue);

  M5.Lcd.setCursor(220, 30);
  M5.Lcd.printf("Y:");
  M5.Lcd.setCursor(235, 30);
  M5.Lcd.print(2 - xValue);

  M5.Lcd.setCursor(220, 40);
  M5.Lcd.printf("Yaw:");
  M5.Lcd.setCursor(250, 40);
  M5.Lcd.print(yawValue);

  // Map the xValue and yValue to pixel coordinates
  float pixelX = mapValueToPixel(2 - yValue, minXValue, maxXValue, 10, gridWidth) + 5;
  float pixelY = mapValueToPixel(2 - xValue , minYValue, maxYValue, 10, gridHeight) + 5;
  // Serial.println(pixelX);
  // Serial.println(pixelX);

  // Draw circle with arrow
  M5.Lcd.fillCircle(pixelX, pixelY, 5, TFT_RED);

  Draw_Grid();
}

void Draw_Grid() {
  M5.Lcd.drawFastHLine(10, 10, 200, WHITE);   // Top line
  M5.Lcd.drawFastHLine(10, 210, 200, WHITE);  // Bottom line
  M5.Lcd.drawFastVLine(10, 10, 200, WHITE);   // Left vertical
  M5.Lcd.drawFastVLine(210, 10, 200, WHITE);  // Right vertical

  // Vertical lines
  M5.Lcd.drawFastVLine(60, 10, 200, WHITE);
  M5.Lcd.drawFastVLine(110, 10, 200, WHITE);
  M5.Lcd.drawFastVLine(160, 10, 200, WHITE);

  // Horizontal lines
  M5.Lcd.drawFastHLine(10, 60, 200, WHITE);
  M5.Lcd.drawFastHLine(10, 110, 200, WHITE);
  M5.Lcd.drawFastHLine(10, 160, 200, WHITE);
}

void setupWifi() {
  delay(10);
  M5.Lcd.printf("Connecting to %s", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }
  M5.Lcd.printf("\nSuccess\n");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String payloadStr = "";
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  Serial.println(payloadStr);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payloadStr);

  float x = doc["position"]["x"];
  float y = doc["position"]["y"];
  float yaw = doc["position"]["Yaw"];
  float linear_v = doc["motion"]["linear_velocity"];
  float ang_v = doc["motion"]["angular_velocity"];

  // Convert floating-point coordinates to pixel coordinates
  float pixelX = mapValueToPixel(x, minXValue, maxXValue, 10, gridWidth) + 5;
  float pixelY = mapValueToPixel(y, minYValue, maxYValue, 10, gridHeight) + 5;

  // Clear only the area where the previous circle was drawn
  M5.Lcd.clear();

  xValue = x;
  yValue = y;
  yawValue = yaw;
  linear_velocity = linear_v;
  ang_velocity = ang_v;

}

void reConnect() {
  while (!client.connected()) {
    M5.Lcd.print("Attempting MQTT connection...");
    String clientId = "M5Stack-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      M5.Lcd.printf("\nSuccess\n");
      client.publish(mqtt_topic_general, "hello world");
      client.subscribe(mqtt_topic_general);
    } else {
      M5.Lcd.print("failed, rc=");
      M5.Lcd.print(client.state());
      M5.Lcd.println("try again in 5 seconds");
      delay(5000);
    }
    M5.Lcd.clear();
    Draw_Grid();
  }
}
