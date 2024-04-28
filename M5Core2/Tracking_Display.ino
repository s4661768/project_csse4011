#include "M5Core2.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>  // Include the ArduinoJson library

WiFiClient espClient;
PubSubClient client(espClient);

const int trigPin = 33;  // GPIO pin connected to the Trig pin of the ultrasonic ranger
const int echoPin = 32;  // GPIO pin connected to the Echo pin of the ultrasonic ranger

// const char* ssid = "infrastructure";
// const char* password = "";
// const char* mqtt_server = "csse4011-iot.zones.eait.uq.edu.au";
const char* ssid = "TelstraD74476";
const char* password = "sam9494sam";
const char* mqtt_server = "mqtt.eclipseprojects.io";
const char* mqtt_topic_general = "46591300_General";  // Additional MQTT topic

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (100)  // Increased buffer size for JSON message
char msg[MSG_BUFFER_SIZE];
int xValue = 0;
int yValue = 0;
int value = 0;

char receivedMessage[MSG_BUFFER_SIZE];

void setupWifi();
void callback(char* topic, byte* payload, unsigned int length);
void reConnect();

const int squareSize = min(M5.Lcd.width(), M5.Lcd.height()) / 2;  // Size of the square
const int crossSize = squareSize / 5;                             // Size of the cross lines
const int squareCenterX = M5.Lcd.width() / 2;                     // X-coordinate of the square center
const int squareCenterY = M5.Lcd.height() / 2;                    // Y-coordinate of the square center

// Calculate the coordinates of the top-left corner of the square
int squareTopLeftX = squareCenterX - squareSize / 2;
int squareTopLeftY = squareCenterY - squareSize / 2;
int squareTopRightX = squareCenterX + squareSize / 2;
int squareTopRightY = squareCenterY - squareSize / 2;
int squareBottomLeftX = squareCenterX - squareSize / 2;
int squareBottomLeftY = squareCenterY + squareSize / 2;
int squareBottomRightX = squareCenterX + squareSize / 2;
int squareBottomRightY = squareCenterY + squareSize / 2;

const int minXValue = 0;
const int maxXValue = 2;
const int minYValue = 0;
const int maxYValue = 2;
const int minPixelX = 0;
const int maxPixelX = 240;
const int minPixelY = 0;
const int maxPixelY = 320;

// Function to map input values to pixel coordinates
int mapValueToPixel(int value, int minValue, int maxValue, int minPixel, int maxPixel) {
  // Map the value to the range between minPixel and maxPixel
  return map(value, minValue, maxValue, minPixel, maxPixel);
}

void setup() {
  Serial.begin(115200);
  M5.begin();
  setupWifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.subscribe(mqtt_topic_general);     // Subscribe to the general MQTT topic

  // Draw horizontal cross line
  M5.Lcd.drawLine(squareCenterX - squareSize / 2, squareCenterY, squareCenterX + squareSize / 2, squareCenterY, TFT_WHITE);
  // Draw vertical cross line
  M5.Lcd.drawLine(squareCenterX, squareCenterY - squareSize / 2, squareCenterX, squareCenterY + squareSize / 2, TFT_WHITE);
  M5.Lcd.drawRect(squareTopLeftX, squareTopLeftY, squareSize, squareSize, TFT_WHITE);
  M5.Lcd.drawCircle(squareCenterX, squareCenterY, 5, TFT_RED);

  Serial.println(squareCenterX);
  Serial.println(squareCenterY);
  Serial.println(squareTopLeftX);
  Serial.println(squareTopLeftY);
  Serial.println(squareTopRightX);
  Serial.println(squareTopRightY);
  Serial.println(squareBottomLeftX);
  Serial.println(squareBottomLeftY);
  Serial.println(squareBottomRightX);
  Serial.println(squareBottomRightY);
}

void loop() {
  long duration, distance;

  if (!client.connected()) {
    reConnect();
  }
  client.loop();

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("Battery Life: 50%");

  // Map the xValue and yValue to pixel coordinates
  int pixelX = mapValueToPixel(xValue, minXValue, maxXValue, squareTopLeftX, squareTopRightX);
  int pixelY = mapValueToPixel(yValue, minYValue, maxYValue, squareTopLeftY, squareBottomLeftY);
  Serial.println(pixelX);
  Serial.println(pixelY);


  // Draw horizontal cross line
  M5.Lcd.drawLine(squareCenterX - squareSize / 2, squareCenterY, squareCenterX + squareSize / 2, squareCenterY, TFT_WHITE);
  // Draw vertical cross line
  M5.Lcd.drawLine(squareCenterX, squareCenterY - squareSize / 2, squareCenterX, squareCenterY + squareSize / 2, TFT_WHITE);
  M5.Lcd.drawRect(squareTopLeftX, squareTopLeftY, squareSize, squareSize, TFT_WHITE);
  M5.Lcd.drawCircle(pixelX, pixelY, 5, TFT_RED);
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

  memset(receivedMessage, 0, MSG_BUFFER_SIZE);
  memcpy(receivedMessage, payload, length);
  char* commaPtr = strchr(receivedMessage, ',');
  if (commaPtr != NULL) {
    *commaPtr = '\0';                // Replace comma with null terminator to split the string
    xValue = atoi(receivedMessage);  // Convert the first part to integer
    yValue = atoi(commaPtr + 1);     // Convert the second part to integer
  }
  M5.Lcd.clear();

  // M5.Lcd.print("Message arrived [");
  // M5.Lcd.print(topic);
  // M5.Lcd.print("] ");
  // for (int i = 0; i < length; i++) {
  //   M5.Lcd.print((char)payload[i]);
  // }
  // M5.Lcd.println();
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
      client.subscribe(mqtt_topic_general);  // Subscribe again after reconnection
    } else {
      M5.Lcd.print("failed, rc=");
      M5.Lcd.print(client.state());
      M5.Lcd.println("try again in 5 seconds");
      delay(5000);
    }
  }
}
