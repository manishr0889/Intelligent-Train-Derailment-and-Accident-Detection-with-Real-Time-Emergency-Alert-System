#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "MPU6050.h"

MPU6050 mpu;
float roll = 0;

#define ALERT_ANGLE 60  // Threshold angle for alert (normal alert)
#define MAX_ALERT_ANGLE 200  // Threshold angle for maximum alert
#define LED_PIN 2  // LED connected to pin 2
#define BUZZER_PIN 15  // Buzzer connected to pin 15

uint8_t peerAddress[] = {0x1C, 0x69, 0x20, 0x93, 0xBC, 0xC0};  // Replace with the peer ESP32 MAC address

typedef struct struct_message {
  bool flipped;  // If the roll angle exceeds threshold
  bool maxAlert;  // If the roll angle exceeds max threshold (200°)
} struct_message;

struct_message outgoingData;
struct_message incomingData;

bool alertSent = false;

// Corrected callback function signature
void onDataReceive(const esp_now_recv_info* info, const uint8_t* incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));

  if (incomingData.maxAlert) {
    Serial.println("⚠ MAX ALERT received: Exceeds 200°");
    digitalWrite(LED_PIN, HIGH);  // Turn on LED for max alert
    digitalWrite(BUZZER_PIN, HIGH);  // Turn on buzzer for max alert
  } else if (incomingData.flipped) {
    Serial.println("⚠ ALERT received: Flipped!");
    digitalWrite(LED_PIN, HIGH);  // Turn on LED for normal alert
    digitalWrite(BUZZER_PIN, HIGH);  // Turn on buzzer for normal alert
  } else {
    Serial.println("Normal State");
    digitalWrite(LED_PIN, LOW);  // Turn off LED
    digitalWrite(BUZZER_PIN, LOW);  // Turn off buzzer
  }
}

void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  // Register callback functions for send and receive
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceive);

  // Set up peer for communication
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 initialized");
}

void loop() {
  // Read raw values from the MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate roll angle in degrees
  roll = atan2(ay, az) * 180 / PI;
  Serial.print("Roll Angle: ");
  Serial.println(roll);

  // Check if roll exceeds the threshold (ALERT)
  if (abs(roll) > MAX_ALERT_ANGLE && !alertSent) {
    outgoingData.maxAlert = true;  // Max alert: Exceeds 200°
    outgoingData.flipped = false;  // Reset flipped state
    esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));  // Send max alert to receiver
    alertSent = true;
    Serial.println("⚠ MAX ALERT sent: Exceeds 200°!");
  }
  // Check if roll exceeds the normal alert threshold (60°)
  else if (abs(roll) > ALERT_ANGLE && !alertSent) {
    outgoingData.flipped = true;  // Normal alert: Flipped
    outgoingData.maxAlert = false;  // Reset max alert state
    esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));  // Send alert to receiver
    alertSent = true;
    Serial.println("⚠ ALERT sent: Flipped!");
  }

  // Reset alert condition if roll angle is within the threshold
  if (abs(roll) <= ALERT_ANGLE) {
    alertSent = false;
    outgoingData.flipped = false;  // Normal state
    outgoingData.maxAlert = false;  // Normal state (no max alert)
    esp_now_send(peerAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));  // Send normal state to receiver
  }

  delay(200);  // Delay for stability
}
