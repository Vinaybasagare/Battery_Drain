// Include Libraries
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <cstring> // For memcpy
#include "esp_sleep.h"
#include <esp_wifi.h>


#define LED_PIN 17 // Define the LED pin
#define SENSOR_POWER_PIN 16
#define ONE_WIRE_BUS 4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// Variables for test data
float batteryVoltage;
const int MAX_SENSORS = 2;
float temperatures[MAX_SENSORS];

// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x8E, 0xBA, 0xE8};

int readTemperatures(float tempArray[], int maxSize) {
  // Function to read the 1wire temperature sensor(s)
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(100);

  sensors.requestTemperatures();
  int count = min(sensors.getDeviceCount(), (uint8_t)maxSize);

  for (int i = 0; i < count; i++) {
    tempArray[i] = sensors.getTempCByIndex(i);
  }

  digitalWrite(SENSOR_POWER_PIN, LOW);
  return count;
}
struct DataPacket {
  float variable1;
  float variable2;
  float variable3;
};
// Peer info
esp_now_peer_info_t peerInfo;

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  
  // Set up Serial Monitor
  Serial.begin(115200);
 pinMode(SENSOR_POWER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  WiFi.mode(WIFI_OFF);
  delay(100);
  // Read the raw ADC value from GPIO26
  int rawADC = analogRead(26);
  Serial.print("Raw ADC Value on GPIO26: ");
  Serial.println(rawADC);
  batteryVoltage = rawADC * 0.0017 + 0.6326;
  // Serial.print("Battery Voltage: ");
  // Serial.print(batteryVoltage, 2); // Print battery voltage with 2 decimal places
  // Serial.println(" V");
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(100);
  sensors.begin();
  digitalWrite(SENSOR_POWER_PIN, LOW);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {

 // Read temperatures
  int numReadings = readTemperatures(temperatures, MAX_SENSORS);

  // Prepare data packet
  DataPacket data;
  data.variable1 = batteryVoltage;
  data.variable2 = (numReadings > 0) ? temperatures[0] : -273;
  data.variable3 = (numReadings > 1) ? temperatures[1] : -273;

  // Print sensor readings
  Serial.print("Battery Voltage: ");
  Serial.println(data.variable1, 2);
  for (int i = 0; i < numReadings; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(temperatures[i]);
    Serial.println(" °C");
  }
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));
   
  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  }
  else {
    Serial.println("Sending error");
  }
  delay(100);
  // Configure ESP32 to wake up after 60 seconds
  esp_sleep_enable_timer_wakeup(1 * 1000000);
  esp_deep_sleep_start();
}