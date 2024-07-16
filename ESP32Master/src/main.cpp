
#include <WiFi.h>
#include <esp_now.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>

#define LED_PIN 2 // Example LED pin for status indication

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // Adjust timezone and update interval as needed

// Define a data structure
struct DataPacket {
  float variable1;
  float variable2;
  float variable3;
};

struct DataRecord {
  float variable1;       // Battery voltage
  float variable2;       // Temperature 1
  float variable3;       // Temperature 2
  uint8_t macAddr[6];    // MAC address of the sender
  char timestamp[30];    // Timestamp as a string (including date)
};

DataPacket receivedData;

const int maxRecords = 1500; // Adjust the size as needed
DataRecord dataRecords[maxRecords];
int recordIndex = 0;

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (recordIndex < maxRecords) {
    // Copy the incoming data into receivedData struct
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    // Update the NTP client to get the latest time
    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();

    // Convert epoch time to human-readable format
    char formattedTime[30];
    snprintf(formattedTime, sizeof(formattedTime), "%02d-%02d-%04d %02d:%02d:%02d",
             day(epochTime), month(epochTime), year(epochTime),
             hour(epochTime), minute(epochTime), second(epochTime));

    // Store the data in the array
    dataRecords[recordIndex].variable1 = receivedData.variable1;
    dataRecords[recordIndex].variable2 = receivedData.variable2;
    dataRecords[recordIndex].variable3 = receivedData.variable3;
    memcpy(dataRecords[recordIndex].macAddr, mac, 6);
    snprintf(dataRecords[recordIndex].timestamp, sizeof(dataRecords[recordIndex].timestamp), "%s", formattedTime);
    recordIndex++;

    // Print the received values
    Serial.print("Data received: ");
    Serial.print("Received Battery Voltage: ");
    Serial.println(receivedData.variable1, 2); // Print with 2 decimal places
    Serial.print("Received Temperature 1: ");
    Serial.println(receivedData.variable2, 2); // Print with 2 decimal places
    Serial.print("Received Temperature 2: ");
    Serial.println(receivedData.variable3, 2); // Print with 2 decimal places
    Serial.print("Timestamp: ");
    Serial.println(dataRecords[recordIndex - 1].timestamp); // Print the timestamp
    Serial.println();
  } else {
    Serial.println("Data storage full. Increase maxRecords to store more data.");
  }
}

void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);

  // Initialize Wi-Fi
  WiFi.mode(WIFI_STA);
WiFi.begin("your_SSID", "your_PASSWORD");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  // Initialize NTP client
  timeClient.begin();
  timeClient.update();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}

void printStoredData() {
  for (int i = 0; i < recordIndex; i++) {
    Serial.print("Record ");
    Serial.print(i);
    Serial.print(": Battery Voltage: ");
    Serial.print(dataRecords[i].variable1, 2);
    Serial.print(", Temperature 1: ");
    Serial.print(dataRecords[i].variable2, 2);
    Serial.print(", Temperature 2: ");
    Serial.print(dataRecords[i].variable3, 2);
    Serial.print(", MAC Address: ");
    for (int j = 0; j < 6; j++) {
      Serial.printf("%02X", dataRecords[i].macAddr[j]);
      if (j < 5) Serial.print(":");
    }
    Serial.print(", Timestamp: ");
    Serial.println(dataRecords[i].timestamp);
  }
}

void loop() {
  printStoredData();
  delay(5000); 
}
