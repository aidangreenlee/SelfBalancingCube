#include <esp_now.h>
#include <WiFi.h>

// Initialize received int
int data_received = 0;

// Callback function that will be called when a packet is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  // Copy the received data to the data_received variable
  memcpy(&data_received, data, sizeof(data_received));
  
  // Print the received data to the serial monitor
  Serial.print("Received data: ");
  Serial.println(data_received);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function to handle received data
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Keep the loop empty as all the data processing is done in the callback function
}
