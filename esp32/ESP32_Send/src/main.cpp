// THIS CODE IS FOR THE SENDER

#include <esp_now.h>
#include <WiFi.h>

// MAC Address of Reciever: 10:06:1C:82:5A:68
uint8_t recieverMAC[] = {0x10, 0x06, 0x1C, 0x82, 0x5A, 0x68};


// Create int to store data value
int data;

esp_now_peer_info_t peerInfo; // store peer info

// callback when data is sent, prints message whether fail or success
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {

  Serial.begin(115200); 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer ESP32
  memcpy(peerInfo.peer_addr, recieverMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer ESP32      
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {

if (Serial.available() > 0){
    data = Serial.parseInt(); // Read int value from Serial
  }
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(recieverMAC, (uint8_t *) &data, sizeof(data));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  // Send this message every two seconds
  delay(2000);
}