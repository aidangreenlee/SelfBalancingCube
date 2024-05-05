#include <esp_now.h>
#include <WiFi.h>

// Sender

// Replace these MAC addresses with the MAC addresses of your receiver ESP32
uint8_t recieverMAC[] = {0x10, 0x06, 0x1C, 0x82, 0x5A, 0x68};

// Create variables to store gain values
float KP;
float KI;
float KD;

bool new_KP = false; // Flag to track if new values are available
bool new_KI = false; // Flag to track if new values are available
bool new_KD = false; // Flag to track if new values are available

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
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input string from Serial
    input.trim(); // Remove leading/trailing whitespaces
    
    // Parse input string and extract gain values
    if (input.startsWith("KP=") || input.startsWith("KI=") || input.startsWith("KD=")) {
      // input.remove(0, 3); // Remove "KP="
      // KP = input.toFloat(); // Convert remaining string to float
      new_KP = true; // Set flag to indicate new values are available
    } 
    // else if (input.startsWith("KI=")) {
    //   // input.remove(0, 3); // Remove "KI="
    //   // KI = input.toFloat(); // Convert remaining string to float
    //   new_KI = true; // Set flag to indicate new values are available
    // }
    // else if (input.startsWith("KD=")) {
    //   // input.remove(0, 3); // Remove "KD="
    //   // KD = input.toFloat(); // Convert remaining string to float
    //   new_KD = true; // Set flag to indicate new values are available
    // }

  // If new values are available, send them via ESP-NOW
    esp_err_t result;

    if (new_KP) {

    // result = esp_now_send(recieverMAC, (uint8_t *) &KP, sizeof(KP));
    char SMELLME[input.length() + 1];
    input.toCharArray(SMELLME, input.length() + 1);

    result = esp_now_send(recieverMAC, (const uint8_t *) &SMELLME, sizeof(SMELLME));
    if (result == ESP_OK) {
      Serial.println("Data Sent with success");
    } else {
      Serial.println("Error sending Data");
    }
    delay(100); // Delay to ensure packets are sent separately
    }

    // Reset the flag to indicate that values have been sent
    new_KP = false;
  }
  // Delay before sending the next set of gains
}
