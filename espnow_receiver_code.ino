/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>


const int pwmFreq = 500; // Frequency of the PWM signal (5 kHz in this case)
const int pwmResolution = 8; // Resolution of PWM (8-bit resolution gives values from 0 to 255)
const int RChannel = 0; // Channel to use for PWM (ESP32 supports 16 channels)
const int LChannel = 1;
const int rightMotor = 23;
const int leftMotor = 22;
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    char a[32];
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.println();

  if (myData.a[0] == 'w') { 
    ledcWrite(RChannel, 200);
    ledcWrite(LChannel, 200);
  }
  else if (myData.a[0] == 'a') {  
    ledcWrite(RChannel, 200);
    ledcWrite(LChannel, 180);
  }
  else if (myData.a[0] == 's') { 
    ledcWrite(RChannel, 180);
    ledcWrite(LChannel, 180);
  }
  else if (myData.a[0] == 'd') {  
    ledcWrite(RChannel, 180);
    ledcWrite(LChannel, 200);
  }
  else{
    ledcWrite(RChannel, 191);
    ledcWrite(LChannel, 191);
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  ledcSetup(RChannel, pwmFreq, pwmResolution);
  ledcAttachPin(rightMotor, RChannel);  // Attach the PWM to the pin
  ledcSetup(LChannel, pwmFreq, pwmResolution);
  ledcAttachPin(leftMotor, LChannel);  // Attach the PWM to the pin

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {

}