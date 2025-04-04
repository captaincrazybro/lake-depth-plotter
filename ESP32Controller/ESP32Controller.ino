#include <TinyGPS++.h>
#include <esp_now.h>
#include <WiFi.h>

//VARS
//Motor control 
const int pwmFreq = 500; // Frequency of the PWM signal (5 kHz in this case)
const int pwmResolution = 8; // Resolution of PWM (8-bit resolution gives values from 0 to 255)
const int RChannel = 0; // Channel to use for PWM (ESP32 supports 16 channels)
const int LChannel = 1;
const int rightMotor = 23;
const int leftMotor = 22;
const int Idle = 191;
const int maxForwardSpeed = 256;
const int minForwardSpeed = 192;
const int maxReverseSpeed = 128;
const int minReverseSpeed = 190;
int forwardSpeed = 210;
int reverseSpeed = 172;

//grid generation
const int numPoints = 6;
float gridPoints[numPoints * numPoints][2];
float splitCoordinates[4];

//depth collection
int depth;

//navigation

//sdcard

// The TinyGPS++ object
TinyGPSPlus gps;

// Must match the sender structure
typedef struct struct_message {
    char a[44];
} struct_message;

// Struct to hold motor data for controlling the motors
struct Motor_Data {
  int left;
  int right;
};

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.print("Speed");
  Serial.println(forwardSpeed);
  Serial.println(reverseSpeed);
  Serial.println();
  
  if (myData.a[0] == 'w') { 
    ledcWrite(RChannel, forwardSpeed);
    ledcWrite(LChannel, forwardSpeed);
  }
  else if (myData.a[0] == 'a') {  
    ledcWrite(RChannel, forwardSpeed);
    ledcWrite(LChannel, reverseSpeed);
  }
  else if (myData.a[0] == 's') { 
    ledcWrite(RChannel, reverseSpeed);
    ledcWrite(LChannel, reverseSpeed);
  }
  else if (myData.a[0] == 'd') {  
    ledcWrite(RChannel, reverseSpeed);
    ledcWrite(LChannel, forwardSpeed);
  }
  else if (myData.a[0] == 'u') { 
    Serial.print("forward");
    forwardSpeed = forwardSpeed + 1;
    reverseSpeed = reverseSpeed - 1;
    if(forwardSpeed > maxForwardSpeed){
      forwardSpeed = maxForwardSpeed;
    }
    if(reverseSpeed < maxReverseSpeed){
      reverseSpeed = maxReverseSpeed;
    }
  }
  else if (myData.a[0] == 'j') {  
    Serial.print("back");
    forwardSpeed = forwardSpeed - 1;
    reverseSpeed = reverseSpeed + 1;
    if(forwardSpeed < minForwardSpeed){
      forwardSpeed = minForwardSpeed;
    }
    if(reverseSpeed > minReverseSpeed){
      reverseSpeed = minReverseSpeed;
    }
  }
  else if (strlen(myData.a) > 10){
    SD_Init(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute());
    convertCords(myData.a, splitCoordinates);
    memset(gridPoints, 0, sizeof(gridPoints));  // Clear array
    generateGrid(splitCoordinates[0], splitCoordinates[1], splitCoordinates[2], splitCoordinates[3], numPoints, gridPoints);
  }
  else if (myData.a[0] == 'i'){
    ledcWrite(RChannel, Idle);
    ledcWrite(LChannel, Idle);
  }
  else{
    Record_Reading(gps.locaiton.lng(), gps.location.lat(), Measure_Depth(), gps.speed.mph);
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Depth_Init();
  GPS_Init();
  SD_Init(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute());
  ledcSetup(RChannel, pwmFreq, pwmResolution);
  ledcAttachPin(rightMotor, RChannel);  
  ledcSetup(LChannel, pwmFreq, pwmResolution);
  ledcAttachPin(leftMotor, LChannel);  

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
 
void loop() {}
