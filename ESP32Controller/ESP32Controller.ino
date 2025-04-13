#include <TinyGPS++.h>
#include <esp_now.h>
#include <WiFi.h>

//VARS
//Motor control 
const int pwmFreq = 500; // Frequency of the PWM signal (5 kHz in this case)
const int pwmResolution = 8; // Resolution of PWM (8-bit resolution gives values from 0 to 255)
const int RChannel = 0; // Channel to use for PWM (ESP32 supports 16 channels)
const int LChannel = 1;
const int rightMotor = 25;
const int leftMotor = 26;
const int Idle = 191;
const int maxForwardSpeed = 256;
const int minForwardSpeed = 192;
const int maxReverseSpeed = 128;
const int minReverseSpeed = 190;
int forwardSpeed = 210;
int reverseSpeed = 172;

//grid generation
const int numPoints = 2;
float gridPoints[numPoints * numPoints][2];
float splitCoordinates[4];

//depth collection
int depth;

//navigation
int pointIndex = 0;// how the boat knows which point to go to next
float yPoint = gridPoints[pointIndex][0]; //lat 
float xPoint = gridPoints[pointIndex][1]; //long
float threshold = 0.00003;
float currentX;
float currentY;
float targetX;
float targetY;
float dx;
float dy;
float dD;//delta distance
bool isTraversing = false;
bool goHome = false;
bool needsStopped = false;
float homeX;
float homeY;
int currentTraj;

//sdcard
long record_millis = 0;
long record_delay_ms = 1000;

// The TinyGPS++ object
TinyGPSPlus gps;
bool gpsLockFound = false;

// Must match the sender structure
typedef struct struct_message {
    char a[44];
} struct_message;

// Struct to hold motor data for controlling the motors
struct Motor_Data {
  int left;
  int right;
};
Motor_Data data;

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
  else if (myData.a[0] == 'h') {  //home
    isTraversing = false;
    goHome = true;
  }
  else if (myData.a[0] == 'r') {  //resume
    isTraversing = true;
    goHome = false;
  }
  else if (myData.a[0] == 't') {  //stop
    isTraversing = false;
    goHome = false;
    needsStopped = true;
  }
  else if (strlen(myData.a) > 3){
    if (!isTraversing) {
      New_File(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute());
      convertCords(myData.a, splitCoordinates);
      memset(gridPoints, 0, sizeof(gridPoints));  // Clear array
      generateGrid(splitCoordinates[0], splitCoordinates[1], splitCoordinates[2], splitCoordinates[3], numPoints, gridPoints);
      isTraversing = true;
    }
  }
  else if (myData.a[0] == 'i'){
    ledcWrite(RChannel, Idle);
    ledcWrite(LChannel, Idle);
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Depth_Init();
  GPS_Init();
  SD_Init();
  record_millis = millis();
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
 
void loop() {
  // Updates the gps store
  GPS_Update();

  while (!gps.location.isValid()) {
    Serial.println("Waiting for GPS fix...");
    Serial.print("Lat: "); Serial.println(gps.location.lat(), 6);
    Serial.print("Lng: "); Serial.println(gps.location.lng(), 6);
    Serial.print("Satellites: "); Serial.println(gps.satellites.value());

    delay(100);
    GPS_Update();
  }

  // Waits for first GPS lock then records home coordinates
  if (!gpsLockFound && gps.location.isValid()) {//changed this from updated to valid
    gpsLockFound = true;
    homeX = gps.location.lng();
    homeY = gps.location.lat();
    Serial.print("home coords");
    Serial.print(homeX);
    Serial.print(homeY);
  }

  // if (isIdle){//needs this or only goes idle for a second
  //   ledcWrite(RChannel, Idle);
  //   ledcWrite(LChannel, Idle);
  // }

  if (needsStopped) {
    ledcWrite(RChannel, Idle);
    ledcWrite(LChannel, Idle);
    needsStopped = false;
  } else {
    // If in the midst of traversing point grid
    if (isTraversing) {
      currentX = gps.location.lng();
      currentY = gps.location.lat();
      targetX = gridPoints[pointIndex][1]; //long
      targetY = gridPoints[pointIndex][0]; //lat
      dx = currentX - targetX;
      dy = currentY - targetY;
      dD = dx*dx + dy*dy;
      // If reaches the point within threshold, move on to text point
      if (dD <= threshold*threshold){
        Serial.print("Point reached");
        pointIndex++;
      }
      currentTraj = Compass_Get_Trajectory();
      data = Calculate_Motor_Data(currentX, currentY, targetX, targetY, currentTraj);
      // TODO: Remove this after first testing
      Serial.print("Traversing... Current traj: ");
      Serial.println(currentX);
      Serial.println(currentY);
      Serial.println(targetX);
      Serial.println(targetY);
      delay(2000);
      //Serial.print(currentTraj);
      //Serial.print(", Left code: ");
      //Serial.print(data.left);
      //Serial.print(", Right code: ");
      //Serial.println(data.right);

      // Measures the depth
      depth = Measure_Depth();
      if (depth != 0 && (millis() - record_millis) > record_delay_ms) {
        Record_Reading(gps.location.lng(), gps.location.lat(), Measure_Depth(), gps.speed.mph());
        record_millis = millis();
      }

      // If reaches the end of the grid, stops traversing
      if (pointIndex >= numPoints*numPoints) {
        isTraversing = false;
        goHome = true;
      // Outputs commands to motor only if haven't finished traversing
      } else {
        ledcWrite(RChannel, data.right);
        ledcWrite(LChannel, data.left);
      }
    } 
    else if (goHome) {
      currentX = gps.location.lng();
      currentY = gps.location.lat();
      dx = currentX - homeX;
      dy = currentY - homeY;
      dD = dx*dx + dy*dy;
      if (dD <= threshold*threshold){
        goHome = false;
        ledcWrite(RChannel, Idle);
        ledcWrite(LChannel, Idle);
      }
      currentTraj = Compass_Get_Trajectory();
      data = Calculate_Motor_Data(currentX, currentY, homeX, homeY, currentTraj);
      // TODO: Remove this after first testing
      Serial.print("Going home... Current traj: ");
      Serial.print("x: ");
      Serial.println(dx);
      Serial.print("y: ");
      Serial.println(dy);
      Serial.print(dD);
      //Serial.print(currentTraj);
      //Serial.print(", Left code: ");
      //Serial.print(data.left);
      //Serial.print(", Right code: ");
      //Serial.println(data.right);

      ledcWrite(RChannel, data.right);
      ledcWrite(LChannel, data.left);
    }
  }
  delay(100);
}
