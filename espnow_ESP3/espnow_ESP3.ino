#include <esp_now.h>
#include <WiFi.h>


const int pwmFreq = 500; // Frequency of the PWM signal (5 kHz in this case)
const int pwmResolution = 8; // Resolution of PWM (8-bit resolution gives values from 0 to 255)
const int RChannel = 0; // Channel to use for PWM (ESP32 supports 16 channels)
const int LChannel = 1;
const int rightMotor = 23;
const int leftMotor = 22;
const int Idle = 191;

int maxForwardSpeed = 256;
int minForwardSpeed = 192;
int maxReverseSpeed = 128;
int minReverseSpeed = 190;
int forwardSpeed = 200;
int reverseSpeed = 182;

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
  else{
    ledcWrite(RChannel, Idle);
    ledcWrite(LChannel, Idle);
  }
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  const int numberPoints = 6;
  float coordinates[numberPoints * numberPoints][2];

  generateGrid(10.734432, 10.329487, 100.230458, 100.349327, numberPoints, coordinates);
  ledcSetup(RChannel, pwmFreq, pwmResolution);
  ledcAttachPin(rightMotor, RChannel);  // Attach the PWM to the pin
  ledcSetup(LChannel, pwmFreq, pwmResolution);
  ledcAttachPin(leftMotor, LChannel);  // Attach the PWM to the pin
  Serial.println("setup test");
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
 
void generateGrid(float X1, float Y1, float X2, float Y2, int numberPoints, float coordinates[][2]) {
    float xStep = (X2 - X1) / (numberPoints - 1);
    float yStep = (Y2 - Y1) / (numberPoints - 1);

    int index = 0;
    for (int i = 0; i < numberPoints; i++) {
        for (int j = 0; j < numberPoints; j++) {
            coordinates[index][0] = X1 + i * xStep;
            coordinates[index][1] = Y1 + j * yStep;
            index++;
        }
    }
}

void loop() {

}