#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>

char direction;

//ESP-Now Vars
//MAC Address
uint8_t broadcastAddress[] = {0x08, 0xa6, 0xf7, 0xb1, 0xd3, 0xec};
// Must match the receiver structure
typedef struct struct_message {
  char a[44];
} struct_message;
// Create a struct_message called myData
struct_message myData;
esp_now_peer_info_t peerInfo;

//LoRa Definitions
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,1: 250 kHz,2: 500 kHz,3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,2: 4/6,3: 4/7,4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 45 // Define the payload size here

//LoRa Vars
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;
int16_t txNumber;
int16_t rssi,rxSize;
bool lora_idle = true;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  //ESP-Now setup
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
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


  //LoRa Setup
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  txNumber=0;
  rssi=0;
  RadioEvents.RxDone = OnRxDone;
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                              LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                              LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                              0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
}



void loop()
{
  //LoRa
  if(lora_idle)
  {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess( );
  // delay(2000);
}

void Send_ESP_Now_Sig() {
  //Espnow
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';
    Radio.Sleep( );
    Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",rxpacket,rssi,rxSize);
    if (strlen(rxpacket) == 1)
    {
      if (strcmp(rxpacket, "w") == 0) {
          strcpy(myData.a, "w");
      } 
      else if (strcmp(rxpacket, "a") == 0) {
          strcpy(myData.a, "a");
      } 
      else if (strcmp(rxpacket, "s") == 0) {
          strcpy(myData.a, "s");
      } 
      else if (strcmp(rxpacket, "d") == 0) {
          strcpy(myData.a, "d");              
      } 
      else if (strcmp(rxpacket, "u") == 0) {
          strcpy(myData.a, "u");
      } 
      else if (strcmp(rxpacket, "j") == 0) {
          strcpy(myData.a, "j");              
      } 
      else if (strcmp(rxpacket, "h") == 0) {
          strcpy(myData.a, "h");
      } 
      else if (strcmp(rxpacket, "t") == 0) {
          strcpy(myData.a, "t");
      }
      else if (strcmp(rxpacket, "r") == 0) {
          strcpy(myData.a, "r");
      }
      else {
          strcpy(myData.a, "i");
      }
    }
    else 
    {
      strncpy(myData.a, rxpacket, 44);
    }
    
    Send_ESP_Now_Sig();
    lora_idle = true;
}