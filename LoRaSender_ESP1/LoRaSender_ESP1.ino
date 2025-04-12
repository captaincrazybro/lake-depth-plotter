#include "LoRaWan_APP.h"
#include "Arduino.h"


#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 45 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle=true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
	
    txNumber=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
   }



void loop()
{
	if(lora_idle == true){}//wait to send instruction
  if (Serial.available() > 0)
  {
    String input = Serial.readString();
    input.trim();//fix arduino adding a /n
    if (input.length() == 1)
    {
      char command = input [0];
      switch (command)
      {
        case 'w':
          sprintf(txpacket,"w"); 
          break;
        case 'a':
          sprintf(txpacket,"a"); 
          break;
        case 's':
          sprintf(txpacket,"s"); 
          break;
        case 'd':
          sprintf(txpacket,"d"); 
          break;
        case 'u':
          sprintf(txpacket,"u"); 
          break;
        case 'j':
          sprintf(txpacket,"j"); 
          break;
        case 'h':
          sprintf(txpacket,"h");
          break;
        case 't':
          sprintf(txpacket,"t");
          break;
        case 'r':
          sprintf(txpacket,"r");
          break;
        default:
          sprintf(txpacket,"i");
          break;
      }
    }
    else
    {
      sprintf(txpacket, "%s", input.c_str());
    }
  
  // delay(1000);
  txNumber += 0.01;
  //sprintf(txpacket,"Hello world number %0.2f",txNumber);  //start a package
  
  Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

  Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out	
  lora_idle = false;
	}
  Radio.IrqProcess( );
}

void OnTxDone( void )
{
	Serial.println("TX done......");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}