#include <SoftwareSerial.h>
unsigned char buffer_RTT[4] = {0};
uint8_t CS;
#define COM 0x55
#define RXD2 16
#define TXD2 17
int Depth_Reading = 0;
// SoftwareSerial depthSerial(12, 13); 

// Initializes the serial module for reading the depth data
void Depth_Init() {
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 
}

// Measures a single depth point
int Measure_Depth() {
  // Writes the `COM` character to the serial
  Serial2.write(COM);
  // Delays for a bit
  delay(100);
  if(Serial2.available() > 0){
    delay(4);
    if(Serial2.read() == 0xff){ 
      buffer_RTT[0] = 0xff;
      // Reads all four
      for (int i=1; i<4; i++){
        buffer_RTT[i] = Serial2.read();   
      }
      // Parses the read buffers
      CS = buffer_RTT[0] + buffer_RTT[1]+ buffer_RTT[2];  
      if(buffer_RTT[3] == CS) {
        Depth_Reading = (buffer_RTT[1] << 8) + buffer_RTT[2];
        if (Depth_Reading > 0) {
          Serial.print("Successful non-zero depth reading: ");
          Serial.println(Depth_Reading);
        }
      } else {
        return -3;
      }
    } else {
      return -2;
    }
  } else {
    return -1;
  }

  return Depth_Reading;
}
