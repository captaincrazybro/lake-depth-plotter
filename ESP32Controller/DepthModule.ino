#include <SoftwareSerial.h>
unsigned char buffer_RTT[4] = {0};
uint8_t CS;
#define COM 0x55
int Distance = 0;
SoftwareSerial mySerial(7, 8); 

// Initializes the serial module for reading the depth data
void Depth_Init() {
  mySerial.begin(115200);
}

// Measures a single depth point
int Measure_Depth() {
  // Writes the `COM` character to the serial
  mySerial.write(COM);
  // Delays for a bit
  delay(100);
  if(mySerial.available() > 0){
    delay(4);
    if(mySerial.read() == 0xff){    
      buffer_RTT[0] = 0xff;
      // Reads all four
      for (int i=1; i<4; i++){
        buffer_RTT[i] = mySerial.read();   
      }
      // Parses the read buffers
      CS = buffer_RTT[0] + buffer_RTT[1]+ buffer_RTT[2];  
      if(buffer_RTT[3] == CS) {
        Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
        Serial.print("Distance:");
        Serial.print(Distance);
        Serial.println("mm");
      }
    }
  }
}
