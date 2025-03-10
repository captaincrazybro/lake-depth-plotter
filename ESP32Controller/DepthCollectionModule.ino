#include <SoftwareSerial.h>
unsigned char buffer_RTT[4] = {0};
uint8_t CS;
#define COM 0x55
int Depth_Reading = 0;
SoftwareSerial depthSerial(7, 8); 

// Initializes the serial module for reading the depth data
void Depth_Init() {
  depthSerial.begin(115200);
}

// Measures a single depth point
int Measure_Depth() {
  // Writes the `COM` character to the serial
  depthSerial.write(COM);
  // Delays for a bit
  delay(100);
  if(depthSerial.available() > 0){
    delay(4);
    if(depthSerial.read() == 0xff){    
      buffer_RTT[0] = 0xff;
      // Reads all four
      for (int i=1; i<4; i++){
        buffer_RTT[i] = depthSerial.read();   
      }
      // Parses the read buffers
      CS = buffer_RTT[0] + buffer_RTT[1]+ buffer_RTT[2];  
      if(buffer_RTT[3] == CS) {
        Depth_Reading = (buffer_RTT[1] << 8) + buffer_RTT[2];
      }
    }
  }

  return Depth_Reading;
}
