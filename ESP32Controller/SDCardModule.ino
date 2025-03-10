#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SD_MOSI     23
#define SD_MISO     19
#define SD_SCLK     18
#define SD_CS       5

File myFile;
char fileName[50];

void SD_Init(int year, int month, int date, int hour, int seconds) {
  Serial.println("Starting SD Card setup!");
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  // Try continually to mount the SD card
  while (!SD.begin(SD_CS)) {
    Serial.println("SD Card Mount Failed! Trying again in 5 seconds...");
    sleep(5);
  }
 
  uint32_t cardSize = SD.cardSize() / (1024 * 1024);
  String str = "SDCard Size: " + String(cardSize) + "MB";
  Serial.println(str);
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE) {
    Serial.println("No SD Card attached!");
  }

  // Creates the file name
  sprintf(fileName, "/data_%d-%d-%d_%d-%d.csv", year, month, date, hour, seconds);
  //sprintf(fileName, "test.txt");
  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile) {
    myFile.println("longitude,latitude,depth");
    myFile.close();
    Serial.printf("Successfully created new data file \"%s\"!\n", fileName);
  } else {
    Serial.printf("Error opening new file \"%s\"!\n", fileName);
  }
}

void Record_Reading(float longitude, float latitude, float depth) {
  myFile = SD.open(fileName, FILE_APPEND);
  if (myFile) { 
    char newDataLine[100];
    sprintf(newDataLine, "%f,%f,%f", longitude, latitude, depth);
    Serial.printf("Writing: %s\n", newDataLine);
    myFile.println(newDataLine);
    myFile.close();
    Serial.println("Successfully recorded new reading to SD card!");
  } else {  
    Serial.printf("Error opening file \"%s\"!\n", fileName);
  }
}

void SD_Uninit() {
  SD.end();
  SPI.end();
  Serial.println("Successfully uninitialized SD card module!");
}