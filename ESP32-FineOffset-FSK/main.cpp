// ESP32 FineOffset Weather Station Internet Bridge
// Copyright (c) 2020 SevenWatt.com , all rights reserved
//

#include <Arduino.h>
#include <SPI.h>
#include <SX1276fsk.h>
#include <libb64/cencode.h>
#include <lwip/apps/sntp.h>
#include "analog.h"
#include "weather.h"
#include "stationconfig.h"
#include "SX1276ws.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// SD Card

#include "FS.h"
#include "SD.h"
//SPIClass spi_sd;
#define SD_SS 32

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// VSPI!
#define RF_SS 5
#define RF_RESET 14
#define RF_CLK 18
#define RF_MISO 19
#define RF_MOSI 23
//#define RF_DIO0 26
//#define RF_DIO4 -1

#define LED 25 // pulsed for each RF packet or MQTT message received
#define LED_RF LED
#define LED_MQTT LED
#define LED_WIFI LED
#define LED_ON 1
#define LED_OFF 0

SPIClass spi_radio;
SX1276ws radio(spi_radio, RF_SS, RF_RESET); // ss and reset pins

uint8_t rfId = 63; // 61=tx-only node, 63=promisc node
uint8_t rfGroup = 42;
uint32_t rfFreq = 868300000;
int8_t rfPow = 17;

uint32_t rfLed = 0;
uint32_t mqttLed = 0;
uint32_t vBatt = 1;

time_t lastWSts = 0;

// Singleton instance of WSConfig
WSConfig wsConfig;

// Singleton class to detect type of weatherstation
WeatherStationProcessor wsProcessor;
uint32_t rfRxNum = 0;

void rfLoop()
{
    static uint8_t pktbuf[70];
    int len = radio.receive(pktbuf, sizeof(pktbuf));
    if (len <= 0)
        return;
    rfRxNum++;
    digitalWrite(LED_RF, LED_ON);
    rfLed = millis();

    WSBase *ws = wsProcessor.processWSPacket(pktbuf, len, radio.rxAt, radio.rssi, radio.snr, radio.lna, radio.afc);
    if (ws)
    {
        ws->print();

        WSSetting *thisStation = wsConfig.lookup(ws->msgformat, ws->stationID);

        if (thisStation)
        {
            thisStation->update(ws, pktbuf);

            // for OLED display: last configured good packet.
            struct timeval tvnow;
            gettimeofday(&tvnow, NULL);
            lastWSts = tvnow.tv_sec;
        }
        else
        {
            // It is a weather station, but not configured.
            // It may be decoded or unknown but with succesful CRC check
            // report succesful and unknown packets on MQTT. Note: WH1080 burst of upto 6 repeating signals.
            printf(ws->mqttPayload().c_str());

            display.clearDisplay();
            display.setTextSize(1);      // Normal 1:1 pixel scale
            display.setTextColor(WHITE); // Draw white text
            display.setCursor(0, 0);     // Start at top-left corner
            display.print("Wavg: ");
            display.print(ws->windspeed/3.6);
            appendFile(SD, "/hello.txt", "World!\n");
            display.println(" m/s");
            display.print("Wgust: ");
            display.print(ws->windgust/3.6);
            display.println(" m/s");
            display.print("Wdir: ");
            display.print(ws->winddir);
            display.println((char)247);
            display.print("Temp: ");
            display.print(ws->temperature);
            display.print((char)247);
            display.println("C");
            display.print("Bat: ");
            display.print(ws->battery_lvl);
            display.println("%");

            display.display();
        }
        delete ws;
    };
}

//===== Setup

void setup()
{

    Serial.begin(115200);

    printf("\n===== ESP32 RF Gateway =====\n");
    printf("Running ESP-IDF %s\n", ESP.getSdkVersion());
    printf("Board type: %s\n", ARDUINO_BOARD);

    setenv("TZ", "UTC-01:00", 1);
    tzset();

    // radio init
    printf("Initializing radio\n");
    spi_radio.begin(RF_CLK, RF_MISO, RF_MOSI);
    radio.init(rfId, rfGroup, rfFreq);
    // Do not use interrupts for weatherstation application
    // radio.setIntrPins(RF_DIO0, RF_DIO4);
    radio.setIntrPins(-1, -1);
    radio.txPower(rfPow);
    radio.setMode(SX1276fsk::MODE_STANDBY);

    // sd init 

    if (!SD.begin(SD_SS,spi_radio,80000000)) {
    Serial.println("Card Mount Failed");
    return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
    Serial.println("MMC");
    } else if(cardType == CARD_SD){
    Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
    } else {
    Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    listDir(SD, "/", 0);
    createDir(SD, "/mydir");
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFileIO(SD, "/test.txt");
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
    // sd init end

    pinMode(LED_RF, OUTPUT);
    digitalWrite(LED_RF, LED_OFF);
    pinMode(LED_WIFI, OUTPUT);
    digitalWrite(LED_WIFI, LED_ON);

    delay(200);

    // Load the weather station configuration from flash memory
    wsConfig.load();

    // Display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    display.display();
    delay(2000); // Pause for 2 seconds

    // Clear the buffer
    display.clearDisplay();

    printf("===== Setup complete\n");
}

uint32_t lastWiFiConn = millis();
uint32_t lastInfo = -1000000;
bool wifiConn = false;
uint32_t lastReport = -50 * 1000;

void loop()
{
    rfLoop();

    if (rfLed != 0 && millis() - rfLed > 200)
    {
        digitalWrite(LED_RF, LED_OFF);
        rfLed = 0;
    }



}