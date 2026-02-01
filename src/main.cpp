#include <Arduino.h>
#include <CrashReport.h>
#include <LSR_Struct.h>
#include <SD.h>
#include <SPI.h>

#define DEBUG 1

#if DEBUG
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

enum State {
  STATE_IDLE,
  STATE_LAUNCH,
  STATE_APOGEE,
  STATE_LANDED
} rckt_state;

E22_Packet packet;

void setup() {
  Serial.begin(115200);

  // Initalize SPI
  SPI.begin();

  // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    DEBUG_PRINTLN("SD card initialization failed!");
  }

  // Check for crash report
  if(CrashReport) {
    DEBUG_PRINTLN("Previous crash detected!\n");
    File crashReport = SD.open("crashreport.log", FILE_WRITE);

    if (crashReport) {
      CrashReport.printTo(crashReport);
      crashReport.close();
      DEBUG_PRINTLN("Crash report written to crashreport.log");
    } else {
      DEBUG_PRINTLN("Failed to open crashreport.log for writing!");
    }
    
    if(DEBUG) {
      DEBUG_PRINTLN("Crash Report:");
      CrashReport.printTo(Serial);
    }

    CrashReport.clear();
  }

  // Initialize state machine
  rckt_state = STATE_IDLE;
  packet.State = STATE_IDLE;

}

void loop() {
  
}
