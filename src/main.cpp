#include <Arduino.h>
#include "SensorManager.h"

// Chip Select pins
constexpr uint8_t LSM_CS = 10;
constexpr uint8_t BMP_CS = 9;

SensorManager sensors(LSM_CS, BMP_CS);

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("Teensy 4.1 Sensor System");

  if (!sensors.begin()) {
    Serial.println("Sensor init failed");
    while (1);
  }

  Serial.println("All sensors initialized");
}

void loop() {
  sensors.update();

  if (millis() - lastPrint > 200) {
    lastPrint = millis();

    Serial.println("---- DATA ----");

    Serial.print("Accel g: ");
    Serial.print(sensors.ax, 3); Serial.print(", ");
    Serial.print(sensors.ay, 3); Serial.print(", ");
    Serial.println(sensors.az, 3);

    Serial.print("Gyro dps: ");
    Serial.print(sensors.gx, 3); Serial.print(", ");
    Serial.print(sensors.gy, 3); Serial.print(", ");
    Serial.println(sensors.gz, 3);

    Serial.print("Temp C: ");
    Serial.println(sensors.temperature, 2);

    Serial.print("Pressure hPa: ");
    Serial.println(sensors.pressure_hPa, 2);

    Serial.print("Altitude m: ");
    Serial.println(sensors.altitude_m, 2);

    if (sensors.gpsFix) {
      Serial.print("Lat: ");
      Serial.println(sensors.latitude, 6);

      Serial.print("Lon: ");
      Serial.println(sensors.longitude, 6);

      Serial.print("GPS Alt m: ");
      Serial.println(sensors.gpsAltitude);

      Serial.print("Sats: ");
      Serial.println(sensors.satellites);
    } else {
      Serial.println("GPS: No fix");
    }

    Serial.println();
  }
}

