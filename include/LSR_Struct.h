#pragma once

#include <Arduino.h>

typedef struct LSR_Struct {
  uint32_t TimeStamp;
  float AccelX, AccelY, AccelZ;
  float GyroX, GyroY, GyroZ;
  float VelX, VelY, VelZ;
  float PosX, PosY, PosZ;
  float Theta, Phi, Psi;
  float Pressure;
  float Temp;
  uint8_t State;
} E22_Packet;
