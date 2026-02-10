#pragma once

#include <Arduino.h>

enum State {
    PRE_LAUNCH,
    ASCENT,
    DESCENT,
    LANDED
};

enum RollState {
    OFF,
    CLOCKWISE,
    COUNTERCLOCKWISE,
    HOLDING
};

typedef struct LSR_Struct {
  uint32_t TimeStamp;
  float AccelX, AccelY, AccelZ;
  float GyroX, GyroY, GyroZ;
  float VelX, VelY, VelZ;
  float PosX, PosY, PosZ;
  float Theta, Phi, Psi;
  float Pressure;
  float Temp;
  State flightState = PRE_LAUNCH;
  RollState rollControlState = OFF;
} E22_Packet;
