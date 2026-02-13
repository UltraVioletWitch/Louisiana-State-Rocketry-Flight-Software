#pragma once

#include <Arduino.h>
#include <vector>

enum State {
    PRE_LAUNCH,
    BURN,
    COAST,
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

template <int N>
class RingBuffer {
    private:
        LSR_Struct ring[N];
        int ring_ptr = 0;
        bool full = false;

    public:
        void push(LSR_Struct packet) {
            if (ring_ptr == N - 1) {
                ring_ptr = 0;
                full = true;
            } else {
                ring_ptr++;
            }

            ring[ring_ptr] = packet;
        }
    
        LSR_Struct getFirst(void) const {
            return ring[ring_ptr];
        }

        LSR_Struct getLast(void) const {
            int last_ptr;
            if (ring_ptr == N - 1) {
                last_ptr = 0;
            } else {
                last_ptr = ring_ptr + 1;
            }

            return ring[last_ptr];
        }

        bool isFull(void) const {
            return full;
        }
};
