#pragma once

#include <Arduino.h>

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

enum Acceleration {
    G_1 = 10,      // 9.81 m/s²
    G_2 = 20,      // 19.62 m/s²
    G_3 = 29,      // 29.43 m/s²
    G_4 = 39,      // 39.24 m/s²
    G_5 = 49,      // 49.05 m/s²
    G_6 = 59,      // 58.86 m/s²
    G_7 = 69,      // 68.67 m/s²
    G_8 = 78,      // 78.48 m/s²
    G_9 = 88,      // 88.29 m/s²
    G_10 = 98,     // 98.1 m/s²
    G_11 = 108,    // 107.91 m/s²
    G_12 = 118,    // 117.72 m/s²
    G_13 = 128,    // 127.53 m/s²
    G_14 = 137,    // 137.34 m/s²
    G_15 = 147,    // 147.15 m/s²
    G_16 = 157,    // 156.96 m/s²
    G_17 = 167,    // 166.77 m/s²
    G_18 = 177,    // 176.58 m/s²
    G_19 = 186,    // 186.39 m/s²
    G_20 = 196,    // 196.2 m/s²
    G_21 = 206,    // 206.01 m/s²
    G_22 = 216,    // 215.82 m/s²
    G_23 = 226,    // 225.63 m/s²
    G_24 = 235,    // 235.44 m/s²
    G_25 = 245,    // 245.25 m/s²
    G_26 = 255,    // 255.06 m/s²
    G_27 = 265,    // 264.87 m/s²
    G_28 = 275,    // 274.68 m/s²
    G_29 = 284,    // 284.49 m/s²
    G_30 = 294,    // 294.3 m/s²
    G_31 = 304,    // 304.11 m/s²
    G_32 = 314     // 313.92 m/s²
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

        LSR_Struct getElement(int index) const {
            if(index < 0 || index >= N) {
                return ring[ring_ptr];
            } else {
                return ring[index];
            }
            
        }

        LSR_Struct& operator[](int index) {
            if(index < 0 || index >= N) {
                return ring[ring_ptr];
            } else {
                return ring[index];
            }
        }

        LSR_Struct operator[](int index) const {
            if(index < 0 || index >= N) {
                return ring[ring_ptr];
            } else {
                return ring[index];
            }
        }
};
