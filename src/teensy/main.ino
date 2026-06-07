#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "robot_protocol.h"

// RoboCap Brenner 2026 - Teensy 4.1 compact two-attacker strategy
// Front = camera 1 = dribbler side = 0 deg. IR front is between sensors 1 and 2.

// ---------------- Pins ----------------
#define ENG1_DR1 13
#define ENG1_DR2 41
#define ENG1_SP  23
#define ENG2_DR1