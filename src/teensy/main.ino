#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "robot_protocol.h"

// ============================================================================
// RoboCap Brenner 2026 - Teensy 4.1 main strategy
// Two-attacker RoboCupJunior Soccer strategy
//
// Hardware assumptions from the project/conversation:
// - 4 omni / X-drive wheels, front = camera 1 = dribbler side
// - 20 TS