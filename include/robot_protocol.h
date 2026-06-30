// ============================================================================
/*  robot_protocol.h   ―   RoboCap 2026 shared protocol (v3.1 camera-ball)
    Shared by:
      - Teensy 4.1 main firmware
      - XIAO ESP32-S3 camera firmware

    Wire summary:
      ESP -> Teensy detection packet:
        detected:  [ESP_ID][objType][angle int8 camera-local][distance 0..255]
        no detect: [ESP_ID][0xFF]

      angle is camera-local (-60..+60). Teensy adds MOUNT[espID].
*/
// ============================================================================
#pragma once
#include <stdint.h>

// ============================================================================
//  1. UART camera protocol
// ============================================================================
#define UART_BAUD            115200
#define PACKET_NO_DETECT     0xFF

// Detected objects
#define OBJ_GOAL             0x00
#define OBJ_LINE_WHITE       0x01
#define OBJ_LINE_BLACK       0x02
#define OBJ_GOAL_YELLOW      0x03
#define OBJ_GOAL_BLUE        0x04
#define OBJ_BALL             0x05      // [v3.1] orange ball from camera

static inline bool isGoalType(uint8_t t) {
  return t == OBJ_GOAL || t == OBJ_GOAL_YELLOW || t == OBJ_GOAL_BLUE;
}
static inline bool isBallType(uint8_t t) {
  return t == OBJ_BALL;
}

// Camera IDs / physical positions
#define ESP_ID_FRONT         0x01
#define ESP_ID_RIGHT         0x02
#define ESP_ID_REAR          0x03
#define ESP_ID_LEFT          0x04

// ============================================================================
//  2. Binary commands Teensy -> ESP
// ============================================================================
#define CMD_QUERY_ID         0xA0
#define CMD_WIFI_START       0xA1
#define CMD_WIFI_STOP        0xA2
#define CMD_WIFI_STATUS      0xA3
#define CMD_ROBOT_STATE      0xA4
#define CMD_RELAY_DATA       0xC0

#define EVT_PARTNER_FOUND    0xB0
#define EVT_WIFI_DATA        0xB1

#define ROBOT_STATE_READY    0
#define ROBOT_STATE_GAME     1
#define ROBOT_STATE_TEST     2

// ============================================================================
//  3. ASCII calibration protocol
// ============================================================================
// CAL:BALL:hMin,hMax,sMin,sMax,vMin,vMax
// CAL:YELLOW:hMin,hMax,sMin,sMax,vMin,vMax
// CAL:BLUE:hMin,hMax,sMin,sMax,vMin,vMax
// CAL:WHITE:sMax,vMin
// CAL:BLACK:sMax,vMax
// CAL:SAVE | CAL:LOAD | CAL:RESET | CAL:STATUS

// ============================================================================
//  4. ESP-NOW / teammate protocol placeholders kept for compatibility
// ============================================================================
#define COMM_MAGIC           0x5243
#define COMM_VERSION         1
#define MY_TEAM_ID           0xA7
#define COMM_CHANNEL         1

enum RobotRole : uint8_t {
  ROLE_ATTACKER = 0,
  ROLE_GOALIE   = 1,
  ROLE_UNKNOWN  = 255
};

#define ENMSG_BALL_VISIBLE   (1 << 0)
#define ENMSG_HAS_BALL       (1 << 1)
#define ENMSG_YGOAL_VISIBLE  (1 << 2)
#define ENMSG_BGOAL_VISIBLE  (1 << 3)
#define ENMSG_NEAR_LINE      (1 << 4)

struct __attribute__((packed)) RobotMsg {
  uint16_t magic;
  uint8_t  version;
  uint8_t  team_id;
  uint8_t  robot_id;
  uint8_t  flags;
  uint32_t seq;
  uint32_t tx_millis;
  RobotRole role;
  uint8_t   battery_pct;
  int16_t  ball_angle;
  uint16_t ball_radius_px;
  int16_t  yellow_goal_angle;
  int16_t  blue_goal_angle;
  int16_t  field_x_cm;
  int16_t  field_y_cm;
  uint8_t  reserved[2];
};

// ============================================================================
//  5. TEST layer constants kept so older webapp/ESP code can still include this
// ============================================================================
#define TEST_AP_PREFIX       "RCap_"
#define TEST_AP_PASSWORD     "1q2w3e4r"
#define TEST_AP_CHANNEL      COMM_CHANNEL
#define TEST_HTTP_PORT       80
#define TEST_IDLE_TIMEOUT_MS 600000

#define RELAY_MAGIC0         0x70
#define RELAY_MAGIC1         0x54
#define RELAY_TYPE_CMD       0
#define RELAY_TYPE_RESP      1
#define RELAY_MAX_PAYLOAD    58
#define MSG_GAME_STARTED     "GAME_STARTED"
