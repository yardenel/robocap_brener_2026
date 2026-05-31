// ============================================================================
//  robot_protocol.h   ―   RoboCap 2026  shared protocol  (v3.0)
// ----------------------------------------------------------------------------
//  SINGLE SOURCE OF TRUTH. Keep this file IDENTICAL on:
//    - the 4 camera ESP32-S3 firmware images (robocap_esp32s3_v3.ino)
//    - the Teensy 4.1 main firmware
//    - the teammate robot (same team)
//
//  v3.0 changes vs the v2.2 era:
//    * Consolidated: every constant the firmware uses now lives HERE, so the
//      .ino just #includes this file (no duplicated #defines that can drift).
//    * Removed the unused CamToTeensy struct / 921600 baud / binary CAL_* /
//      CalReport / VIS_* / LINE_DIST_* block — the running firmware never used
//      it (it uses the compact 4-byte detection packet + ASCII CAL: instead).
//    * ADDED the full TEST-mode layer (SoftAP + Web console + relay to robot B).
//
//  Wire summary (forward ESP <-> Teensy share ONE UART, dual-mode):
//    bytes >= 0x80  -> BINARY  (commands / events, handled immediately)
//    bytes 0x01..0x04 as byte[0] -> DETECTION packet from a camera ESP
//    bytes 0x20..0x7E + \n       -> ASCII line (CAL: , test cmds, TLM:, OK:)
// ============================================================================
#pragma once
#include <stdint.h>

// ============================================================================
//  1.  UART  (camera ESP  <->  Teensy 4.1)
// ============================================================================
#define UART_BAUD            115200      // matches Serial.begin() in firmware
#define PACKET_NO_DETECT     0xFF        // byte[1] sentinel: nothing detected

// ----- Detection packet : ESP -> Teensy ------------------------------------
//  Detected (4 bytes):  [ESP_ID][objType][angle int8 cam-local][dist 0..255]
//  No detect (2 bytes):  [ESP_ID][0xFF]
//  angle is CAMERA-LOCAL (-60..+60). Teensy adds MOUNT[espID] then wraps.
#define OBJ_GOAL             0x00        // colored goal (yellow OR blue)
#define OBJ_LINE_WHITE       0x01        // white boundary / penalty line
#define OBJ_LINE_BLACK       0x02        // black center-circle / neutral marker

// ----- ESP unique IDs (byte[0] of every detection packet) ------------------
#define ESP_ID_FRONT         0x01
#define ESP_ID_RIGHT         0x02
#define ESP_ID_REAR          0x03
#define ESP_ID_LEFT          0x04

// Mount angle lookup, indexed by ESP id (Teensy: robotAngle = camAngle+MOUNT[id])
//   const int MOUNT[5] = {0, 0, 90, 180, 270};  // index 0 unused

// ============================================================================
//  2.  Binary commands : Teensy -> ESP   (all >= 0x80, never collide w/ ASCII)
// ============================================================================
#define CMD_QUERY_ID         0xA0        // -> ESP replies its ESP_ID byte
#define CMD_WIFI_START       0xA1        // start ESP-NOW bridge (override auto)
#define CMD_WIFI_STOP        0xA2        // stop ESP-NOW bridge
#define CMD_WIFI_STATUS      0xA3        // -> ESP replies partnerID (0=unpaired)
#define CMD_ROBOT_STATE      0xA4        // [v3 NEW] +1 byte ROBOT_STATE_* below
#define CMD_RELAY_DATA       0xC0        // +<len>+<bytes> -> forward to partner

// ----- ESP -> Teensy async events (between detection packets) ---------------
#define EVT_PARTNER_FOUND    0xB0        // +1 byte partnerID
#define EVT_WIFI_DATA        0xB1        // +<len>+<bytes> (data from partner)

// ----- Robot state (payload of CMD_ROBOT_STATE) ----------------------------
//  The Teensy is the authority. It pushes its state to the forward ESP so the
//  ESP can gate the TEST SoftAP (up only in READY/TEST) and reject commands
//  during a live game. Mirrors the Teensy system state machine.
#define ROBOT_STATE_READY    0           // == STANDBY: powered, waiting for GO
#define ROBOT_STATE_GAME     1           // == RUNNING: pin9 GO, autonomous play
#define ROBOT_STATE_TEST     2           // manual test console active

// ============================================================================
//  3.  ASCII calibration protocol : Teensy -> ESP   ('\n' terminated)
//      (unchanged from v2.2 — the camera HSV thresholds live in ESP NVS)
// ============================================================================
//   CAL:YELLOW:hMin,hMax,sMin,sMax,vMin,vMax
//   CAL:BLUE:hMin,hMax,sMin,sMax,vMin,vMax
//   CAL:WHITE:sMax,vMin
//   CAL:BLACK:sMax,vMax
//   CAL:SAVE | CAL:LOAD | CAL:RESET | CAL:STATUS
//  ESP replies (ASCII): OK:<cmd> | ERR:<reason> | INFO:<msg> | WARN:<msg>

// ============================================================================
//  4.  ESP-NOW  (forward ESP <-> teammate forward ESP)
// ============================================================================
#define COMM_MAGIC           0x5243      // "RC"
#define COMM_VERSION         1
#define MY_TEAM_ID           0xA7        // CHANGE per team to avoid clashes
#define COMM_CHANNEL         1           // {1,6,11}; BOTH robots + SoftAP match!

// Discovery handshake (ASCII over ESP-NOW): "WHO_AM_I:<id>" -> "ROBOT_ID:<id>"

enum RobotRole : uint8_t {
  ROLE_ATTACKER = 0,
  ROLE_GOALIE   = 1,
  ROLE_UNKNOWN  = 255
};

// Inter-robot game-data flags
#define ENMSG_BALL_VISIBLE   (1 << 0)
#define ENMSG_HAS_BALL       (1 << 1)
#define ENMSG_YGOAL_VISIBLE  (1 << 2)
#define ENMSG_BGOAL_VISIBLE  (1 << 3)
#define ENMSG_NEAR_LINE      (1 << 4)

// Application-layer payload the Teensy relays robot<->robot (transparent via ESP)
struct __attribute__((packed)) RobotMsg {
  uint16_t magic;            // COMM_MAGIC
  uint8_t  version;          // COMM_VERSION
  uint8_t  team_id;          // MY_TEAM_ID
  uint8_t  robot_id;         // 1 or 2 within the team
  uint8_t  flags;            // ENMSG_* bitmap
  uint32_t seq;
  uint32_t tx_millis;
  RobotRole role;
  uint8_t   battery_pct;
  int16_t  ball_angle;       // robot-relative deg*10
  uint16_t ball_radius_px;
  int16_t  yellow_goal_angle;
  int16_t  blue_goal_angle;
  int16_t  field_x_cm;
  int16_t  field_y_cm;
  uint8_t  reserved[2];
};

// ============================================================================
//  5.  TEST-MODE LAYER   [v3 NEW]   (forward ESP only)
// ============================================================================

// ----- SoftAP (smartphone TEST console) ------------------------------------
//  SSID = TEST_AP_PREFIX + last 4 hex of this ESP's MAC  -> e.g. "RCap_9F3A"
//  so two robots are distinguishable from the phone's WiFi list.
//  Brought up ONLY in READY/TEST, on COMM_CHANNEL so ESP-NOW keeps working.
#define TEST_AP_PREFIX       "RCap_"
#define TEST_AP_PASSWORD     "1q2w3e4r"   // >=8 chars (WPA2). Change before comp.
#define TEST_AP_CHANNEL      COMM_CHANNEL  // must equal ESP-NOW channel
#define TEST_HTTP_PORT       80
#define TEST_IDLE_TIMEOUT_MS 600000        // 10 min no activity -> back to READY

// ----- ASCII test commands : forward ESP -> Teensy  ('\n' terminated) -------
//  Emitted by the ESP when the phone (or relay from robot A) issues an action.
//  The Teensy executes ONLY while its state == ROBOT_STATE_TEST (the gate).
//
//    TEST:ON                 enter TEST (Teensy accepts only from READY)
//    TEST:OFF                exit TEST -> READY
//    ESTOP                   kill motors + dribbler off + kicker disarm
//    MOTOR:n:dir:pwm         n=1..4  dir=0/1  pwm=0..100   (single-motor test)
//    OMNI:vx:vy:r            each -100..100  (% of max; omni drive vector)
//    KICK:pct                pct = 30 | 50 | 70 | 100
//    DRIBBLER:on             on = 0 | 1
//    GOAL_LOCK:color         color = yellow | blue   (rotate-to-center + save)
//    IR:RAW                  request 20 IR values + U1   -> Teensy emits IR:...
//    COMPASS:READ            request heading            -> Teensy emits CMP:...
//    COMPASS:CAL_START       begin 360 deg compass cal
//    COMPASS:CAL_STOP        finish + save compass cal
//    QUERY:STATUS            request Status-tab data     -> Teensy emits TLM:/STA:
//
//  Confirmation for physical actions (MOTOR/OMNI/KICK) is enforced in the Web
//  UI before the /cmd is issued (Rule-safe, ch.14.5).

// ----- ASCII telemetry : Teensy -> forward ESP  ('\n' terminated) -----------
//  The ESP parses these into JSON and pushes them to the phone over SSE.
//
//    TLM:state,batt,ballVis,ballAng,ballDist,pocket,line,heading,goalBearing
//         state 0/1/2  batt 0..100  ballVis 0/1  ballAng -1..359  ballDist 0..255
//         pocket 0/1 (U1)  line 0/1  heading 0..359  goalBearing -1..359
//    IR:v0,v1,...,v19,u1            20 IR raw + U1 boolean (reply to IR:RAW)
//    VIS:cam,yAng,bAng,wAng,kAng    per-camera vision (cam 0..3; -1 if not seen)
//    CMP:heading,calib              compass live + calib status (0/1)
//    STA:batt,id1,id2,id3,id4,partnerID,partnerMAC,uptime   Status tab
//    LOG:<text>                     free-text line shown in the UI log
//    ACK:<text>                     acknowledgement of an action

// ----- ESP-NOW TEST relay (control robot B from robot A's phone) ------------
//  A distinct 2-byte magic so the receiver never confuses a TEST relay with
//  ordinary transparent game-data relay (CMD_RELAY_DATA path).
//    Frame: [0x70][0x54][type][ascii payload ...]
//      type 0 = command  (A -> B): payload is one ASCII test command line
//      type 1 = response (B -> A): payload is one ASCII telemetry/ACK line
#define RELAY_MAGIC0         0x70        // 'p'
#define RELAY_MAGIC1         0x54        // 'T'
#define RELAY_TYPE_CMD       0
#define RELAY_TYPE_RESP      1
#define RELAY_MAX_PAYLOAD    58          // 64 ESP-NOW max - 6 header/safety

// ----- ESP-NOW partner "game started" notice -------------------------------
//  If the partner enters GAME while we are in TEST, we must drop TEST too
//  (ch.14.5). Sent as ASCII over ESP-NOW: "GAME_STARTED"
#define MSG_GAME_STARTED     "GAME_STARTED"

// ============================================================================
//  6.  PCB strap mount-angle encoding (forward = 0,0 = auto WiFi/TEST host)
// ============================================================================
//   GPIO1=strap A (LSB), GPIO2=strap B (MSB); INPUT_PULLUP, GND-short = 1
//     open,open -> 0deg Forward  (auto-hosts ESP-NOW + TEST SoftAP)
//     open,GND  -> 90deg Right
//     GND,open  -> 180deg Rear
//     GND,GND   -> 270deg Left
