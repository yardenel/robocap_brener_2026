// robot_protocol.h
// Shared between camera ESPs, Teensy, and the teammate robot.
// Keep this file IDENTICAL on Teensy and ESP firmware.
#pragma once
#include <stdint.h>

// ============================================================================
//   UART packet: Camera ESP --> Teensy
// ============================================================================
//   Sent at ~30 Hz by every camera ESP on its own UART line.
//   All angles are ALREADY in robot-relative coordinates (ESP applies mount
//   offset before TX), so Teensy can fuse the four streams directly.
//
//   Wire format: START(0xAA) | LEN | payload[LEN] | XOR_CHECKSUM
// ============================================================================

#define UART_START_BYTE        0xAA
#define UART_BAUD              921600        // Teensy 4.1 has no problem at this rate

#define BALL_NOT_SEEN          INT16_MIN
#define GOAL_NOT_SEEN          INT16_MIN
#define LINE_NOT_SEEN          INT16_MIN

// Bit flags for "what's visible right now"
#define VIS_BALL               (1 << 0)
#define VIS_YELLOW_GOAL        (1 << 1)
#define VIS_BLUE_GOAL          (1 << 2)
#define VIS_WHITE_LINE         (1 << 3)   // line seen at any distance
#define VIS_LINE_CLOSE         (1 << 4)   // line is in the "act now" band

// Distance bands for the white line — coarse, derived by ESP from line position
// in frame. Teensy uses these for TACTICAL decisions (kick now / slow / orient).
// The HARD STOP is still TCS34725 on Teensy — these are EARLY WARNING only.
#define LINE_DIST_NONE         0
#define LINE_DIST_FAR          1          // line visible, plenty of room
#define LINE_DIST_MID          2          // closing in — consider kicking
#define LINE_DIST_NEAR         3          // very close — TCS may fire any moment

struct __attribute__((packed)) CamToTeensy {
  uint8_t  start;            // 0xAA
  uint8_t  len;              // sizeof(payload), excludes start/len/checksum
  uint8_t  cam_id;           // 0=front, 1=right, 2=rear, 3=left
  uint8_t  mount_deg_div10;  // 0, 9, 18, 27 (i.e., angle/10 to fit in uint8)
  uint32_t frame_seq;        // monotonic, helps Teensy spot frozen cameras

  uint8_t  visible_flags;    // VIS_* bitmap

  // All angles in DEGREES * 10, robot-relative (0=front of robot, +CW)
  // Range -1800..1800, *_NOT_SEEN if absent
  int16_t  ball_angle;
  uint16_t ball_radius_px;   // apparent size, used by Teensy to estimate distance

  int16_t  yellow_goal_angle;
  uint16_t yellow_goal_width_px;

  int16_t  blue_goal_angle;
  uint16_t blue_goal_width_px;

  // --- White line: early-warning, NOT hard-stop ---
  // Line appears in the lower portion of the frame (camera looks slightly down).
  // "Closer line" = lower in the frame = larger row_y. We report:
  //   line_angle    : robot-relative direction to line centroid
  //   line_dist_band: coarse band so Teensy doesn't need pixel math
  //   line_row_y    : raw row index of line centroid (0=top, FRAME_H=bottom)
  int16_t  line_angle;
  uint8_t  line_dist_band;   // LINE_DIST_*
  uint8_t  line_row_y;       // 0..120; bottom of frame = closest

  uint8_t  ball_confidence;  // 0..100
  uint8_t  checksum;         // XOR of all bytes from cam_id .. ball_confidence
};

// ============================================================================
//   UART command: Teensy --> camera ESP
// ============================================================================

#define CMD_START_BYTE         0xBB
#define CMD_SET_TEAM_STATE     0x01     // payload = TeamStatePayload (ID=0 only)
#define CMD_SET_TX_POWER       0x03     // payload = uint8_t qdBm

// Calibration commands
#define CMD_CAL_BEGIN          0x10     // payload = uint8_t target (CAL_TARGET_*)
                                        //   ESP samples center ROI for ~1 sec,
                                        //   computes HSV stats, replies with
                                        //   CalReport. New thresholds become
                                        //   active immediately but NOT persisted
                                        //   until CMD_CAL_COMMIT.
#define CMD_CAL_COMMIT         0x11     // payload = empty
                                        //   Persist current active thresholds to NVS
#define CMD_CAL_RESET          0x12     // payload = empty
                                        //   Restore factory defaults + erase NVS
#define CMD_CAL_DUMP           0x13     // payload = empty
                                        //   ESP replies with CalReport for each
                                        //   target showing currently active ranges

// Calibration targets — what's currently filling the center of the camera frame
enum CalTarget : uint8_t {
  CAL_TARGET_BALL        = 0,           // orange ball held in center
  CAL_TARGET_YELLOW_GOAL = 1,           // robot pointed at yellow goal
  CAL_TARGET_BLUE_GOAL   = 2,           // robot pointed at blue goal
  CAL_TARGET_WHITE_LINE  = 3,           // looking down at a white line strip
  CAL_TARGET_FIELD       = 4,           // plain field carpet (debug only)
  CAL_TARGET_COUNT       = 5
};

// ============================================================================
//   Reply: Camera ESP --> Teensy (calibration results)
// ============================================================================
//   Sent prefixed with CAL_REPORT_START so Teensy parser can tell it apart
//   from regular CamToTeensy (0xAA) and teammate-forward (0xCC) frames.
// ============================================================================

#define CAL_REPORT_START       0xDD

#define CAL_STATUS_OK              0
#define CAL_STATUS_TOO_FEW_PIXELS  1
#define CAL_STATUS_UNSATURATED     2     // for chromatic targets, S too low
#define CAL_STATUS_OVEREXPOSED     3
#define CAL_STATUS_NVS_FAILED      4

struct __attribute__((packed)) CalReport {
  uint8_t  start;                       // 0xDD
  uint8_t  cam_id;
  uint8_t  target;                      // CalTarget value
  uint8_t  status;                      // CAL_STATUS_*
  uint8_t  h_min, h_max;
  uint8_t  s_min, s_max;
  uint8_t  v_min, v_max;
  uint16_t sample_count;                // pixels included in stats
  uint8_t  checksum;                    // XOR of cam_id..sample_count_high
};

struct __attribute__((packed)) TeamStatePayload {
  uint8_t  role;             // 0=attacker, 1=goalie, 255=unknown
  uint8_t  has_ball;         // 0 or 1 (from Teensy's dribbler/IR)
  int16_t  field_x_cm;       // estimated position; 0 if unknown
  int16_t  field_y_cm;
  uint8_t  battery_pct;      // 0..100
};

struct __attribute__((packed)) TeensyToCam {
  uint8_t  start;            // 0xBB
  uint8_t  cmd;
  uint8_t  len;
  uint8_t  payload[16];      // up to 16 bytes
  uint8_t  checksum;         // XOR of cmd..last payload byte
};

// ============================================================================
//   ESP-NOW message: ID=0 camera ESP <--> teammate robot's ID=0 camera ESP
// ============================================================================

#define COMM_MAGIC             0x5243         // "RC"
#define COMM_VERSION           1
#define MY_TEAM_ID             0xA7           // CHANGE per team to avoid clashes
#define COMM_CHANNEL           1              // pick from {1, 6, 11}, both robots match

enum RobotRole : uint8_t {
  ROLE_ATTACKER = 0,
  ROLE_GOALIE   = 1,
  ROLE_UNKNOWN  = 255
};

#define ENMSG_BALL_VISIBLE     (1 << 0)
#define ENMSG_HAS_BALL         (1 << 1)
#define ENMSG_YGOAL_VISIBLE    (1 << 2)
#define ENMSG_BGOAL_VISIBLE    (1 << 3)
#define ENMSG_NEAR_LINE        (1 << 4)   // "I'm near a line, may slow/stop soon"

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

  // World-as-seen-by-front-camera (robot-relative, deg*10)
  int16_t  ball_angle;
  uint16_t ball_radius_px;
  int16_t  yellow_goal_angle;
  int16_t  blue_goal_angle;

  int16_t  field_x_cm;       // from Teensy, optional
  int16_t  field_y_cm;

  uint8_t  reserved[2];
};
