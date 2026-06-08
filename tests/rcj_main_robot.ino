/*
 * ============================================================================
 * RoboCupJunior Soccer 2026 — Teensy 4.1 Main Strategy
 * Team Brenner — TWO ATTACKERS strategy
 * ----------------------------------------------------------------------------
 * Hardware assumed from project:
 * - Teensy 4.1
 * - 4 omni / X-drive wheels
 * - 20 TSOP IR ball sensors around 360 degrees
 * - Dribbler centered between IR sensors 1 and 2
 * - Camera 1/front exactly above dribbler = 0 degrees
 * - Other ESP cameras at 90, 180, 270 degrees
 * - BNO055 gyro/IMU
 * - Kicker
 * - TCS34725 / line sensor support placeholder
 *
 * Main priorities:
 * 1. RCJ stop signal / emergency stop
 * 2. White line escape
 * 3. If ball in dribbler pocket -> attack goal
 * 4. If ball visible -> chase/capture
 * 5. If teammate has ball -> support attacker position
 * 6. Search ball
 *
 * This file is intentionally standalone.
 * Paste as: src/teensy/main.ino
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// If your repo already has robot_protocol.h in include path, keep this.
// Otherwise comment it out and use the fallback definitions below.
#include "robot_protocol.h"

// ============================================================================
// CONFIG
// ============================================================================

#define THIS_ROBOT_ID 1

// Set this according to side before match.
// true  = shoot to blue goal
// false = shoot to yellow goal
#define TARGET_GOAL_BLUE true

// RCJ start/stop module
#define PIN_RCJ_SIGNAL 9

// Dribbler / kicker
#define DRIBBLER_PIN 11
#define KICKER_PIN   2

// If you have a real pocket sensor, set correct pin here.
// If no sensor yet, code estimates pocket from IR sensors 0+1.
#define POCKET_PIN   12
#define USE_POCKET_SENSOR 0

// Motor pins
#define ENG1_DR1 13
#define ENG1_DR2 41
#define ENG1_SP  23

#define ENG2_DR1 40
#define ENG2_DR2 39
#define ENG2_SP  22

#define ENG3_DR1 38
#define ENG3_DR2 35
#define ENG3_SP  37

#define ENG4_DR1 34
#define ENG4_DR2 33
#define ENG4_SP  36

// IR MUX
#define MUX_S0  31
#define MUX_S1  30
#define MUX_S2  29
#define MUX_S3  28
#define MUX_SIG 32

const uint8_t DIRECT_PINS[4] = {24, 25, 26, 27};

#define NUM_IR_SENSORS 20
#define NUM_MUX_CH     16

// Camera UARTs
#define CAM_FRONT Serial1
#define CAM_RIGHT Serial2
#define CAM_REAR  Serial3
#define CAM_LEFT  Serial4

#ifndef UART_BAUD
#define UART_BAUD 115200
#endif

// Detection protocol from robot_protocol.h fallback
#ifndef PACKET_NO_DETECT
#define PACKET_NO_DETECT 0xFF
#define OBJ_GOAL         0x00
#define OBJ_LINE_WHITE   0x01
#define OBJ_LINE_BLACK   0x02
#define ESP_ID_FRONT     0x01
#define ESP_ID_RIGHT     0x02
#define ESP_ID_REAR      0x03
#define ESP_ID_LEFT      0x04
#endif

// Tuning
static constexpr float KP_HEADING = 0.018f;
static constexpr float KP_GOAL    = 0.025f;
static constexpr float KP_BALL_TURN = 0.010f;

static constexpr uint8_t DRIBBLER_PWM = 210;
static constexpr uint32_t KICKER_PULSE_MS = 70;
static constexpr uint32_t KICKER_COOLDOWN_MS = 1300;

static constexpr uint32_t IR_SCAN_PERIOD_MS = 35;
static constexpr uint32_t STRATEGY_PERIOD_MS = 20;
static constexpr uint32_t CAMERA_TIMEOUT_MS = 350;
static constexpr uint32_t PARTNER_TIMEOUT_MS = 600;

static constexpr float SEARCH_ROT = 0.36f;
static constexpr float CHASE_SPEED_FAST = 0.88f;
static constexpr float CHASE_SPEED_SLOW = 0.48f;
static constexpr float CAPTURE_SPEED = 0.42f;
static constexpr float ATTACK_SPEED = 0.78f;
static constexpr float SUPPORT_SPEED = 0.45f;
static constexpr float LINE_ESCAPE_SPEED = 0.88f;

// IMPORTANT:
// You said the dribbler is centered between IR sensors 1 and 2.
// In zero-based array that is sensors 0 and 1.
// Therefore front center is 9 degrees between them.
const float IR_SENSOR_ANGLES[NUM_IR_SENSORS] = {
   9,  27,  45,  63,  81,
  99, 117, 135, 153, 171,
 189, 207, 225, 243, 261,
 279, 297, 315, 333, 351
};

// ============================================================================
// TYPES
// ============================================================================

enum class SystemMode : uint8_t {
  READY,
  GAME,
  PAUSED,
  TEST,
  FAULT
};

enum class StrategyState : uint8_t {
  SEARCH_BALL,
  CHASE_BALL,
  CAPTURE_BALL,
  ATTACK_GOAL,
  SUPPORT_ATTACKER,
  AVOID_LINE,
  RECOVER
};

enum class AttackRole : uint8_t {
  PRIMARY_ATTACKER,
  SUPPORT_ATTACKER,
  SOLO_ATTACKER
};

struct CameraState {
  bool connected = false;
  bool goalVisible = false;
  bool whiteLineVisible = false;
  bool blackLineVisible = false;

  int goalAngle = 0;       // robot-relative degrees, -180..180
  uint8_t goalDist = 0;    // 0 far, 255 close/proxy
  int lineAngle = 0;       // robot-relative degrees, -180..180
  uint8_t lineDist = 0;
  uint32_t lastUpdate = 0;
};

struct TeamState {
  bool valid = false;
  uint8_t partnerId = 0;
  bool partnerBallVisible = false;
  bool partnerHasBall = false;
  bool partnerNearLine = false;
  int partnerBallAngle = 0;
  RobotRole partnerRole = ROLE_UNKNOWN;
  uint32_t lastRx = 0;
};

// ============================================================================
// GLOBALS
// ============================================================================

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_A, &Wire1);

volatile bool g_rcjRun = false;
volatile uint32_t g_rcjLastChange = 0;

SystemMode systemMode = SystemMode::READY;
StrategyState state = StrategyState::SEARCH_BALL;
AttackRole attackRole = AttackRole::SOLO_ATTACKER;

bool gyroOK = false;
float headingDeg = 0.0f;
float targetHeadingDeg = 0.0f;

// IR
bool irSensors[NUM_IR_SENSORS] = {};
volatile bool directDetected[4] = {};
float ballAngleDeg = -1.0f;  // 0..360, -1 = no ball
int ballCount = 0;
bool ballVisible = false;
bool hasBallPocket = false;

// Cameras
CameraState cams[4];

// Team
TeamState team;
uint32_t msgSeq = 0;

// Timing
uint32_t lastIRScan = 0;
uint32_t lastStrategyTick = 0;
uint32_t lineEscapeStart = 0;
uint32_t recoverStart = 0;

// Kicker
bool kickerActive = false;
uint32_t kickerStart = 0;
uint32_t lastKick = 0;

// ============================================================================
// SMALL MATH HELPERS
// ============================================================================

float wrap360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;
  return a;
}

float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float degToRad(float deg) {
  return deg * DEG_TO_RAD;
}

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int clampPwm(int v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return v;
}

// ============================================================================
// MOTOR / X DRIVE
// ============================================================================

void setMotor(int dr1, int dr2, int pwmPin, int speed) {
  speed = clampPwm(speed);

  if (speed > 0) {
    digitalWrite(dr1, HIGH);
    digitalWrite(dr2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(dr1, LOW);
    digitalWrite(dr2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(dr1, LOW);
    digitalWrite(dr2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void xDrive(float vx, float vy, float omega) {
  vx = clampFloat(vx, -1.0f, 1.0f);
  vy = clampFloat(vy, -1.0f, 1.0f);
  omega = clampFloat(omega, -1.0f, 1.0f);

  float vFL = vy + vx + omega;
  float vFR = vy - vx - omega;
  float vRL = vy - vx + omega;
  float vRR = vy + vx - omega;

  float m = max(max(fabsf(vFL), fabsf(vFR)), max(fabsf(vRL), fabsf(vRR)));
  if (m < 1.0f) m = 1.0f;

  int pwmFL = (int)(vFL / m * 255.0f);
  int pwmFR = (int)(vFR / m * 255.0f);
  int pwmRL = (int)(vRL / m * 255.0f);
  int pwmRR = (int)(vRR / m * 255.0f);

  setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, pwmFL);
  setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, pwmFR);
  setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, pwmRL);
  setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, pwmRR);
}

void stopDrive() {
  xDrive(0, 0, 0);
}

// ============================================================================
// DRIBBLER / KICKER
// ============================================================================

void dribblerSet(bool on) {
  analogWrite(DRIBBLER_PIN, on ? DRIBBLER_PWM : 0);
}

void kickerUpdate() {
  if (!kickerActive) return;

  if (millis() - kickerStart >= KICKER_PULSE_MS) {
    digitalWrite(KICKER_PIN, LOW);
    kickerActive = false;
    lastKick = millis();
  }
}

void kick() {
  uint32_t now = millis();

  if (kickerActive) return;
  if (now - lastKick < KICKER_COOLDOWN_MS) return;
  if (systemMode != SystemMode::GAME) return;

  digitalWrite(KICKER_PIN, HIGH);
  kickerActive = true;
  kickerStart = now;
}

void emergencyStopAll() {
  stopDrive();
  dribblerSet(false);
  digitalWrite(KICKER_PIN, LOW);
  kickerActive = false;
}

// ============================================================================
// BNO055
// ============================================================================

void updateGyro() {
  if (!gyroOK) return;

  sensors_event_t event;
  bno.getEvent(&event);
  headingDeg = wrap360(event.orientation.x);
}

float headingHoldOmega(float desiredHeading) {
  if (!gyroOK) return 0.0f;

  float err = wrap180(desiredHeading - headingDeg);
  return clampFloat(err * KP_HEADING, -0.55f, 0.55f);
}

// ============================================================================
// IR SENSOR ARRAY
// ============================================================================

void setMuxChannel(uint8_t ch) {
  digitalWriteFast(MUX_S0, (ch >> 0) & 1);
  digitalWriteFast(MUX_S1, (ch >> 1) & 1);
  digitalWriteFast(MUX_S2, (ch >> 2) & 1);
  digitalWriteFast(MUX_S3, (ch >> 3) & 1);
}

bool waitForBurst(uint8_t pin, uint32_t timeoutUs) {
  uint32_t deadline = micros() + timeoutUs;
  while ((int32_t)(micros() - deadline) < 0) {
    if (!digitalReadFast(pin)) return true;  // TSOP active LOW
  }
  return false;
}

void FASTRUN isrDirect0() { directDetected[0] = true; }
void FASTRUN isrDirect1() { directDetected[1] = true; }
void FASTRUN isrDirect2() { directDetected[2] = true; }
void FASTRUN isrDirect3() { directDetected[3] = true; }

void scanIR() {
  bool results[NUM_IR_SENSORS] = {};

  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) directDetected[i] = false;
  interrupts();

  for (uint8_t ch = 0; ch < NUM_MUX_CH; ch++) {
    setMuxChannel(ch);
    delayMicroseconds(2);
    results[ch] = waitForBurst(MUX_SIG, 200);
  }

  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) {
    results[NUM_MUX_CH + i] = directDetected[i];
  }
  interrupts();

  memcpy(irSensors, results, sizeof(results));
}

void processIR() {
  float sinSum = 0.0f;
  float cosSum = 0.0f;
  ballCount = 0;

  for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
    if (!irSensors[i]) continue;

    float rad = degToRad(IR_SENSOR_ANGLES[i]);
    sinSum += sinf(rad);
    cosSum += cosf(rad);
    ballCount++;
  }

  if (ballCount > 0) {
    ballAngleDeg = atan2f(sinSum, cosSum) * RAD_TO_DEG;
    ballAngleDeg = wrap360(ballAngleDeg);
    ballVisible = true;
  } else {
    ballAngleDeg = -1.0f;
    ballVisible = false;
  }

#if USE_POCKET_SENSOR
  hasBallPocket = digitalReadFast(POCKET_PIN) == LOW;
#else
  // Dribbler is between IR sensors 0 and 1.
  // This is an estimate until real U1/pocket sensor is wired.
  bool frontPair = irSensors[0] && irSensors[1];
  bool frontCentered =
      ballVisible &&
      (ballAngleDeg <= 28.0f || ballAngleDeg >= 332.0f);
  hasBallPocket = frontPair && frontCentered && ballCount >= 4;
#endif
}

void updateIR() {
  uint32_t now = millis();
  if (now - lastIRScan < IR_SCAN_PERIOD_MS) return;

  lastIRScan = now;
  scanIR();
  processIR();
}

// ============================================================================
// CAMERA PARSER
// ESP packets from v3 protocol:
// Detected:  [ESP_ID][objType][angle int8 camera-local][dist]
// No detect: [ESP_ID][0xFF]
// ============================================================================

int cameraIndexFromId(uint8_t id) {
  switch (id) {
    case ESP_ID_FRONT: return 0;
    case ESP_ID_RIGHT: return 1;
    case ESP_ID_REAR:  return 2;
    case ESP_ID_LEFT:  return 3;
    default: return -1;
  }
}

int mountFromId(uint8_t id) {
  switch (id) {
    case ESP_ID_FRONT: return 0;
    case ESP_ID_RIGHT: return 90;
    case ESP_ID_REAR:  return 180;
    case ESP_ID_LEFT:  return 270;
    default: return 0;
  }
}

void handleCamPacket(uint8_t espId, uint8_t obj, int8_t camAngle, uint8_t dist) {
  int idx = cameraIndexFromId(espId);
  if (idx < 0) return;

  CameraState& c = cams[idx];
  c.connected = true;
  c.lastUpdate = millis();

  int robotAngle = (int)wrap180((float)camAngle + mountFromId(espId));

  if (obj == OBJ_GOAL) {
    c.goalVisible = true;
    c.goalAngle = robotAngle;
    c.goalDist = dist;
  } else if (obj == OBJ_LINE_WHITE) {
    c.whiteLineVisible = true;
    c.lineAngle = robotAngle;
    c.lineDist = dist;
  } else if (obj == OBJ_LINE_BLACK) {
    c.blackLineVisible = true;
  }
}

void readCameraPort(HardwareSerial& port) {
  while (port.available() >= 2) {
    uint8_t b0 = (uint8_t)port.peek();

    if (b0 < ESP_ID_FRONT || b0 > ESP_ID_LEFT) {
      port.read();
      continue;
    }

    uint8_t espId = (uint8_t)port.read();

    if (!port.available()) return;
    uint8_t obj = (uint8_t)port.read();

    int idx = cameraIndexFromId(espId);
    if (idx >= 0) {
      cams[idx].connected = true;
      cams[idx].lastUpdate = millis();
    }

    if (obj == PACKET_NO_DETECT) {
      continue;
    }

    while (port.available() < 2) {
      if (!port.available()) return;
    }

    int8_t angle = (int8_t)port.read();
    uint8_t dist = (uint8_t)port.read();

    handleCamPacket(espId, obj, angle, dist);
  }
}

void expireCameraData() {
  uint32_t now = millis();

  for (int i = 0; i < 4; i++) {
    if (now - cams[i].lastUpdate <= CAMERA_TIMEOUT_MS) continue;

    cams[i].goalVisible = false;
    cams[i].whiteLineVisible = false;
    cams[i].blackLineVisible = false;
  }
}

void updateCameras() {
  readCameraPort(CAM_FRONT);
  readCameraPort(CAM_RIGHT);
  readCameraPort(CAM_REAR);
  readCameraPort(CAM_LEFT);
  expireCameraData();
}

bool getBestGoal(int& angleOut, uint8_t& distOut) {
  bool found = false;
  uint8_t bestDist = 0;
  int bestAngle = 0;

  // Prefer front camera if it sees the goal, because it is above dribbler.
  if (cams[0].goalVisible) {
    angleOut = cams[0].goalAngle;
    distOut = cams[0].goalDist;
    return true;
  }

  for (int i = 1; i < 4; i++) {
    if (!cams[i].goalVisible) continue;

    if (!found || cams[i].goalDist > bestDist) {
      found = true;
      bestDist = cams[i].goalDist;
      bestAngle = cams[i].goalAngle;
    }
  }

  if (!found) return false;

  angleOut = bestAngle;
  distOut = bestDist;
  return true;
}

bool getLineThreat(int& escapeAngleOut) {
  bool threat = false;
  float sx = 0.0f;
  float sy = 0.0f;

  for (int i = 0; i < 4; i++) {
    if (!cams[i].whiteLineVisible) continue;

    threat = true;

    // Escape away from line direction.
    float escape = wrap360(cams[i].lineAngle + 180.0f);
    float rad = degToRad(escape);
    sx += sinf(rad);
    sy += cosf(rad);
  }

  if (!threat) return false;

  float a = atan2f(sx, sy) * RAD_TO_DEG;
  escapeAngleOut = (int)wrap360(a);
  return true;
}

// ============================================================================
// TEAM COMMS OVER FORWARD ESP UART RELAY
// Minimal support:
// - send RobotMsg through CMD_RELAY_DATA on front ESP UART
// - receive EVT_WIFI_DATA frames if ESP forwards them
// ============================================================================

void sendRobotMsg() {
  RobotMsg msg;
  memset(&msg, 0, sizeof(msg));

  msg.magic = COMM_MAGIC;
  msg.version = COMM_VERSION;
  msg.team_id = MY_TEAM_ID;
  msg.robot_id = THIS_ROBOT_ID;
  msg.seq = msgSeq++;
  msg.tx_millis = millis();

  msg.role =
      (attackRole == AttackRole::PRIMARY_ATTACKER) ? ROLE_ATTACKER :
      (attackRole == AttackRole::SUPPORT_ATTACKER) ? ROLE_ATTACKER :
      ROLE_ATTACKER;

  msg.battery_pct = 100;

  if (ballVisible) msg.flags |= ENMSG_BALL_VISIBLE;
  if (hasBallPocket) msg.flags |= ENMSG_HAS_BALL;

  int goalAng;
  uint8_t goalDist;
  if (getBestGoal(goalAng, goalDist)) {
#if TARGET_GOAL_BLUE
    msg.flags |= ENMSG_BGOAL_VISIBLE;
#else
    msg.flags |= ENMSG_YGOAL_VISIBLE;
#endif
  }

  int esc;
  if (getLineThreat(esc)) msg.flags |= ENMSG_NEAR_LINE;

  msg.ball_angle = ballVisible ? (int16_t)(wrap180(ballAngleDeg) * 10.0f) : -32768;
  msg.ball_radius_px = (uint16_t)constrain(ballCount * 20, 0, 65535);

  msg.yellow_goal_angle = 0;
  msg.blue_goal_angle = 0;

  CAM_FRONT.write((uint8_t)CMD_RELAY_DATA);
  CAM_FRONT.write((uint8_t)sizeof(RobotMsg));
  CAM_FRONT.write((uint8_t*)&msg, sizeof(RobotMsg));
}

void handlePartnerMsg(const uint8_t* data, uint8_t len) {
  if (len < sizeof(RobotMsg)) return;

  RobotMsg msg;
  memcpy(&msg, data, sizeof(RobotMsg));

  if (msg.magic != COMM_MAGIC) return;
  if (msg.version != COMM_VERSION) return;
  if (msg.team_id != MY_TEAM_ID) return;
  if (msg.robot_id == THIS_ROBOT_ID) return;

  team.valid = true;
  team.partnerId = msg.robot_id;
  team.partnerBallVisible = msg.flags & ENMSG_BALL_VISIBLE;
  team.partnerHasBall = msg.flags & ENMSG_HAS_BALL;
  team.partnerNearLine = msg.flags & ENMSG_NEAR_LINE;
  team.partnerBallAngle = msg.ball_angle / 10;
  team.partnerRole = msg.role;
  team.lastRx = millis();
}

void updateTeamComms() {
  static uint32_t lastTx = 0;

  // Parse async events from front ESP mixed into CAM_FRONT.
  // Note: if your camera and relay share the same Serial1, this simple parser
  // may need tightening. It is compatible with v3 protocol idea.
  while (CAM_FRONT.available() >= 1) {
    uint8_t b = (uint8_t)CAM_FRONT.peek();

    if (b == EVT_WIFI_DATA) {
      CAM_FRONT.read();

      if (!CAM_FRONT.available()) return;
      uint8_t len = (uint8_t)CAM_FRONT.read();

      uint8_t buf[80];
      if (len > sizeof(buf)) {
        for (uint8_t i = 0; i < len && CAM_FRONT.available(); i++) CAM_FRONT.read();
        continue;
      }

      uint32_t start = millis();
      while (CAM_FRONT.available() < len && millis() - start < 5) {}

      if (CAM_FRONT.available() >= len) {
        CAM_FRONT.readBytes(buf, len);
        handlePartnerMsg(buf, len);
      }
    } else {
      // Not team event; camera parser will handle normal ESP packets.
      break;
    }
  }

  if (millis() - team.lastRx > PARTNER_TIMEOUT_MS) {
    team.valid = false;
  }

  if (millis() - lastTx >= 50) {
    lastTx = millis();
    sendRobotMsg();
  }
}

// ============================================================================
// RCJ START/STOP
// ============================================================================

void FASTRUN isrRcjSignalChange() {
  uint32_t now = millis();
  if (now - g_rcjLastChange < 5) return;

  g_rcjLastChange = now;
  g_rcjRun = digitalReadFast(PIN_RCJ_SIGNAL);
}

void updateSystemMode() {
  static bool lastRun = false;

  bool nowRun = g_rcjRun;

  if (nowRun == lastRun) return;
  lastRun = nowRun;

  if (nowRun) {
    if (systemMode == SystemMode::READY || systemMode == SystemMode::PAUSED) {
      systemMode = SystemMode::GAME;
      targetHeadingDeg = headingDeg;
      state = StrategyState::SEARCH_BALL;
    } else if (systemMode == SystemMode::TEST) {
      emergencyStopAll();
      systemMode = SystemMode::GAME;
      targetHeadingDeg = headingDeg;
      state = StrategyState::SEARCH_BALL;
    }
  } else {
    if (systemMode == SystemMode::GAME) {
      emergencyStopAll();
      systemMode = SystemMode::PAUSED;
    }
  }
}

// ============================================================================
// ROLE AUCTION: TWO ATTACKERS, NO GOALIE
// ============================================================================

float myBallScore() {
  if (!ballVisible) return 0.0f;

  float rel = fabsf(wrap180(ballAngleDeg));
  float angleQuality = (rel < 25.0f) ? 1.4f :
                       (rel < 70.0f) ? 1.0f :
                       0.55f;

  float strength = constrain((float)ballCount / 10.0f, 0.1f, 1.4f);

  if (hasBallPocket) return 999.0f;

  return strength * angleQuality;
}

float partnerBallScore() {
  if (!team.valid || !team.partnerBallVisible) return 0.0f;
  if (team.partnerHasBall) return 998.0f;

  float rel = fabsf((float)team.partnerBallAngle);
  float angleQuality = (rel < 25.0f) ? 1.4f :
                       (rel < 70.0f) ? 1.0f :
                       0.55f;

  return angleQuality;
}

void updateAttackRole() {
  if (!team.valid) {
    attackRole = AttackRole::SOLO_ATTACKER;
    return;
  }

  float myScore = myBallScore();
  float pScore = partnerBallScore();

  // If partner has ball, we support.
  if (team.partnerHasBall && !hasBallPocket) {
    attackRole = AttackRole::SUPPORT_ATTACKER;
    return;
  }

  // If we have ball, we primary.
  if (hasBallPocket) {
    attackRole = AttackRole::PRIMARY_ATTACKER;
    return;
  }

  // Two attackers: higher score goes primary; lower score supports/runs rebound.
  if (myScore >= pScore) attackRole = AttackRole::PRIMARY_ATTACKER;
  else attackRole = AttackRole::SUPPORT_ATTACKER;
}

// ============================================================================
// STRATEGY BEHAVIORS
// ============================================================================

void vectorDriveAngle(float angleDeg, float speed, float omega) {
  float rad = degToRad(angleDeg);
  float vx = sinf(rad) * speed;
  float vy = cosf(rad) * speed;
  xDrive(vx, vy, omega);
}

void enterLineEscape() {
  state = StrategyState::AVOID_LINE;
  lineEscapeStart = millis();
  dribblerSet(false);
}

void behaviorAvoidLine() {
  int escapeAngle = 180;
  getLineThreat(escapeAngle);

  uint32_t elapsed = millis() - lineEscapeStart;

  if (elapsed < 380) {
    vectorDriveAngle(escapeAngle, LINE_ESCAPE_SPEED, 0);
    return;
  }

  if (elapsed < 620) {
    xDrive(0, 0, 0.45f);
    return;
  }

  state = StrategyState::RECOVER;
  recoverStart = millis();
}

void behaviorRecover() {
  dribblerSet(false);

  if (millis() - recoverStart < 250) {
    xDrive(0, 0, headingHoldOmega(targetHeadingDeg));
    return;
  }

  state = StrategyState::SEARCH_BALL;
}

void behaviorSearchBall() {
  dribblerSet(false);

  // If partner sees ball, rotate/slide toward estimated direction.
  if (team.valid && team.partnerBallVisible && !ballVisible) {
    float a = (float)team.partnerBallAngle;
    vectorDriveAngle(a, 0.25f, 0.25f);
    return;
  }

  // Search pattern: slow rotation + tiny forward drift.
  xDrive(0.0f, 0.10f, SEARCH_ROT);
}

void behaviorChaseBall() {
  dribblerSet(false);

  if (!ballVisible) {
    state = StrategyState::SEARCH_BALL;
    return;
  }

  float rel = wrap180(ballAngleDeg);

  // If ball is centered and strong, go capture.
  if (fabsf(rel) < 22.0f && ballCount >= 4) {
    state = StrategyState::CAPTURE_BALL;
    return;
  }

  float speed = (ballCount >= 8) ? CHASE_SPEED_SLOW : CHASE_SPEED_FAST;

  // Use x-drive advantage: move toward ball while turning slightly to center it.
  float omega = clampFloat(rel * KP_BALL_TURN, -0.42f, 0.42f);
  vectorDriveAngle(ballAngleDeg, speed, omega);
}

void behaviorCaptureBall() {
  dribblerSet(true);

  if (hasBallPocket) {
    state = StrategyState::ATTACK_GOAL;
    return;
  }

  if (!ballVisible) {
    state = StrategyState::SEARCH_BALL;
    return;
  }

  float rel = wrap180(ballAngleDeg);

  // If ball slipped far sideways, chase again.
  if (fabsf(rel) > 55.0f) {
    state = StrategyState::CHASE_BALL;
    return;
  }

  // Go mostly straight into dribbler mouth, slight sideways correction.
  float side = clampFloat(rel / 45.0f, -0.45f, 0.45f);
  float omega = clampFloat(rel * 0.012f, -0.35f, 0.35f);
  xDrive(side, CAPTURE_SPEED, omega);
}

void behaviorAttackGoal() {
  dribblerSet(true);

  if (!hasBallPocket && ballVisible) {
    state = StrategyState::CAPTURE_BALL;
    return;
  }

  int goalAngle = 0;
  uint8_t goalDist = 0;
  bool goalFound = getBestGoal(goalAngle, goalDist);

  if (goalFound) {
    float err = wrap180((float)goalAngle);
    float omega = clampFloat(err * KP_GOAL, -0.65f, 0.65f);

    // If aligned and close enough, kick.
    if (fabsf(err) < 8.0f && goalDist > 70 && hasBallPocket) {
      stopDrive();
      kick();
      state = StrategyState::RECOVER;
      recoverStart = millis();
      return;
    }

    // Drive forward while aiming.
    float forward = (fabsf(err) < 28.0f) ? ATTACK_SPEED : 0.32f;
    xDrive(0.0f, forward, omega);
    return;
  }

  // No goal: rotate with ball until goal visible.
  xDrive(0.0f, 0.18f, 0.38f);
}

void behaviorSupportAttacker() {
  dribblerSet(false);

  // If we suddenly get the ball, become primary.
  if (hasBallPocket) {
    attackRole = AttackRole::PRIMARY_ATTACKER;
    state = StrategyState::ATTACK_GOAL;
    return;
  }

  // If we see ball and partner doesn't, become primary.
  if (ballVisible && (!team.valid || !team.partnerBallVisible)) {
    attackRole = AttackRole::PRIMARY_ATTACKER;
    state = StrategyState::CHASE_BALL;
    return;
  }

  // Support attacker: do not collide with primary.
  // Move to side/rebound lane relative to ball.
  if (ballVisible) {
    float rel = wrap180(ballAngleDeg);

    // If ball is in front, take side lane. If side, shadow it.
    float supportAngle = wrap360(ballAngleDeg + ((THIS_ROBOT_ID == 1) ? 65.0f : -65.0f));
    float omega = headingHoldOmega(targetHeadingDeg);
    vectorDriveAngle(supportAngle, SUPPORT_SPEED, omega);

    // If ball becomes very close and partner doesn't have it, attack.
    if (ballCount >= 9 && fabsf(rel) < 45.0f && !team.partnerHasBall) {
      state = StrategyState::CHASE_BALL;
    }
    return;
  }

  // No ball: occupy open lane and scan.
  xDrive((THIS_ROBOT_ID == 1) ? 0.32f : -0.32f, 0.15f, 0.22f);
}

void updateStrategyState() {
  int escapeAngle;
  if (getLineThreat(escapeAngle) && state != StrategyState::AVOID_LINE) {
    enterLineEscape();
    return;
  }

  updateAttackRole();

  if (state == StrategyState::AVOID_LINE || state == StrategyState::RECOVER) {
    return;
  }

  if (hasBallPocket) {
    state = StrategyState::ATTACK_GOAL;
    return;
  }

  if (attackRole == AttackRole::SUPPORT_ATTACKER) {
    state = StrategyState::SUPPORT_ATTACKER;
    return;
  }

  if (ballVisible) {
    if (fabsf(wrap180(ballAngleDeg)) < 25.0f && ballCount >= 4) {
      state = StrategyState::CAPTURE_BALL;
    } else {
      state = StrategyState::CHASE_BALL;
    }
    return;
  }

  state = StrategyState::SEARCH_BALL;
}

void runStrategy() {
  if (systemMode != SystemMode::GAME) {
    emergencyStopAll();
    return;
  }

  uint32_t now = millis();
  if (now - lastStrategyTick < STRATEGY_PERIOD_MS) return;
  lastStrategyTick = now;

  updateStrategyState();

  switch (state) {
    case StrategyState::SEARCH_BALL:
      behaviorSearchBall();
      break;

    case StrategyState::CHASE_BALL:
      behaviorChaseBall();
      break;

    case StrategyState::CAPTURE_BALL:
      behaviorCaptureBall();
      break;

    case StrategyState::ATTACK_GOAL:
      behaviorAttackGoal();
      break;

    case StrategyState::SUPPORT_ATTACKER:
      behaviorSupportAttacker();
      break;

    case StrategyState::AVOID_LINE:
      behaviorAvoidLine();
      break;

    case StrategyState::RECOVER:
      behaviorRecover();
      break;
  }
}

// ============================================================================
// TEST COMMANDS FROM ESP WEB UI
// Minimal parser for protocol test strings.
// ============================================================================

void handleAsciiCommand(const String& line) {
  if (line == "TEST:ON") {
    // TEST is allowed only before match start.
    // Once GPIO9 says START, the web app must not enter manual test.
    if (!g_rcjRun && (systemMode == SystemMode::READY || systemMode == SystemMode::PAUSED)) {
      emergencyStopAll();
      systemMode = SystemMode::TEST;
    }
    return;
  }

  if (line == "TEST:OFF") {
    emergencyStopAll();

    // If the game already started while we were exiting test,
    // go straight to GAME. Otherwise go back to READY.
    if (g_rcjRun) {
      systemMode = SystemMode::GAME;
      targetHeadingDeg = headingDeg;
      state = StrategyState::SEARCH_BALL;
    } else {
      systemMode = SystemMode::READY;
    }

    return;
  }

  if (line == "ESTOP") {
    emergencyStopAll();

    // ESTOP stops physical outputs, but does not override the RCJ start signal.
    // If GPIO9 is START, stay in GAME logic after the stop pulse.
    if (!g_rcjRun && systemMode != SystemMode::TEST) {
      systemMode = SystemMode::READY;
    }

    return;
  }

  // From here down: all manual physical commands are allowed ONLY in TEST.
  // This blocks app control immediately when the match starts.
  if (systemMode != SystemMode::TEST) return;
  if (g_rcjRun) return;

  if (line.startsWith("OMNI:")) {
    int a, b, c;
    if (sscanf(line.c_str(), "OMNI:%d:%d:%d", &a, &b, &c) == 3) {
      xDrive(a / 100.0f, b / 100.0f, c / 100.0f);
    }
    return;
  }

  if (line.startsWith("MOTOR:")) {
    int n, dir, pwm;
    if (sscanf(line.c_str(), "MOTOR:%d:%d:%d", &n, &dir, &pwm) == 3) {
      pwm = constrain(pwm, 0, 100);
      int speed = map(pwm, 0, 100, 0, 255);
      if (dir == 0) speed = -speed;

      switch (n) {
        case 1: setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, speed); break;
        case 2: setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, speed); break;
        case 3: setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, speed); break;
        case 4: setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, speed); break;
        default: break;
      }
    }
    return;
  }

  if (line.startsWith("DRIBBLER:")) {
    int pct = 0;
    if (sscanf(line.c_str(), "DRIBBLER:%d", &pct) == 1) {
      pct = constrain(pct, 0, 100);
      analogWrite(DRIBBLER_PIN, constrain(map(pct, 0, 100, 0, 255), 0, 255));
    }
    return;
  }

  if (line.startsWith("KICK:")) {
    int pct = 0;
    if (sscanf(line.c_str(), "KICK:%d", &pct) == 1) {
      // In TEST we do a controlled kicker pulse directly,
      // because kick() intentionally allows kicking only in GAME.
      if (!kickerActive && millis() - lastKick >= KICKER_COOLDOWN_MS) {
        digitalWrite(KICKER_PIN, HIGH);
        kickerActive = true;
        kickerStart = millis();
      }
    }
    return;
  }

  if (line == "IR:RAW") {
    CAM_FRONT.print("IR:");
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
      CAM_FRONT.print(irSensors[i] ? 1 : 0);
      CAM_FRONT.print(",");
    }
    CAM_FRONT.println(hasBallPocket ? 1 : 0);
    return;
  }

  if (line == "COMPASS:READ") {
    CAM_FRONT.print("CMP:");
    CAM_FRONT.print((int)headingDeg);
    CAM_FRONT.print(",");
    CAM_FRONT.println(gyroOK ? 1 : 0);
    return;
  }

  if (line == "QUERY:STATUS") {
    CAM_FRONT.print("STA:");
    CAM_FRONT.print("100,");
    CAM_FRONT.print(ESP_ID_FRONT);
    CAM_FRONT.print(",");
    CAM_FRONT.print(ESP_ID_RIGHT);
    CAM_FRONT.print(",");
    CAM_FRONT.print(ESP_ID_REAR);
    CAM_FRONT.print(",");
    CAM_FRONT.print(ESP_ID_LEFT);
    CAM_FRONT.print(",");
    CAM_FRONT.print(team.valid ? team.partnerId : 0);
    CAM_FRONT.print(",--,");
    CAM_FRONT.println(millis() / 1000);
    return;
  }
}

void readAsciiFromFrontEsp() {
  static char buf[96];
  static uint8_t idx = 0;

  while (CAM_FRONT.available()) {
    char c = (char)CAM_FRONT.peek();

    // Binary packets/events are handled elsewhere.
    if ((uint8_t)c < 0x20 && c != '\n' && c != '\r') return;
    if ((uint8_t)c >= 0x80) return;

    CAM_FRONT.read();

    if (c == '\n') {
      buf[idx] = '\0';
      String line(buf);
      line.trim();
      idx = 0;
      if (line.length()) handleAsciiCommand(line);
    } else if (c != '\r' && idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}

// ============================================================================
// TELEMETRY TO TEST UI
// ============================================================================

void sendTelemetry() {
  static uint32_t last = 0;
  if (millis() - last < 120) return;
  last = millis();

  int goalAngle = -1;
  uint8_t goalDist = 0;
  bool goal = getBestGoal(goalAngle, goalDist);

  int lineEsc = -1;
  bool line = getLineThreat(lineEsc);

  CAM_FRONT.print("TLM:");
  CAM_FRONT.print((int)systemMode);
  CAM_FRONT.print(",100,");
  CAM_FRONT.print(ballVisible ? 1 : 0);
  CAM_FRONT.print(",");
  CAM_FRONT.print(ballVisible ? (int)ballAngleDeg : -1);
  CAM_FRONT.print(",");
  CAM_FRONT.print(constrain(ballCount * 20, 0, 255));
  CAM_FRONT.print(",");
  CAM_FRONT.print(hasBallPocket ? 1 : 0);
  CAM_FRONT.print(",");
  CAM_FRONT.print(line ? 1 : 0);
  CAM_FRONT.print(",");
  CAM_FRONT.print((int)headingDeg);
  CAM_FRONT.print(",");
  CAM_FRONT.println(goal ? goalAngle : -1);
}

// ============================================================================
// SETUP
// ============================================================================

void setupPins() {
  pinMode(ENG1_DR1, OUTPUT);
  pinMode(ENG1_DR2, OUTPUT);
  pinMode(ENG1_SP, OUTPUT);

  pinMode(ENG2_DR1, OUTPUT);
  pinMode(ENG2_DR2, OUTPUT);
  pinMode(ENG2_SP, OUTPUT);

  pinMode(ENG3_DR1, OUTPUT);
  pinMode(ENG3_DR2, OUTPUT);
  pinMode(ENG3_SP, OUTPUT);

  pinMode(ENG4_DR1, OUTPUT);
  pinMode(ENG4_DR2, OUTPUT);
  pinMode(ENG4_SP, OUTPUT);

  analogWriteFrequency(ENG1_SP, 20000);
  analogWriteFrequency(ENG2_SP, 20000);
  analogWriteFrequency(ENG3_SP, 20000);
  analogWriteFrequency(ENG4_SP, 20000);

  pinMode(DRIBBLER_PIN, OUTPUT);
  pinMode(KICKER_PIN, OUTPUT);
  analogWrite(DRIBBLER_PIN, 0);
  digitalWrite(KICKER_PIN, LOW);

#if USE_POCKET_SENSOR
  pinMode(POCKET_PIN, INPUT_PULLUP);
#endif

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(DIRECT_PINS[i], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(DIRECT_PINS[0]), isrDirect0, FALLING);
  attachInterrupt(digitalPinToInterrupt(DIRECT_PINS[1]), isrDirect1, FALLING);
  attachInterrupt(digitalPinToInterrupt(DIRECT_PINS[2]), isrDirect2, FALLING);
  attachInterrupt(digitalPinToInterrupt(DIRECT_PINS[3]), isrDirect3, FALLING);

  pinMode(PIN_RCJ_SIGNAL, INPUT_PULLDOWN);
  g_rcjRun = digitalReadFast(PIN_RCJ_SIGNAL);
  attachInterrupt(digitalPinToInterrupt(PIN_RCJ_SIGNAL), isrRcjSignalChange, CHANGE);
}

void setupGyro() {
  Wire1.begin();

  if (bno.begin()) {
    gyroOK = true;
    bno.setExtCrystalUse(true);
    delay(50);
    updateGyro();
    targetHeadingDeg = headingDeg;
  } else {
    gyroOK = false;
  }
}

void setupCameras() {
  CAM_FRONT.begin(UART_BAUD);
  CAM_RIGHT.begin(UART_BAUD);
  CAM_REAR.begin(UART_BAUD);
  CAM_LEFT.begin(UART_BAUD);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  setupPins();
  setupCameras();
  setupGyro();

  emergencyStopAll();

  systemMode = g_rcjRun ? SystemMode::GAME : SystemMode::READY;
  state = StrategyState::SEARCH_BALL;

  Serial.println("[RoboCap] Teensy main loaded");
  Serial.println("[RoboCap] Strategy: TWO ATTACKERS, no goalie");
  Serial.print("[RoboCap] Gyro: ");
  Serial.println(gyroOK ? "OK" : "FAILED");

  uint8_t initialEspState =
  (systemMode == SystemMode::GAME) ? ROBOT_STATE_GAME :
  (systemMode == SystemMode::TEST) ? ROBOT_STATE_TEST :
  ROBOT_STATE_READY;

CAM_FRONT.write((uint8_t)CMD_ROBOT_STATE);
CAM_FRONT.write(initialEspState);
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  updateSystemMode();

  updateGyro();
  updateIR();

  // Important: camera reading and team comms both use CAM_FRONT in your design.
  // If this causes parser conflict, split front camera and forward ESP bridge to
  // different UARTs, or add a stricter binary packet parser.
  updateTeamComms();
  readAsciiFromFrontEsp();
  updateCameras();

  kickerUpdate();

  // Inform forward ESP of state for test UI gating.
  static SystemMode lastSentMode = SystemMode::FAULT;
  if (lastSentMode != systemMode) {
    lastSentMode = systemMode;

    uint8_t espState =
      (systemMode == SystemMode::GAME) ? ROBOT_STATE_GAME :
      (systemMode == SystemMode::TEST) ? ROBOT_STATE_TEST :
      ROBOT_STATE_READY;

    CAM_FRONT.write((uint8_t)CMD_ROBOT_STATE);
    CAM_FRONT.write(espState);
  }

  if (systemMode == SystemMode::GAME) {
    runStrategy();
  } else {
    emergencyStopAll();
  }

  sendTelemetry();
}