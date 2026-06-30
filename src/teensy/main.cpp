// ============================================================================
//  RoboCap 2026 - Teensy 4.1 MAIN firmware
//  Full robot system FSM + TEST + ESP-NOW relay support, but CAMERA-ONLY ball.
//
//  Uses now:
//    - 2/4 ESP32-S3 cameras: ball, yellow/blue goal, white/black lines
//    - 4x TCS34725 colour sensors through TCA9548A
//    - 4 omni motors + dribbler
//    - RCJ Run/Stop pin
//
//  Disabled by request:
//    - NO gyro / BNO055
//    - NO IR ring / IR pocket
//    - NO kicker in strategy. KICK command returns disabled ACK.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "robot_protocol.h"

#ifndef OBJ_BALL
#define OBJ_BALL 0x05
#endif

// ============================================================================
// 0. SYSTEM MACHINE - preserved from the real robot architecture
// ============================================================================
enum SysState : uint8_t {
  S_POWER_ON,
  S_POST,
  S_READY,
  S_TEST,
  S_GAME,
  S_PAUSED,
  S_FAULT
};

volatile SysState sysState = S_POWER_ON;
SysState gamePrevState = S_GAME;

// ============================================================================
// 1. PIN MAP
// ============================================================================
#define ENG1_SP  23
#define ENG1_DR1 13
#define ENG1_DR2 41
#define ENG2_SP  22
#define ENG2_DR1 40
#define ENG2_DR2 39
#define ENG3_SP  37
#define ENG3_DR1 38
#define ENG3_DR2 35
#define ENG4_SP  36
#define ENG4_DR1 34
#define ENG4_DR2 33

#define PIN_DRIBBLER 11
#define PIN_RCJ_RUN  9
#define PIN_SW2      12

#define TCA_ADDR 0x70
#define TCA_A0   4
#define TCA_A1   5
#define TCA_A2   6
#define TCA_RST  10
#define NUM_COLOUR 4
const uint8_t COLOUR_CH[NUM_COLOUR] = {0, 1, 2, 3};

HardwareSerial *const ESP_PORT[4] = {&Serial1, &Serial2, &Serial3, &Serial4};
const int MOUNT[5] = {0, 0, 90, 180, 270};

// Assumption: 0=front-left, 1=front-right, 2=rear-right, 3=rear-left.
const float COLOUR_BEARING[NUM_COLOUR] = {315, 45, 135, 225};

// ============================================================================
// 2. TUNABLES
// ============================================================================
static constexpr bool AUTO_GAME_ON_BOOT = true;      // bench only. false for real RCJ start.
static constexpr bool AUTO_GAME_IGNORE_POST_FAIL = true;
static constexpr bool REQUIRE_SW2_ENABLE = false;
static constexpr int  SW2_ENABLE_LEVEL = HIGH;

static constexpr bool TEST_ALLOW_FROM_FAULT = true;
static constexpr bool TEST_IGNORE_RCJ_GO = true;
static constexpr bool TEST_DEADMAN_ENABLE = true;
static constexpr uint32_t TEST_DEADMAN_MS = 450;

const uint32_t MOTOR_PWM_HZ = 1000;
const uint32_t DRIBBLER_PWM_HZ = 20000;
const float DRIVE_MAX = 0.88f;

const int MOTOR_INV[5] = {
  0,
  1,    // ENG1 RF
  -1,   // ENG2 RR
  -1,   // ENG3 LR
  1     // ENG4 LF
};

const uint8_t DRIBBLER_RUN_PWM = 225;
const uint8_t DRIBBLER_HOLD_PWM = 205;
const uint8_t DRIBBLER_IDLE_PWM = 135;

// Camera object timeout
static constexpr uint32_t VIS_TIMEOUT_MS = 650;
static constexpr uint32_t STRATEGY_PERIOD_MS = 20;
static constexpr uint32_t ROBOTMSG_TX_MS = 50;
static constexpr uint32_t PARTNER_TIMEOUT_MS = 650;

// Ball from camera distance proxy 0..255
static constexpr uint8_t BALL_APPROACH_DIST = 45;
static constexpr uint8_t BALL_CLOSE_DIST = 125;
static constexpr uint8_t BALL_HELD_DIST = 155;
static constexpr int BALL_CENTER_DEG = 12;
static constexpr int BALL_NEAR_CENTER_DEG = 24;

// Goal
static constexpr bool TARGET_GOAL_BLUE = true;
bool opponentGoalIsYellow = !TARGET_GOAL_BLUE;
uint8_t lockedGoalColour = 0xFF; // 0 yellow, 1 blue, 0xFF default from TARGET_GOAL_BLUE
static constexpr uint8_t GOAL_MIN_DIST = 35;
static constexpr float GOAL_ALIGN_DEG = 15.0f;

// Line
const uint16_t LINE_CLEAR_MIN = 1500;
const uint16_t LINE_RGB_SPREAD_MAX = 420;
static constexpr uint8_t CAMERA_LINE_CLOSE_DIST = 145;
static constexpr float LINE_ESCAPE_SPEED = 0.80f;
static constexpr uint32_t LINE_ESCAPE_DRIVE_MS = 360;
static constexpr uint32_t LINE_ESCAPE_ROT_MS = 600;
static constexpr uint32_t LINE_COOLDOWN_MS = 220;

// Behaviour tuning
static constexpr float SEARCH_ROT = 0.42f;
static constexpr float SEARCH_DRIFT = 0.08f;
static constexpr float CHASE_SPEED_FAST = 0.62f;
static constexpr float CHASE_SPEED_SLOW = 0.32f;
static constexpr float CAPTURE_SPEED = 0.44f;
static constexpr float SUPPORT_SPEED = 0.42f;
static constexpr float KP_BALL_TURN = 0.018f;
static constexpr float KP_GOAL_TURN = 0.016f;
static constexpr uint32_t CAPTURE_SETTLE_MS = 170;
static constexpr uint32_t RECOVER_MS = 260;

// Push-shot / spin-pushback without kicker
static constexpr bool USE_SPIN_PUSHBACK = true;
static constexpr float PUSH_ALIGN_DEG = 14.0f;
static constexpr float PUSH_DRIVE_SPEED = 0.90f;
static constexpr uint32_t PUSH_HOLD_MS = 150;
static constexpr uint32_t PUSH_RELEASE_MS = 260;
static constexpr uint32_t PUSH_RECOVER_MS = 170;
static constexpr float SPIN_OMEGA_FAST = 0.72f;
static constexpr float SPIN_FORWARD = 0.16f;
static constexpr float SPIN_SIDE = 0.22f;
static constexpr float SPIN_RELEASE_SPEED = 0.95f;
static constexpr float SPIN_GOAL_WINDOW_DEG = 22.0f;
static constexpr uint32_t SPIN_MIN_HOLD_MS = 180;
static constexpr uint32_t SPIN_MAX_HOLD_MS = 850;
static constexpr uint32_t SPIN_RELEASE_MS = 230;

// ============================================================================
// 3. GLOBAL STATE
// ============================================================================
volatile bool runSignal = false;
volatile bool runEdge = false;
bool testEnteredFromFault = false;
uint32_t lastTestPhysicalCmdMs = 0;
uint32_t lastTlm = 0;
uint32_t lastStrategyTick = 0;
uint32_t lastCamDebug = 0;
uint32_t lineCooldownUntil = 0;

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
bool colourGood[NUM_COLOUR] = {false};
bool lineDetected = false;
uint8_t lineMask = 0;

int forwardPortIdx = -1;
uint16_t camPkt[5] = {0};
uint16_t rawByte[4] = {0};

struct CamVis {
  int16_t goalR = -1, goalC = -1;
  uint8_t goalCol = 0;       // 0 none, 1 yellow, 2 blue
  uint8_t goalDist = 0;
  int16_t white = -1;
  uint8_t whiteDist = 0;
  int16_t black = -1;
  uint8_t blackDist = 0;
  int16_t ballR = -1, ballC = -1;
  uint8_t ballDist = 0;
  uint32_t tG = 0, tW = 0, tK = 0, tB = 0;
};
CamVis camVis[4];

struct LastObj {
  int8_t type = -1;
  int16_t angleRobot = 0;
  uint8_t dist = 0;
  uint32_t t = 0;
};
LastObj lastGoal, lastBall;

bool ballVisible = false;
float ballAngle = -1;
int ballStrength = 0;
bool ballInPocket = false;   // camera-close proxy, not IR
uint32_t lastBallHeldMs = 0;
int goalBearing = -1;

// ---- Partner / ESP-NOW relay state ----
RobotRole myRole = ROLE_ATTACKER;
bool partnerOnline = false;
bool iAmHighMac = false;
uint32_t lastPartnerMsgMs = 0;
int partnerScore = 0;

#ifndef THIS_ROBOT_ID
#define THIS_ROBOT_ID 1
#endif

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
TeamState team;
uint32_t msgSeq = 0;

// ---- Strategy FSM ----
enum class StrategyState : uint8_t {
  SEARCH_BALL,
  CHASE_BALL,
  CAPTURE_BALL,
  ATTACK_GOAL,
  SPIN_PUSHBACK,
  PUSH_SHOT,
  SUPPORT_ATTACKER,
  AVOID_LINE,
  RECOVER
};

enum class AttackRole : uint8_t {
  PRIMARY_ATTACKER,
  SUPPORT_ATTACKER,
  SOLO_ATTACKER
};

StrategyState strategyState = StrategyState::SEARCH_BALL;
AttackRole attackRole = AttackRole::SOLO_ATTACKER;
uint32_t lineEscapeStart = 0;
uint32_t recoverStart = 0;
uint32_t pushShotStart = 0;
bool pushShotReleasing = false;
uint32_t spinStart = 0;
bool spinReleasing = false;
int spinDir = 1;

struct PortParser {
  uint8_t buf[90];
  int len = 0;
  int need = 0;
  uint8_t kind = 0;
  char ascii[128];
  int aidx = 0;
};
PortParser parser[4];

// ============================================================================
// 4. MATH / UTIL
// ============================================================================
static inline float wrap360f(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;
  return a;
}

static inline float wrap180f(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static inline int wrap180i(int a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

static inline float clampF(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static const char *sysName(SysState s) {
  switch (s) {
    case S_POWER_ON: return "POWER_ON";
    case S_POST: return "POST";
    case S_READY: return "READY";
    case S_TEST: return "TEST";
    case S_GAME: return "GAME";
    case S_PAUSED: return "PAUSED";
    case S_FAULT: return "FAULT";
    default: return "?";
  }
}

static const char *strategyName(StrategyState s) {
  switch (s) {
    case StrategyState::SEARCH_BALL: return "SEARCH_BALL";
    case StrategyState::CHASE_BALL: return "CHASE_BALL";
    case StrategyState::CAPTURE_BALL: return "CAPTURE_BALL";
    case StrategyState::ATTACK_GOAL: return "ATTACK_GOAL";
    case StrategyState::SPIN_PUSHBACK: return "SPIN_PUSHBACK";
    case StrategyState::PUSH_SHOT: return "PUSH_SHOT";
    case StrategyState::SUPPORT_ATTACKER: return "SUPPORT_ATTACKER";
    case StrategyState::AVOID_LINE: return "AVOID_LINE";
    case StrategyState::RECOVER: return "RECOVER";
    default: return "?";
  }
}

// ============================================================================
// 5. MOTORS / DRIBBLER
// ============================================================================
void setMotor(int dir1, int dir2, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void motorKill() {
  setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, 0);
  setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, 0);
  setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, 0);
  setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, 0);
}

void omniDrive(float vx, float vy, float omega) {
  float vRF = vy - vx - omega;
  float vRR = vy + vx - omega;
  float vLR = vy - vx + omega;
  float vLF = vy + vx + omega;
  float m = max(max(fabs(vRF), fabs(vRR)), max(fabs(vLR), fabs(vLF)));
  if (m > 1.0f) {
    vRF /= m; vRR /= m; vLR /= m; vLF /= m;
  }
  setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, (int)(vRF * 255 * DRIVE_MAX * MOTOR_INV[1]));
  setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, (int)(vRR * 255 * DRIVE_MAX * MOTOR_INV[2]));
  setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, (int)(vLR * 255 * DRIVE_MAX * MOTOR_INV[3]));
  setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, (int)(vLF * 255 * DRIVE_MAX * MOTOR_INV[4]));
}

void vectorDriveAngle(float angleDeg, float speed, float omega) {
  float rad = angleDeg * DEG_TO_RAD;
  float vx = sinf(rad) * speed;
  float vy = cosf(rad) * speed;
  omniDrive(vx, vy, omega);
}

void dribblerSet(uint8_t pwm) {
  analogWrite(PIN_DRIBBLER, pwm);
}

void motorsInit() {
  const int sp[4] = {ENG1_SP, ENG2_SP, ENG3_SP, ENG4_SP};
  const int d1[4] = {ENG1_DR1, ENG2_DR1, ENG3_DR1, ENG4_DR1};
  const int d2[4] = {ENG1_DR2, ENG2_DR2, ENG3_DR2, ENG4_DR2};
  for (int i = 0; i < 4; i++) {
    pinMode(sp[i], OUTPUT);
    pinMode(d1[i], OUTPUT);
    pinMode(d2[i], OUTPUT);
    analogWriteFrequency(sp[i], MOTOR_PWM_HZ);
  }
  motorKill();
}

void actuatorsInit() {
  pinMode(PIN_DRIBBLER, OUTPUT);
  analogWriteFrequency(PIN_DRIBBLER, DRIBBLER_PWM_HZ);
  dribblerSet(0);
  pinMode(PIN_SW2, INPUT_PULLUP);
}

void markTestPhysicalCmd() { lastTestPhysicalCmdMs = millis(); }

void testDeadmanUpdate() {
  if (!TEST_DEADMAN_ENABLE || sysState != S_TEST) return;
  if (lastTestPhysicalCmdMs && millis() - lastTestPhysicalCmdMs > TEST_DEADMAN_MS) {
    motorKill();
    dribblerSet(0);
    lastTestPhysicalCmdMs = 0;
  }
}

// ============================================================================
// 6. COLOUR SENSORS / LINE
// ============================================================================
void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}
void tcaDeselect() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}
void tcaReset() {
  digitalWrite(TCA_RST, LOW);
  delayMicroseconds(25);
  digitalWrite(TCA_RST, HIGH);
  delayMicroseconds(60);
  tcaDeselect();
}

void colourInit() {
  pinMode(TCA_A0, OUTPUT); pinMode(TCA_A1, OUTPUT); pinMode(TCA_A2, OUTPUT);
  digitalWrite(TCA_A0, LOW); digitalWrite(TCA_A1, LOW); digitalWrite(TCA_A2, LOW);
  pinMode(TCA_RST, OUTPUT); digitalWrite(TCA_RST, HIGH);
  tcaReset();

  Serial.print("[COLOUR] good channels:");
  bool any = false;
  for (int i = 0; i < NUM_COLOUR; i++) {
    tcaSelect(COLOUR_CH[i]);
    Wire.beginTransmission(0x29);
    colourGood[i] = (Wire.endTransmission() == 0);
    if (colourGood[i]) { tcs.begin(); Serial.printf(" ch%d", COLOUR_CH[i]); any = true; }
    tcaReset();
  }
  Serial.println(any ? "" : " NONE");
}

bool isWhiteLine(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c <= LINE_CLEAR_MIN) return false;
  uint16_t mx = max(r, max(g, b));
  uint16_t mn = min(r, min(g, b));
  if (mx == 0) return false;
  return ((mx - mn) <= LINE_RGB_SPREAD_MAX) || (((mx - mn) * 100 / mx) < 30);
}

void lineUpdate() {
  lineMask = 0;
  for (int i = 0; i < NUM_COLOUR; i++) {
    if (!colourGood[i]) continue;
    tcaSelect(COLOUR_CH[i]);
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    tcaDeselect();
    if (isWhiteLine(r, g, b, c)) lineMask |= (1 << i);
  }
  lineDetected = (lineMask != 0);
}

void sendCOL();

void i2cScan() {
  Serial.println("[I2C] scanning main bus...");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C] 0x%02X ACK\n", a);
      found++;
    }
  }
  if (!found) Serial.println("[I2C] NONE found");
}

// ============================================================================
// 7. RCJ RUN/STOP
// ============================================================================
void rcjISR() {
  runSignal = (digitalRead(PIN_RCJ_RUN) == HIGH);
  runEdge = true;
}

void rcjInit() {
  pinMode(PIN_RCJ_RUN, INPUT_PULLDOWN);
  runSignal = (digitalRead(PIN_RCJ_RUN) == HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_RCJ_RUN), rcjISR, CHANGE);
}

// ============================================================================
// 8. CAMERA / ESP UART PROTOCOL
// ============================================================================
bool isFresh(uint32_t t, uint32_t timeout = VIS_TIMEOUT_MS) {
  return t && (millis() - t < timeout);
}

void updateBallSummary() {
  uint32_t now = millis();
  bool found = false;
  uint8_t bestDist = 0;
  int bestAng = 0;

  // Prefer front camera because it is above dribbler.
  CamVis &front = camVis[0];
  if (front.tB && now - front.tB < VIS_TIMEOUT_MS) {
    found = true;
    bestDist = front.ballDist;
    bestAng = wrap180i(front.ballR);
  } else {
    for (int i = 0; i < 4; i++) {
      CamVis &cv = camVis[i];
      if (!cv.tB || now - cv.tB >= VIS_TIMEOUT_MS) continue;
      if (!found || cv.ballDist > bestDist) {
        found = true;
        bestDist = cv.ballDist;
        bestAng = wrap180i(cv.ballR);
      }
    }
  }

  ballVisible = found;
  ballAngle = found ? bestAng : -1;
  ballStrength = found ? bestDist : 0;

  bool cameraHeld = isFresh(front.tB) && abs(front.ballR) <= BALL_CENTER_DEG && front.ballDist >= BALL_HELD_DIST;
  if (cameraHeld) lastBallHeldMs = now;
  ballInPocket = (now - lastBallHeldMs < 800);
}

bool goalMatchesLockedColour(int8_t type) {
  if (!isGoalType(type)) return false;
  if (lockedGoalColour == 0xFF) {
    return TARGET_GOAL_BLUE ? (type == OBJ_GOAL_BLUE) : (type == OBJ_GOAL_YELLOW);
  }
  if (lockedGoalColour == 0) return type == OBJ_GOAL_YELLOW;
  if (lockedGoalColour == 1) return type == OBJ_GOAL_BLUE;
  return true;
}

bool getBestGoal(int &angleOut, uint8_t &distOut) {
  bool found = false;
  uint8_t bestDist = 0;
  int bestAngle = 0;
  uint32_t now = millis();

  CamVis &front = camVis[0];
  if (front.tG && now - front.tG < VIS_TIMEOUT_MS) {
    int8_t typ = (front.goalCol == 1) ? OBJ_GOAL_YELLOW : (front.goalCol == 2) ? OBJ_GOAL_BLUE : -1;
    if (goalMatchesLockedColour(typ)) {
      angleOut = wrap180i(front.goalR);
      distOut = front.goalDist;
      return true;
    }
  }

  for (int i = 0; i < 4; i++) {
    CamVis &cv = camVis[i];
    if (!cv.tG || now - cv.tG >= VIS_TIMEOUT_MS) continue;
    int8_t typ = (cv.goalCol == 1) ? OBJ_GOAL_YELLOW : (cv.goalCol == 2) ? OBJ_GOAL_BLUE : -1;
    if (!goalMatchesLockedColour(typ)) continue;
    if (!found || cv.goalDist > bestDist) {
      found = true;
      bestDist = cv.goalDist;
      bestAngle = wrap180i(cv.goalR);
    }
  }

  if (!found && millis() - lastGoal.t < 500 && goalMatchesLockedColour(lastGoal.type)) {
    found = true;
    bestAngle = wrap180i(lastGoal.angleRobot);
    bestDist = lastGoal.dist;
  }

  if (!found) return false;
  angleOut = bestAngle;
  distOut = bestDist;
  return true;
}

bool getLineThreat(int &escapeAngleOut) {
  bool threat = false;
  float sx = 0, sy = 0;
  uint32_t now = millis();

  if (lineMask && millis() > lineCooldownUntil) {
    threat = true;
    for (int i = 0; i < NUM_COLOUR; i++) {
      if (!(lineMask & (1 << i))) continue;
      float escape = wrap360f(COLOUR_BEARING[i] + 180.0f);
      sx += sinf(escape * DEG_TO_RAD);
      sy += cosf(escape * DEG_TO_RAD);
    }
  }

  for (int i = 0; i < 4; i++) {
    CamVis &cv = camVis[i];
    if (!cv.tW || now - cv.tW >= VIS_TIMEOUT_MS || cv.whiteDist < CAMERA_LINE_CLOSE_DIST) continue;
    threat = true;
    float escape = wrap360f(cv.white + 180.0f);
    sx += sinf(escape * DEG_TO_RAD);
    sy += cosf(escape * DEG_TO_RAD);
  }

  if (!threat) return false;
  escapeAngleOut = (int)wrap360f(atan2f(sx, sy) * RAD_TO_DEG);
  return true;
}

void handleDetection(const uint8_t *p, int n) {
  uint8_t espId = p[0];
  if (espId >= 1 && espId <= 4) camPkt[espId]++;
  if (espId == ESP_ID_FRONT && forwardPortIdx < 0) forwardPortIdx = 0;
  if (n == 2 && p[1] == PACKET_NO_DETECT) return;
  if (n < 4 || espId < 1 || espId > 4) return;

  int8_t type = (int8_t)p[1];
  int8_t camAng = (int8_t)p[2];
  uint8_t dist = p[3];
  int robotAng = camAng + MOUNT[espId];
  robotAng = wrap180i(robotAng);
  CamVis &cv = camVis[espId - 1];
  uint32_t now = millis();

  if (type == OBJ_BALL) {
    cv.ballR = robotAng;
    cv.ballC = camAng;
    cv.ballDist = dist;
    cv.tB = now;
    lastBall = {type, (int16_t)robotAng, dist, now};
  } else if (isGoalType(type)) {
    cv.goalR = robotAng;
    cv.goalC = camAng;
    cv.goalCol = (type == OBJ_GOAL_YELLOW) ? 1 : (type == OBJ_GOAL_BLUE) ? 2 : 0;
    cv.goalDist = dist;
    cv.tG = now;
    lastGoal = {type, (int16_t)robotAng, dist, now};
    goalBearing = robotAng;
  } else if (type == OBJ_LINE_WHITE) {
    cv.white = robotAng;
    cv.whiteDist = dist;
    cv.tW = now;
  } else if (type == OBJ_LINE_BLACK) {
    cv.black = robotAng;
    cv.blackDist = dist;
    cv.tK = now;
  }

  updateBallSummary();
}

void handleWifiData(const uint8_t *data, uint8_t len);
void handleAsciiFromEsp(int portIdx, const char *line);

void parseByte(int idx, uint8_t b) {
  PortParser &P = parser[idx];

  if (P.need > 0) {
    P.buf[P.len++] = b;
    if (P.kind == 'D' && P.len == 2) {
      if (P.buf[1] == PACKET_NO_DETECT) {
        handleDetection(P.buf, P.len);
        P.need = 0; P.len = 0;
        return;
      }
      P.need = 4;
    }
    if (P.kind == 'E' && P.buf[0] == EVT_WIFI_DATA && P.len == 2) {
      uint8_t payloadLen = P.buf[1];
      if (payloadLen > sizeof(P.buf) - 2) { P.need = 0; P.len = 0; return; }
      P.need = 2 + payloadLen;
    }
    if (P.len >= P.need) {
      if (P.kind == 'D') handleDetection(P.buf, P.len);
      else if (P.kind == 'E') {
        lastPartnerMsgMs = millis();
        if (P.buf[0] == EVT_PARTNER_FOUND) {
          partnerOnline = (P.buf[1] != 0);
          iAmHighMac = (P.buf[1] == 1);
        } else if (P.buf[0] == EVT_WIFI_DATA) {
          handleWifiData(P.buf + 2, P.buf[1]);
        }
      }
      P.need = 0; P.len = 0;
    }
    return;
  }

  if (b == EVT_PARTNER_FOUND) {
    P.buf[0] = b; P.len = 1; P.kind = 'E'; P.need = 2; return;
  }
  if (b == EVT_WIFI_DATA) {
    P.buf[0] = b; P.len = 1; P.kind = 'E'; P.need = 2; return;
  }
  if (b >= ESP_ID_FRONT && b <= ESP_ID_LEFT) {
    if (b == ESP_ID_FRONT && forwardPortIdx != idx) {
      forwardPortIdx = idx;
      Serial.printf("[FWD] learned forwardPortIdx=%d Serial%d\n", idx, idx + 1);
    }
    P.buf[0] = b; P.len = 1; P.kind = 'D'; P.need = 2; return;
  }
  if (b == '\n' || b == '\r') {
    if (P.aidx > 0) {
      P.ascii[P.aidx] = 0;
      if (forwardPortIdx != idx) {
        forwardPortIdx = idx;
        Serial.printf("[FWD] learned forwardPortIdx=%d Serial%d via ASCII\n", idx, idx + 1);
      }
      handleAsciiFromEsp(idx, P.ascii);
      P.aidx = 0;
    }
    return;
  }
  if (b >= 0x20 && b <= 0x7E) {
    if (P.aidx < (int)sizeof(P.ascii) - 1) P.ascii[P.aidx++] = (char)b;
  }
}

void espRxPump() {
  for (int i = 0; i < 4; i++) {
    while (ESP_PORT[i]->available()) {
      uint8_t b = ESP_PORT[i]->read();
      rawByte[i]++;
      parseByte(i, b);
    }
  }
  updateBallSummary();
}

void espSendByte(uint8_t b) {
  if (forwardPortIdx >= 0) ESP_PORT[forwardPortIdx]->write(b);
}
void espSendAscii(const char *s) {
  if (forwardPortIdx >= 0) {
    ESP_PORT[forwardPortIdx]->print(s);
    ESP_PORT[forwardPortIdx]->print('\n');
  }
  Serial.println(s);
}
void espPushRobotState(SysState s) {
  uint8_t v = (s == S_GAME) ? ROBOT_STATE_GAME : (s == S_TEST) ? ROBOT_STATE_TEST : ROBOT_STATE_READY;
  espSendByte(CMD_ROBOT_STATE);
  espSendByte(v);
}

// ============================================================================
// 9. TELEMETRY
// ============================================================================
void sendTLM() {
  int goalAng = -1;
  uint8_t goalDist = 0;
  getBestGoal(goalAng, goalDist);
  char b[180];
  snprintf(b, sizeof(b), "TLM:%d,%d,%d,%d,%d,%d,%d,%d,%d",
           (sysState == S_GAME) ? 1 : (sysState == S_TEST) ? 2 : 0,
           0,
           ballVisible ? 1 : 0,
           ballVisible ? (int)ballAngle : -1,
           ballStrength,
           ballInPocket ? 1 : 0,
           lineDetected ? 1 : 0,
           0,
           goalAng);
  espSendAscii(b);
}

void sendVIS() {
  uint32_t now = millis();
  for (int c = 0; c < 4; c++) {
    CamVis &cv = camVis[c];
    bool gOK = cv.tG && now - cv.tG < VIS_TIMEOUT_MS;
    bool wOK = cv.tW && now - cv.tW < VIS_TIMEOUT_MS;
    bool kOK = cv.tK && now - cv.tK < VIS_TIMEOUT_MS;
    bool bOK = cv.tB && now - cv.tB < VIS_TIMEOUT_MS;
    char b[96];
    snprintf(b, sizeof(b), "VIS:%d,%d,%d,%d,%d,%d,%d,%d",
             c,
             gOK ? cv.goalCol : 0,
             gOK ? cv.goalR : -1,
             gOK ? cv.goalC : -1,
             wOK ? cv.white : -1,
             kOK ? cv.black : -1,
             bOK ? cv.ballR : -1,
             bOK ? cv.ballDist : 0);
    espSendAscii(b);
  }
}

void sendCOL() {
  for (int i = 0; i < NUM_COLOUR; i++) {
    char b[110];
    if (!colourGood[i]) {
      snprintf(b, sizeof(b), "COL:%d,ABSENT", i);
      espSendAscii(b);
      continue;
    }
    tcaSelect(COLOUR_CH[i]);
    uint16_t r, g, bl, c;
    tcs.getRawData(&r, &g, &bl, &c);
    tcaDeselect();
    snprintf(b, sizeof(b), "COL:%d,%u,%u,%u,%u", i, r, g, bl, c);
    espSendAscii(b);
  }
}

void sendSTA() {
  char b[160];
  snprintf(b, sizeof(b), "STA:0,%u,%u,%u,%u,%d,partner=%d,uptime=%lu,state=%s,strat=%s",
           camPkt[1], camPkt[2], camPkt[3], camPkt[4], partnerOnline ? 1 : 0,
           team.valid ? team.partnerId : 0, millis() / 1000, sysName(sysState), strategyName(strategyState));
  espSendAscii(b);
}

// ============================================================================
// 10. TEST COMMANDS FROM WEB/ESP
// ============================================================================
void handleAsciiFromEsp(int portIdx, const char *line) {
  Serial.printf("[RX-ESP p%d] \"%s\" state=%s\n", portIdx, line, sysName(sysState));

  if (!strcmp(line, "TEST:ON")) {
    bool allowFaultTest = TEST_ALLOW_FROM_FAULT && (sysState == S_FAULT);
    if (sysState == S_READY || sysState == S_PAUSED || allowFaultTest || AUTO_GAME_ON_BOOT) {
      motorKill(); dribblerSet(0);
      testEnteredFromFault = allowFaultTest;
      lastTestPhysicalCmdMs = 0;
      sysState = S_TEST;
      espPushRobotState(sysState);
      espSendAscii(allowFaultTest ? "ACK:TEST_ON_FAULT_BYPASS" : "ACK:TEST_ON");
    } else espSendAscii("ERR:NOT_READY");
    return;
  }

  if (!strcmp(line, "TEST:OFF")) {
    motorKill(); dribblerSet(0);
    lastTestPhysicalCmdMs = 0;
    sysState = testEnteredFromFault ? S_FAULT : S_READY;
    testEnteredFromFault = false;
    espPushRobotState(sysState);
    espSendAscii((sysState == S_FAULT) ? "ACK:TEST_OFF_FAULT" : "ACK:TEST_OFF");
    return;
  }

  if (!strcmp(line, "GAME:ON")) {
    if (sysState == S_READY || sysState == S_TEST || sysState == S_PAUSED || AUTO_GAME_ON_BOOT) {
      motorKill(); dribblerSet(0);
      testEnteredFromFault = false;
      sysState = S_GAME;
      espPushRobotState(sysState);
      espSendAscii("ACK:GAME_ON");
    } else espSendAscii("ERR:GAME_NOT_READY");
    return;
  }

  if (!strcmp(line, "GAME:OFF")) {
    motorKill(); dribblerSet(0);
    sysState = S_READY;
    espPushRobotState(sysState);
    espSendAscii("ACK:GAME_OFF");
    return;
  }

  if (!strcmp(line, "ESTOP")) { motorKill(); dribblerSet(0); espSendAscii("ACK:ESTOP"); return; }
  if (!strcmp(line, "QUERY:STATUS")) { sendTLM(); sendSTA(); return; }
  if (!strcmp(line, "VISION:READ")) { sendVIS(); return; }
  if (!strcmp(line, "COLOUR:RAW") || !strcmp(line, "COLOR:RAW")) { sendCOL(); return; }
  if (!strcmp(line, "IR:RAW")) { espSendAscii("IR:DISABLED_CAMERA_ONLY"); return; }
  if (!strcmp(line, "COMPASS:READ")) { espSendAscii("CMP:0,0"); return; }
  if (!strncmp(line, "CALCAM:", 7)) {
    int cam = atoi(line + 7);
    const char *p = strchr(line + 7, ':');
    if (cam >= 1 && cam <= 4 && p) {
      ESP_PORT[cam - 1]->print("CAL:");
      ESP_PORT[cam - 1]->println(p + 1);
      espSendAscii("ACK:CALCAM");
    } else espSendAscii("ERR:CALCAM");
    return;
  }

  if (sysState != S_TEST) { espSendAscii("ERR:TEST_GATE"); return; }

  int a, b, c;
  if (sscanf(line, "MOTOR:%d:%d:%d", &a, &b, &c) == 3) {
    int spd = map(constrain(c, 0, 100), 0, 100, 0, 255);
    if (b == 0) spd = -spd;
    int sp[5] = {0, ENG1_SP, ENG2_SP, ENG3_SP, ENG4_SP};
    int d1[5] = {0, ENG1_DR1, ENG2_DR1, ENG3_DR1, ENG4_DR1};
    int d2[5] = {0, ENG1_DR2, ENG2_DR2, ENG3_DR2, ENG4_DR2};
    if (a >= 1 && a <= 4) {
      spd *= MOTOR_INV[a];
      setMotor(d1[a], d2[a], sp[a], spd);
      markTestPhysicalCmd();
      espSendAscii("ACK:MOTOR");
    } else espSendAscii("ERR:MOTOR");
    return;
  }

  if (sscanf(line, "OMNI:%d:%d:%d", &a, &b, &c) == 3) {
    omniDrive(constrain(a, -100, 100) / 100.0f,
              constrain(b, -100, 100) / 100.0f,
              constrain(c, -100, 100) / 100.0f);
    markTestPhysicalCmd();
    espSendAscii("ACK:OMNI");
    return;
  }

  if (sscanf(line, "DRIBBLER:%d", &a) == 1) {
    dribblerSet(map(constrain(a, 0, 100), 0, 100, 0, 255));
    markTestPhysicalCmd();
    espSendAscii("ACK:DRIBBLER");
    return;
  }

  if (sscanf(line, "KICK:%d", &a) == 1) { espSendAscii("ACK:KICK_DISABLED"); return; }

  if (!strncmp(line, "GOAL_LOCK:", 10)) {
    lockedGoalColour = (strstr(line, "yellow") ? 0 : 1);
    opponentGoalIsYellow = (lockedGoalColour == 0);
    espSendAscii("ACK:GOAL_LOCK_CAMERA_ONLY");
    return;
  }

  espSendAscii("WARN:UNKNOWN_CMD");
}

// ============================================================================
// 11. TEAM COMMS
// ============================================================================
void handleWifiData(const uint8_t *data, uint8_t len) {
  if (len < sizeof(RobotMsg)) return;
  RobotMsg msg;
  memcpy(&msg, data, sizeof(RobotMsg));
  if (msg.magic != COMM_MAGIC || msg.version != COMM_VERSION || msg.team_id != MY_TEAM_ID) return;
  if (msg.robot_id == THIS_ROBOT_ID) return;

  team.valid = true;
  team.partnerId = msg.robot_id;
  team.partnerBallVisible = msg.flags & ENMSG_BALL_VISIBLE;
  team.partnerHasBall = msg.flags & ENMSG_HAS_BALL;
  team.partnerNearLine = msg.flags & ENMSG_NEAR_LINE;
  team.partnerBallAngle = msg.ball_angle / 10;
  team.partnerRole = msg.role;
  team.lastRx = millis();
  partnerOnline = true;
  lastPartnerMsgMs = team.lastRx;
  partnerScore = (int)constrain(msg.ball_radius_px, 0, 1000);
}

void sendRobotMsg() {
  static uint32_t lastTx = 0;
  if (millis() - lastTx < ROBOTMSG_TX_MS) return;
  lastTx = millis();
  if (forwardPortIdx < 0) return;

  RobotMsg msg;
  memset(&msg, 0, sizeof(msg));
  msg.magic = COMM_MAGIC;
  msg.version = COMM_VERSION;
  msg.team_id = MY_TEAM_ID;
  msg.robot_id = THIS_ROBOT_ID;
  msg.seq = msgSeq++;
  msg.tx_millis = millis();
  msg.role = ROLE_ATTACKER;
  msg.battery_pct = 100;

  if (ballVisible) msg.flags |= ENMSG_BALL_VISIBLE;
  if (ballInPocket) msg.flags |= ENMSG_HAS_BALL;
  if (lineDetected) msg.flags |= ENMSG_NEAR_LINE;

  int goalAng = 0; uint8_t goalDist = 0;
  if (getBestGoal(goalAng, goalDist)) {
    if ((lockedGoalColour == 0) || (lockedGoalColour == 0xFF && !TARGET_GOAL_BLUE)) msg.flags |= ENMSG_YGOAL_VISIBLE;
    else msg.flags |= ENMSG_BGOAL_VISIBLE;
    msg.yellow_goal_angle = (lockedGoalColour == 0) ? goalAng : -32768;
    msg.blue_goal_angle = (lockedGoalColour == 1 || lockedGoalColour == 0xFF) ? goalAng : -32768;
  }

  msg.ball_angle = ballVisible ? (int16_t)(wrap180f(ballAngle) * 10.0f) : -32768;
  msg.ball_radius_px = (uint16_t)constrain(ballStrength * 20, 0, 65535);

  ESP_PORT[forwardPortIdx]->write((uint8_t)CMD_RELAY_DATA);
  ESP_PORT[forwardPortIdx]->write((uint8_t)sizeof(RobotMsg));
  ESP_PORT[forwardPortIdx]->write((uint8_t *)&msg, sizeof(RobotMsg));
}

void updateTeamFreshness() {
  if (team.valid && millis() - team.lastRx > PARTNER_TIMEOUT_MS) team.valid = false;
  if (!team.valid) partnerOnline = false;
}

float myBallScore() {
  if (!ballVisible) return 0.0f;
  float rel = fabsf(wrap180f(ballAngle));
  float angleQuality = (rel < 25.0f) ? 1.4f : (rel < 70.0f) ? 1.0f : 0.55f;
  float strength = constrain((float)ballStrength / 180.0f, 0.1f, 1.4f);
  if (ballInPocket) return 999.0f;
  return strength * angleQuality;
}

float partnerBallScore() {
  if (!team.valid || !team.partnerBallVisible) return 0.0f;
  if (team.partnerHasBall) return 998.0f;
  float rel = fabsf((float)team.partnerBallAngle);
  return (rel < 25.0f) ? 1.4f : (rel < 70.0f) ? 1.0f : 0.55f;
}

void updateAttackRole() {
  if (!team.valid) { attackRole = AttackRole::SOLO_ATTACKER; return; }
  if (team.partnerHasBall && !ballInPocket) { attackRole = AttackRole::SUPPORT_ATTACKER; return; }
  if (ballInPocket) { attackRole = AttackRole::PRIMARY_ATTACKER; return; }
  attackRole = (myBallScore() >= partnerBallScore()) ? AttackRole::PRIMARY_ATTACKER : AttackRole::SUPPORT_ATTACKER;
}

// ============================================================================
// 12. BEHAVIOURS
// ============================================================================
void enterLineEscape() {
  strategyState = StrategyState::AVOID_LINE;
  lineEscapeStart = millis();
  dribblerSet(0);
}

void behaviorAvoidLine() {
  int escapeAngle = 180;
  getLineThreat(escapeAngle);
  uint32_t elapsed = millis() - lineEscapeStart;
  if (elapsed < LINE_ESCAPE_DRIVE_MS) { vectorDriveAngle(escapeAngle, LINE_ESCAPE_SPEED, 0); return; }
  if (elapsed < LINE_ESCAPE_ROT_MS) { omniDrive(0, 0, 0.40f); return; }
  lineCooldownUntil = millis() + LINE_COOLDOWN_MS;
  strategyState = StrategyState::RECOVER;
  recoverStart = millis();
}

void behaviorRecover() {
  dribblerSet(0);
  if (millis() - recoverStart < RECOVER_MS) { omniDrive(0, 0, 0); return; }
  strategyState = StrategyState::SEARCH_BALL;
}

void behaviorSearchBall() {
  dribblerSet(0);
  if (team.valid && team.partnerBallVisible && !ballVisible) {
    vectorDriveAngle((float)team.partnerBallAngle, 0.18f, 0.44f);
    return;
  }
  uint32_t phase = (millis() / 1500) % 4;
  float rot = (phase < 2) ? SEARCH_ROT : -SEARCH_ROT;
  float drift = (phase == 1 || phase == 3) ? SEARCH_DRIFT : 0.0f;
  omniDrive(0.0f, drift, rot);
}

void behaviorChaseBall() {
  dribblerSet(DRIBBLER_IDLE_PWM);
  if (!ballVisible) { strategyState = StrategyState::SEARCH_BALL; return; }

  float err = wrap180f(ballAngle);
  float absErr = fabsf(err);
  float omega = clampF(-err * KP_BALL_TURN, -0.62f, 0.62f);

  if (absErr > 95.0f) { omniDrive(0, 0, omega); return; }
  if (absErr > BALL_NEAR_CENTER_DEG) {
    float side = sinf(err * DEG_TO_RAD) * 0.14f;
    omniDrive(side, 0.08f, omega);
    return;
  }
  strategyState = StrategyState::CAPTURE_BALL;
}

void behaviorCaptureBall() {
  dribblerSet(DRIBBLER_RUN_PWM);
  if (ballInPocket) { strategyState = StrategyState::ATTACK_GOAL; return; }
  if (!ballVisible) { strategyState = StrategyState::SEARCH_BALL; return; }

  float err = wrap180f(ballAngle);
  if (fabsf(err) > BALL_NEAR_CENTER_DEG + 5) { strategyState = StrategyState::CHASE_BALL; return; }
  float omega = clampF(-err * KP_BALL_TURN, -0.32f, 0.32f);
  float side = sinf(err * DEG_TO_RAD) * 0.18f;
  float forward = (ballStrength < BALL_APPROACH_DIST) ? CHASE_SPEED_FAST : (ballStrength < BALL_CLOSE_DIST) ? CAPTURE_SPEED : CHASE_SPEED_SLOW;
  omniDrive(side, forward, omega);
}

void startPushShot() {
  pushShotReleasing = false;
  pushShotStart = millis();
  strategyState = StrategyState::PUSH_SHOT;
}

bool shouldPushShot() {
  if (!ballInPocket) return false;
  int a = 0; uint8_t d = 0;
  return getBestGoal(a, d) && d >= GOAL_MIN_DIST;
}

void behaviorPushShot() {
  int goalAngle = 0; uint8_t goalDist = 0;
  if (!getBestGoal(goalAngle, goalDist)) {
    dribblerSet(DRIBBLER_HOLD_PWM);
    omniDrive(0, 0.12f, 0.34f);
    return;
  }
  float err = wrap180f((float)goalAngle);
  if (!pushShotReleasing) {
    dribblerSet(DRIBBLER_HOLD_PWM);
    if (fabsf(err) > PUSH_ALIGN_DEG) {
      float omega = clampF(-err * KP_GOAL_TURN, -0.65f, 0.65f);
      omniDrive(0, 0.12f, omega);
      return;
    }
    pushShotReleasing = true;
    pushShotStart = millis();
  }
  uint32_t elapsed = millis() - pushShotStart;
  if (elapsed < PUSH_HOLD_MS) { dribblerSet(DRIBBLER_HOLD_PWM); vectorDriveAngle(goalAngle, PUSH_DRIVE_SPEED, 0); return; }
  if (elapsed < PUSH_HOLD_MS + PUSH_RELEASE_MS) { dribblerSet(0); vectorDriveAngle(goalAngle, 1.0f, 0); return; }
  if (elapsed < PUSH_HOLD_MS + PUSH_RELEASE_MS + PUSH_RECOVER_MS) { dribblerSet(0); vectorDriveAngle(goalAngle, 0.55f, 0); return; }
  motorKill(); dribblerSet(0); pushShotReleasing = false; strategyState = StrategyState::RECOVER; recoverStart = millis();
}

void startSpinPushback() {
  spinStart = millis();
  spinReleasing = false;
  int goalAngle = 0; uint8_t goalDist = 0;
  if (getBestGoal(goalAngle, goalDist)) spinDir = (wrap180f(goalAngle) >= 0) ? -1 : 1;
  else if (ballVisible) spinDir = (wrap180f(ballAngle) >= 0) ? 1 : -1;
  else spinDir = (THIS_ROBOT_ID == 1) ? 1 : -1;
  strategyState = StrategyState::SPIN_PUSHBACK;
}

void behaviorSpinPushback() {
  int escapeAngle = 0;
  if (getLineThreat(escapeAngle)) { spinReleasing = false; enterLineEscape(); return; }
  if (!ballInPocket && ballVisible) { spinReleasing = false; strategyState = StrategyState::CAPTURE_BALL; return; }

  int goalAngle = 0; uint8_t goalDist = 0;
  bool goalFound = getBestGoal(goalAngle, goalDist);
  uint32_t elapsed = millis() - spinStart;

  if (!spinReleasing) {
    dribblerSet(DRIBBLER_HOLD_PWM);
    bool goodRelease = goalFound && goalDist >= GOAL_MIN_DIST && elapsed >= SPIN_MIN_HOLD_MS && fabsf(wrap180f(goalAngle)) <= SPIN_GOAL_WINDOW_DEG;
    bool timeoutRelease = goalFound && goalDist >= GOAL_MIN_DIST && elapsed >= SPIN_MAX_HOLD_MS;
    if (goodRelease || timeoutRelease) { spinReleasing = true; spinStart = millis(); return; }
    omniDrive(SPIN_SIDE * spinDir, SPIN_FORWARD, SPIN_OMEGA_FAST * spinDir);
    return;
  }

  elapsed = millis() - spinStart;
  if (elapsed < SPIN_RELEASE_MS) {
    dribblerSet(0);
    vectorDriveAngle(goalFound ? goalAngle : 0, SPIN_RELEASE_SPEED, 0.18f * spinDir);
    return;
  }
  motorKill(); dribblerSet(0); spinReleasing = false; strategyState = StrategyState::RECOVER; recoverStart = millis();
}

void behaviorAttackGoal() {
  dribblerSet(DRIBBLER_HOLD_PWM);
  if (!ballInPocket && ballVisible) { strategyState = StrategyState::CAPTURE_BALL; return; }
  if (USE_SPIN_PUSHBACK && ballInPocket) {
    int a = 0; uint8_t d = 0;
    if (getBestGoal(a, d) && d >= GOAL_MIN_DIST) { startSpinPushback(); return; }
  }
  if (shouldPushShot()) { startPushShot(); return; }

  int goalAngle = 0; uint8_t goalDist = 0;
  if (getBestGoal(goalAngle, goalDist)) {
    float err = wrap180f(goalAngle);
    float omega = clampF(-err * KP_GOAL_TURN, -0.62f, 0.62f);
    float forward = (fabsf(err) < 28.0f) ? 0.64f : 0.22f;
    float side = sinf(err * DEG_TO_RAD) * 0.18f;
    omniDrive(side, forward, omega);
    return;
  }
  omniDrive(0.0f, 0.12f, 0.34f);
}

void behaviorSupportAttacker() {
  dribblerSet(0);
  if (ballInPocket) { attackRole = AttackRole::PRIMARY_ATTACKER; strategyState = StrategyState::ATTACK_GOAL; return; }
  if (ballVisible && (!team.valid || !team.partnerBallVisible)) { attackRole = AttackRole::PRIMARY_ATTACKER; strategyState = StrategyState::CHASE_BALL; return; }
  if (ballVisible) {
    float supportAngle = wrap360f(ballAngle + ((THIS_ROBOT_ID == 1) ? 65.0f : -65.0f));
    vectorDriveAngle(supportAngle, SUPPORT_SPEED, 0.20f);
    if (ballStrength >= BALL_CLOSE_DIST && fabsf(wrap180f(ballAngle)) < 45.0f && !team.partnerHasBall) strategyState = StrategyState::CHASE_BALL;
    return;
  }
  omniDrive((THIS_ROBOT_ID == 1) ? 0.30f : -0.30f, 0.12f, 0.22f);
}

void updateStrategyState() {
  int escapeAngle = 0;
  if (getLineThreat(escapeAngle) && strategyState != StrategyState::AVOID_LINE) { enterLineEscape(); return; }

  updateAttackRole();
  if (strategyState == StrategyState::AVOID_LINE || strategyState == StrategyState::RECOVER || strategyState == StrategyState::SPIN_PUSHBACK || strategyState == StrategyState::PUSH_SHOT) return;

  if (USE_SPIN_PUSHBACK && ballInPocket) {
    int a = 0; uint8_t d = 0;
    if (getBestGoal(a, d) && d >= GOAL_MIN_DIST) { startSpinPushback(); return; }
  }
  if (shouldPushShot()) { startPushShot(); return; }
  if (ballInPocket) { strategyState = StrategyState::ATTACK_GOAL; return; }
  if (attackRole == AttackRole::SUPPORT_ATTACKER) { strategyState = StrategyState::SUPPORT_ATTACKER; return; }
  if (ballVisible) {
    strategyState = (fabsf(wrap180f(ballAngle)) < BALL_NEAR_CENTER_DEG && ballStrength >= BALL_APPROACH_DIST) ? StrategyState::CAPTURE_BALL : StrategyState::CHASE_BALL;
    return;
  }
  strategyState = StrategyState::SEARCH_BALL;
}

void runStrategy() {
  uint32_t now = millis();
  if (now - lastStrategyTick < STRATEGY_PERIOD_MS) return;
  lastStrategyTick = now;
  updateBallSummary();
  updateTeamFreshness();
  sendRobotMsg();
  updateStrategyState();

  switch (strategyState) {
    case StrategyState::SEARCH_BALL: behaviorSearchBall(); break;
    case StrategyState::CHASE_BALL: behaviorChaseBall(); break;
    case StrategyState::CAPTURE_BALL: behaviorCaptureBall(); break;
    case StrategyState::ATTACK_GOAL: behaviorAttackGoal(); break;
    case StrategyState::SPIN_PUSHBACK: behaviorSpinPushback(); break;
    case StrategyState::PUSH_SHOT: behaviorPushShot(); break;
    case StrategyState::SUPPORT_ATTACKER: behaviorSupportAttacker(); break;
    case StrategyState::AVOID_LINE: behaviorAvoidLine(); break;
    case StrategyState::RECOVER: behaviorRecover(); break;
  }
}

// ============================================================================
// 13. POST / SETUP / LOOP
// ============================================================================
bool runPOST() {
  bool ok = true;
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[POST] TCA9548A mux not found");
    espSendAscii("LOG:POST TCA9548A FAIL");
    ok = false;
  }
  bool anyColour = false;
  for (int i = 0; i < NUM_COLOUR; i++) anyColour |= colourGood[i];
  if (!anyColour) {
    Serial.println("[POST] no TCS34725 colour sensors found");
    espSendAscii("LOG:POST COLOUR WARN");
  }
  espSendAscii(ok ? "LOG:POST OK CAMERA_ONLY" : "LOG:POST FAIL CAMERA_ONLY");
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] RoboCap Teensy FULL FSM - camera ball, no gyro, no IR");

  sysState = S_POWER_ON;
  for (int i = 0; i < 4; i++) ESP_PORT[i]->begin(UART_BAUD);
  Wire.begin();

  motorsInit();
  actuatorsInit();
  colourInit();
  rcjInit();
  i2cScan();

  sysState = S_POST;
  bool ok = runPOST();

  if (AUTO_GAME_ON_BOOT && (ok || AUTO_GAME_IGNORE_POST_FAIL)) {
    sysState = S_GAME;
    Serial.println("[BOOT] AUTO GAME ENABLED - bench only");
  } else {
    sysState = ok ? S_READY : S_FAULT;
  }

  espPushRobotState(sysState);
  Serial.printf("[BOOT] POST=%s -> state=%s\n", ok ? "OK" : "FAIL", sysName(sysState));
}

void loop() {
  espRxPump();
  updateBallSummary();

  if (millis() - lastCamDebug > 2000) {
    lastCamDebug = millis();
    Serial.printf("[CAM] pkts F=%u R=%u Rear=%u L=%u | raw S1=%u S2=%u S3=%u S4=%u | ball=%d a=%d d=%d held=%d\n",
                  camPkt[1], camPkt[2], camPkt[3], camPkt[4], rawByte[0], rawByte[1], rawByte[2], rawByte[3],
                  ballVisible ? 1 : 0, (int)ballAngle, ballStrength, ballInPocket ? 1 : 0);
    camPkt[1] = camPkt[2] = camPkt[3] = camPkt[4] = 0;
    rawByte[0] = rawByte[1] = rawByte[2] = rawByte[3] = 0;
  }

  if (runEdge) {
    runEdge = false;
    Serial.printf("[RCJ] edge runSignal=%d state=%s\n", runSignal, sysName(sysState));
    if (!AUTO_GAME_ON_BOOT) {
      if (sysState == S_GAME && !runSignal) { gamePrevState = S_GAME; sysState = S_PAUSED; motorKill(); dribblerSet(0); espPushRobotState(sysState); }
      else if (sysState == S_PAUSED && runSignal) { sysState = S_GAME; espPushRobotState(sysState); }
      else if (sysState == S_READY && runSignal) { sysState = S_GAME; espPushRobotState(sysState); }
      else if (sysState == S_TEST && runSignal) {
        if (TEST_IGNORE_RCJ_GO || testEnteredFromFault) {
          motorKill(); dribblerSet(0); espSendAscii(testEnteredFromFault ? "ERR:POST_FAIL_GAME_LOCKED" : "LOG:RCJ_GO_IGNORED_IN_TEST");
        } else { motorKill(); dribblerSet(0); sysState = S_GAME; espPushRobotState(sysState); }
      }
    }
  }

  switch (sysState) {
    case S_GAME:
      if (REQUIRE_SW2_ENABLE && digitalRead(PIN_SW2) != SW2_ENABLE_LEVEL) { motorKill(); break; }
      lineUpdate();
      runStrategy();
      break;

    case S_TEST:
      lineUpdate();
      testDeadmanUpdate();
      break;

    case S_PAUSED:
    case S_READY:
    case S_FAULT:
    default:
      motorKill();
      dribblerSet(0);
      break;
  }

  if (millis() - lastTlm > 200) {
    lastTlm = millis();
    sendTLM();
  }
}
