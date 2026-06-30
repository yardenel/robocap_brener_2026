// ============================================================================
// RoboCap 2026 - Teensy 4.1 MAIN
// Camera strategy + TEST mode.
// Uses: front/rear cameras, TCS34725 colour sensors, 4 omni motors, dribbler.
// Does NOT use: gyro, IR, kicker.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "robot_protocol.h"

#ifndef OBJ_BALL
#define OBJ_BALL 0x05
#endif

// ============================================================================
// 1. USER TUNING
// ============================================================================
static constexpr bool AUTO_GAME_ON_BOOT = true;   // true for testing. false for RCJ Run/Stop.
static constexpr bool ATTACK_YELLOW_GOAL = false; // false = attack blue, true = attack yellow.

static constexpr float DRIVE_MAX = 0.86f;
static constexpr uint8_t DRIBBLER_IDLE_PWM = 135;
static constexpr uint8_t DRIBBLER_CATCH_PWM = 230;
static constexpr uint8_t DRIBBLER_HOLD_PWM = 210;

static constexpr uint32_t VISION_TIMEOUT_MS = 420;
static constexpr uint32_t BALL_LOST_TO_SEARCH_MS = 650;
static constexpr uint32_t HELD_MEMORY_MS = 850;
static constexpr uint32_t CAPTURE_SETTLE_MS = 180;

static constexpr uint8_t BALL_APPROACH_DIST = 45;
static constexpr uint8_t BALL_CLOSE_DIST = 125;
static constexpr uint8_t BALL_VERY_CLOSE_DIST = 165;
static constexpr int BALL_CENTER_DEG = 12;
static constexpr int BALL_NEAR_CENTER_DEG = 20;

static constexpr uint8_t GOAL_CLOSE_DIST = 90;
static constexpr int GOAL_CENTER_DEG = 15;
static constexpr uint32_t GOAL_LOST_SCAN_MS = 350;

static constexpr float BALL_TURN_KP = 0.018f;
static constexpr float GOAL_TURN_KP = 0.014f;
static constexpr float SEARCH_ROTATE = 0.38f;
static constexpr float FAST_SEARCH_ROTATE = 0.50f;

static constexpr uint16_t LINE_CLEAR_MIN = 1500;
static constexpr uint16_t LINE_RGB_SPREAD_MAX = 420;
static constexpr uint8_t CAMERA_LINE_CLOSE_DIST = 150;
static constexpr uint32_t LINE_ESCAPE_MS = 320;
static constexpr uint32_t LINE_COOLDOWN_MS = 250;

static constexpr uint32_t SAME_STATE_WIGGLE_MS = 1600;
static constexpr uint32_t WIGGLE_MS = 260;
static constexpr uint32_t TEST_DEADMAN_MS = 450;

// ============================================================================
// 2. PIN MAP
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

#define TCA_ADDR 0x70
#define TCA_A0   4
#define TCA_A1   5
#define TCA_A2   6
#define TCA_RST  10
#define NUM_COLOUR 4
const uint8_t COLOUR_CH[NUM_COLOUR] = {0, 1, 2, 3};

const int MOTOR_INV[5] = {
  0,
  1,    // ENG1 RF
  -1,   // ENG2 RR
  -1,   // ENG3 LR
  1     // ENG4 LF
};

HardwareSerial *const ESP_PORT[4] = {&Serial1, &Serial2, &Serial3, &Serial4};
const int MOUNT_DEG[5] = {0, 0, 90, 180, 270};
const float COLOUR_BEARING[NUM_COLOUR] = {315, 45, 135, 225};

// ============================================================================
// 3. TYPES / GLOBAL STATE
// ============================================================================
enum RobotState : uint8_t {
  SYS_READY,
  SYS_GAME,
  SYS_TEST,
  SYS_PAUSED
};

enum StrategyState : uint8_t {
  ST_BOOT_HOLD,
  ST_SEARCH_SPIN,
  ST_SEARCH_SWEEP,
  ST_REAR_BALL_TURN,
  ST_ALIGN_BALL,
  ST_APPROACH_BALL,
  ST_CAPTURE_SETTLE,
  ST_HAS_BALL_FIND_GOAL,
  ST_ALIGN_GOAL,
  ST_DRIVE_TO_GOAL,
  ST_LOST_BALL_RECOVER,
  ST_LINE_ESCAPE,
  ST_WIGGLE_UNSTUCK
};

struct SeenObj {
  bool seen = false;
  int angle = 0;
  uint8_t dist = 0;
  uint32_t t = 0;
};

struct Vision {
  SeenObj ballFront;
  SeenObj ballRear;
  SeenObj ballRight;
  SeenObj ballLeft;
  SeenObj yellowGoal;
  SeenObj blueGoal;
  SeenObj whiteLine;
};

struct Parser {
  uint8_t phase = 0;
  uint8_t id = 0;
  uint8_t obj = 0;
  int8_t angle = 0;
  char ascii[128] = {0};
  uint8_t asciiLen = 0;
};

RobotState robotState = SYS_READY;
StrategyState strat = ST_BOOT_HOLD;
StrategyState returnAfterWiggle = ST_SEARCH_SPIN;
Vision vision;
Parser parser[4];

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
bool colourGood[NUM_COLOUR] = {false};
bool lineDetected = false;
uint8_t lineMask = 0;
float lineEscapeBearing = 180;
uint32_t lineEscapeUntil = 0;
uint32_t lineCooldownUntil = 0;

uint32_t stateSinceMs = 0;
uint32_t timedStateUntil = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastBallFreshMs = 0;
uint32_t lastHeldMs = 0;
uint32_t lastTestDriveCmdMs = 0;
int lastKnownBallAngle = 0;
int lastKnownGoalAngle = 0;
bool testMode = false;

char usbLine[128] = {0};
uint8_t usbLineLen = 0;

// ============================================================================
// 4. HELPERS
// ============================================================================
float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

int wrap180i(int a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

bool fresh(const SeenObj &o) {
  return o.seen && (millis() - o.t <= VISION_TIMEOUT_MS);
}

void setStrategy(StrategyState next) {
  if (strat == next) return;
  strat = next;
  stateSinceMs = millis();
}

uint32_t stateAge() {
  return millis() - stateSinceMs;
}

// ============================================================================
// 5. MOTORS / DRIBBLER
// ============================================================================
void setMotorRaw(int d1, int d2, int pwm, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(d1, HIGH);
    digitalWrite(d2, LOW);
    analogWrite(pwm, speed);
  } else if (speed < 0) {
    digitalWrite(d1, LOW);
    digitalWrite(d2, HIGH);
    analogWrite(pwm, -speed);
  } else {
    digitalWrite(d1, LOW);
    digitalWrite(d2, LOW);
    analogWrite(pwm, 0);
  }
}

void motorKill() {
  setMotorRaw(ENG1_DR1, ENG1_DR2, ENG1_SP, 0);
  setMotorRaw(ENG2_DR1, ENG2_DR2, ENG2_SP, 0);
  setMotorRaw(ENG3_DR1, ENG3_DR2, ENG3_SP, 0);
  setMotorRaw(ENG4_DR1, ENG4_DR2, ENG4_SP, 0);
}

void omniDrive(float vx, float vy, float omega) {
  float vRF = vy - vx - omega;
  float vRR = vy + vx - omega;
  float vLR = vy - vx + omega;
  float vLF = vy + vx + omega;

  float m = max(max(fabs(vRF), fabs(vRR)), max(fabs(vLR), fabs(vLF)));
  if (m > 1.0f) {
    vRF /= m;
    vRR /= m;
    vLR /= m;
    vLF /= m;
  }

  setMotorRaw(ENG1_DR1, ENG1_DR2, ENG1_SP, (int)(vRF * 255 * DRIVE_MAX * MOTOR_INV[1]));
  setMotorRaw(ENG2_DR1, ENG2_DR2, ENG2_SP, (int)(vRR * 255 * DRIVE_MAX * MOTOR_INV[2]));
  setMotorRaw(ENG3_DR1, ENG3_DR2, ENG3_SP, (int)(vLR * 255 * DRIVE_MAX * MOTOR_INV[3]));
  setMotorRaw(ENG4_DR1, ENG4_DR2, ENG4_SP, (int)(vLF * 255 * DRIVE_MAX * MOTOR_INV[4]));
}

void driveBearing(float bearingDeg, float speed, float omega) {
  float r = bearingDeg * DEG_TO_RAD;
  float vx = sinf(r) * speed;
  float vy = cosf(r) * speed;
  omniDrive(vx, vy, omega);
}

void setSingleMotorPercent(int n, int dir, int pct) {
  n = constrain(n, 1, 4);
  pct = constrain(pct, 0, 100);
  int speed = map(pct, 0, 100, 0, 255);
  if (dir == 0) speed = -speed;
  speed *= MOTOR_INV[n];

  if (n == 1) setMotorRaw(ENG1_DR1, ENG1_DR2, ENG1_SP, speed);
  if (n == 2) setMotorRaw(ENG2_DR1, ENG2_DR2, ENG2_SP, speed);
  if (n == 3) setMotorRaw(ENG3_DR1, ENG3_DR2, ENG3_SP, speed);
  if (n == 4) setMotorRaw(ENG4_DR1, ENG4_DR2, ENG4_SP, speed);
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
    analogWriteFrequency(sp[i], 1000);
  }

  pinMode(PIN_DRIBBLER, OUTPUT);
  analogWriteFrequency(PIN_DRIBBLER, 20000);
  dribblerSet(0);
  motorKill();
}

// ============================================================================
// 6. COLOUR / LINE SENSORS
// ============================================================================
void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void tcaDeselect() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

void tcaReset() {
  digitalWrite(TCA_RST, LOW);
  delayMicroseconds(30);
  digitalWrite(TCA_RST, HIGH);
  delayMicroseconds(80);
  tcaDeselect();
}

void colourInit() {
  pinMode(TCA_A0, OUTPUT);
  pinMode(TCA_A1, OUTPUT);
  pinMode(TCA_A2, OUTPUT);
  digitalWrite(TCA_A0, LOW);
  digitalWrite(TCA_A1, LOW);
  digitalWrite(TCA_A2, LOW);

  pinMode(TCA_RST, OUTPUT);
  digitalWrite(TCA_RST, HIGH);
  tcaReset();

  Serial.print("[COLOUR] good:");
  for (int i = 0; i < NUM_COLOUR; i++) {
    tcaSelect(COLOUR_CH[i]);
    Wire.beginTransmission(0x29);
    colourGood[i] = (Wire.endTransmission() == 0);
    if (colourGood[i]) {
      tcs.begin();
      Serial.printf(" ch%d", COLOUR_CH[i]);
    }
    tcaReset();
  }
  Serial.println();
}

bool isWhiteLine(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  uint16_t mx = max(r, max(g, b));
  uint16_t mn = min(r, min(g, b));
  return (c >= LINE_CLEAR_MIN) && ((mx - mn) <= LINE_RGB_SPREAD_MAX);
}

void colourScan() {
  lineDetected = false;
  lineMask = 0;

  float sx = 0;
  float sy = 0;
  int n = 0;

  for (int i = 0; i < NUM_COLOUR; i++) {
    if (!colourGood[i]) continue;

    tcaSelect(COLOUR_CH[i]);
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    tcaDeselect();

    if (isWhiteLine(r, g, b, c)) {
      lineDetected = true;
      lineMask |= (1 << i);

      float a = COLOUR_BEARING[i] * DEG_TO_RAD;
      sx += sinf(a);
      sy += cosf(a);
      n++;
    }
  }

  if (n > 0) {
    float towardLine = atan2f(sx, sy) * RAD_TO_DEG;
    lineEscapeBearing = wrap180(towardLine + 180.0f);
  }
}

void printColourRaw() {
  Serial.print("COLOUR:");
  for (int i = 0; i < NUM_COLOUR; i++) {
    if (!colourGood[i]) {
      Serial.printf(" S%d=NA", i);
      continue;
    }
    tcaSelect(COLOUR_CH[i]);
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    tcaDeselect();
    Serial.printf(" S%d R=%u G=%u B=%u C=%u", i, r, g, b, c);
  }
  Serial.println();
}

// ============================================================================
// 7. VISION UART PARSER + ASCII TEST COMMANDS FROM ESP
// ============================================================================
SeenObj &ballSlotByCamera(uint8_t espID) {
  if (espID == ESP_ID_REAR) return vision.ballRear;
  if (espID == ESP_ID_RIGHT) return vision.ballRight;
  if (espID == ESP_ID_LEFT) return vision.ballLeft;
  return vision.ballFront;
}

void updateVisionFromPacket(uint8_t espID, uint8_t obj, int8_t camAngle, uint8_t dist) {
  if (espID < ESP_ID_FRONT || espID > ESP_ID_LEFT) return;

  int robotAngle = wrap180i((int)camAngle + MOUNT_DEG[espID]);
  uint32_t now = millis();

  if (obj == OBJ_BALL) {
    SeenObj &b = ballSlotByCamera(espID);
    b.seen = true;
    b.angle = robotAngle;
    b.dist = dist;
    b.t = now;
    lastKnownBallAngle = robotAngle;
    lastBallFreshMs = now;
  } else if (obj == OBJ_GOAL_YELLOW || obj == OBJ_GOAL) {
    vision.yellowGoal.seen = true;
    vision.yellowGoal.angle = robotAngle;
    vision.yellowGoal.dist = dist;
    vision.yellowGoal.t = now;
    if (ATTACK_YELLOW_GOAL) lastKnownGoalAngle = robotAngle;
  } else if (obj == OBJ_GOAL_BLUE) {
    vision.blueGoal.seen = true;
    vision.blueGoal.angle = robotAngle;
    vision.blueGoal.dist = dist;
    vision.blueGoal.t = now;
    if (!ATTACK_YELLOW_GOAL) lastKnownGoalAngle = robotAngle;
  } else if (obj == OBJ_LINE_WHITE) {
    vision.whiteLine.seen = true;
    vision.whiteLine.angle = robotAngle;
    vision.whiteLine.dist = dist;
    vision.whiteLine.t = now;
  }
}

SeenObj opponentGoal();
bool ballHeldMemory();
const char *stateName(StrategyState s);
void telemetry();

void handleTestCommand(const char *line) {
  if (!line || !line[0]) return;

  if (strcmp(line, "TEST:ON") == 0) {
    testMode = true;
    robotState = SYS_TEST;
    motorKill();
    dribblerSet(0);
    Serial.println("ACK:TEST_ON");
    return;
  }

  if (strcmp(line, "TEST:OFF") == 0) {
    testMode = false;
    motorKill();
    dribblerSet(0);
    setStrategy(ST_BOOT_HOLD);
    Serial.println("ACK:TEST_OFF");
    return;
  }

  if (strcmp(line, "ESTOP") == 0) {
    motorKill();
    dribblerSet(0);
    testMode = true;
    robotState = SYS_TEST;
    Serial.println("ACK:ESTOP");
    return;
  }

  if (strcmp(line, "QUERY:STATUS") == 0) {
    telemetry();
    Serial.println("ACK:STATUS");
    return;
  }

  if (strcmp(line, "COLOUR:RAW") == 0 || strcmp(line, "COLOR:RAW") == 0) {
    printColourRaw();
    Serial.println("ACK:COLOUR_RAW");
    return;
  }

  if (!testMode) {
    Serial.print("ERR:NOT_IN_TEST ");
    Serial.println(line);
    return;
  }

  int n, dir, pct;
  if (sscanf(line, "MOTOR:%d:%d:%d", &n, &dir, &pct) == 3) {
    setSingleMotorPercent(n, dir, pct);
    lastTestDriveCmdMs = millis();
    Serial.printf("ACK:MOTOR:%d:%d:%d\n", n, dir, pct);
    return;
  }

  int vx, vy, r;
  if (sscanf(line, "OMNI:%d:%d:%d", &vx, &vy, &r) == 3) {
    omniDrive(constrain(vx, -100, 100) / 100.0f,
              constrain(vy, -100, 100) / 100.0f,
              constrain(r, -100, 100) / 100.0f);
    lastTestDriveCmdMs = millis();
    Serial.printf("ACK:OMNI:%d:%d:%d\n", vx, vy, r);
    return;
  }

  if (sscanf(line, "DRIBBLER:%d", &pct) == 1) {
    pct = constrain(pct, 0, 100);
    dribblerSet((uint8_t)map(pct, 0, 100, 0, 255));
    Serial.printf("ACK:DRIBBLER:%d\n", pct);
    return;
  }

  Serial.print("ERR:UNKNOWN_CMD ");
  Serial.println(line);
}

void handleAsciiByte(char *buf, uint8_t &len, uint8_t b) {
  if (b == '\r') return;
  if (b == '\n') {
    buf[len] = 0;
    handleTestCommand(buf);
    len = 0;
    return;
  }
  if (b >= 32 && b <= 126 && len < 127) {
    buf[len++] = (char)b;
  }
}

void readUsbCommands() {
  while (Serial.available()) {
    handleAsciiByte(usbLine, usbLineLen, Serial.read());
  }
}

void readEspPort(int idx) {
  HardwareSerial &s = *ESP_PORT[idx];
  Parser &p = parser[idx];

  while (s.available()) {
    uint8_t b = s.read();

    if (p.phase == 0) {
      if (b >= ESP_ID_FRONT && b <= ESP_ID_LEFT) {
        p.id = b;
        p.phase = 1;
      } else {
        handleAsciiByte(p.ascii, p.asciiLen, b);
      }
    } else if (p.phase == 1) {
      if (b == PACKET_NO_DETECT) {
        p.phase = 0;
      } else {
        p.obj = b;
        p.phase = 2;
      }
    } else if (p.phase == 2) {
      p.angle = (int8_t)b;
      p.phase = 3;
    } else {
      updateVisionFromPacket(p.id, p.obj, p.angle, b);
      p.phase = 0;
    }
  }
}

void readAllEsp() {
  for (int i = 0; i < 4; i++) readEspPort(i);
  readUsbCommands();
}

// ============================================================================
// 8. STRATEGY HELPERS
// ============================================================================
SeenObj opponentGoal() {
  return ATTACK_YELLOW_GOAL ? vision.yellowGoal : vision.blueGoal;
}

SeenObj bestBall() {
  if (fresh(vision.ballFront)) return vision.ballFront;
  if (fresh(vision.ballRear)) return vision.ballRear;
  if (fresh(vision.ballRight)) return vision.ballRight;
  if (fresh(vision.ballLeft)) return vision.ballLeft;
  return SeenObj();
}

bool frontBallCloseAndCentered() {
  return fresh(vision.ballFront) &&
         abs(vision.ballFront.angle) <= BALL_CENTER_DEG &&
         vision.ballFront.dist >= BALL_CLOSE_DIST;
}

bool ballHeldMemory() {
  if (frontBallCloseAndCentered()) {
    lastHeldMs = millis();
    return true;
  }
  return millis() - lastHeldMs <= HELD_MEMORY_MS;
}

bool shouldTriggerLineEscape() {
  if (millis() < lineCooldownUntil) return false;
  bool cameraLineClose = fresh(vision.whiteLine) && vision.whiteLine.dist >= CAMERA_LINE_CLOSE_DIST;
  return lineDetected || cameraLineClose;
}

void enterTimedState(StrategyState next, uint32_t durationMs) {
  setStrategy(next);
  timedStateUntil = millis() + durationMs;
}

void enterLineEscape() {
  dribblerSet(0);
  enterTimedState(ST_LINE_ESCAPE, LINE_ESCAPE_MS);
}

void maybeWiggle(StrategyState after) {
  if (stateAge() > SAME_STATE_WIGGLE_MS) {
    returnAfterWiggle = after;
    enterTimedState(ST_WIGGLE_UNSTUCK, WIGGLE_MS);
  }
}

// ============================================================================
// 9. STRATEGY ACTIONS
// ============================================================================
void searchSpin() {
  dribblerSet(0);
  uint32_t phase = (millis() / 1500) % 4;
  float rot = (phase < 2) ? SEARCH_ROTATE : -SEARCH_ROTATE;
  float drift = (phase == 1 || phase == 3) ? 0.08f : 0.0f;
  omniDrive(0.0f, drift, rot);
}

void searchSweep() {
  dribblerSet(0);
  float sign = (lastKnownBallAngle >= 0) ? -1.0f : 1.0f;
  omniDrive(0.0f, 0.04f, FAST_SEARCH_ROTATE * sign);
}

void turnToRearBall(const SeenObj &ball) {
  dribblerSet(DRIBBLER_IDLE_PWM);
  float omega = clampf(-ball.angle * BALL_TURN_KP, -0.62f, 0.62f);
  omniDrive(0.0f, 0.0f, omega);
}

void alignToFrontBall(const SeenObj &ball) {
  dribblerSet(DRIBBLER_IDLE_PWM);
  int a = ball.angle;
  float omega = clampf(-a * BALL_TURN_KP, -0.50f, 0.50f);
  float forward = (abs(a) <= BALL_NEAR_CENTER_DEG) ? 0.20f : 0.05f;
  float side = sinf(a * DEG_TO_RAD) * 0.15f;
  omniDrive(side, forward, omega);
}

void approachBall(const SeenObj &ball) {
  dribblerSet(DRIBBLER_CATCH_PWM);
  int a = ball.angle;
  float omega = clampf(-a * BALL_TURN_KP, -0.36f, 0.36f);
  float side = sinf(a * DEG_TO_RAD) * 0.20f;
  float forward;
  if (ball.dist < BALL_APPROACH_DIST) forward = 0.58f;
  else if (ball.dist < BALL_CLOSE_DIST) forward = 0.46f;
  else forward = 0.32f;
  omniDrive(side, forward, omega);
}

void captureSettle() {
  dribblerSet(DRIBBLER_CATCH_PWM);
  omniDrive(0.0f, 0.22f, 0.0f);
}

void findGoalWithBall() {
  dribblerSet(DRIBBLER_HOLD_PWM);
  float rot = (lastKnownGoalAngle >= 0) ? -0.28f : 0.28f;
  omniDrive(0.0f, 0.12f, rot);
}

void alignGoalWithBall(const SeenObj &goal) {
  dribblerSet(DRIBBLER_HOLD_PWM);
  int ga = goal.angle;
  float omega = clampf(-ga * GOAL_TURN_KP, -0.42f, 0.42f);
  float side = sinf(ga * DEG_TO_RAD) * 0.25f;
  omniDrive(side, 0.24f, omega);
}

void driveThroughGoal(const SeenObj &goal) {
  dribblerSet(DRIBBLER_HOLD_PWM);
  int ga = goal.angle;
  float omega = clampf(-ga * GOAL_TURN_KP, -0.35f, 0.35f);
  float side = sinf(ga * DEG_TO_RAD) * 0.18f;
  float forward = (goal.dist >= GOAL_CLOSE_DIST) ? 0.82f : 0.66f;
  omniDrive(side, forward, omega);
}

void lostBallRecover() {
  dribblerSet(DRIBBLER_IDLE_PWM);
  float omega = clampf(-lastKnownBallAngle * BALL_TURN_KP, -0.44f, 0.44f);
  omniDrive(0.0f, 0.05f, omega);
}

void lineEscape() {
  dribblerSet(0);
  driveBearing(lineEscapeBearing, 0.50f, 0.0f);

  if (millis() >= timedStateUntil) {
    lineCooldownUntil = millis() + LINE_COOLDOWN_MS;
    setStrategy(ST_SEARCH_SPIN);
  }
}

void wiggleUnstuck() {
  dribblerSet(DRIBBLER_IDLE_PWM);
  uint32_t phase = (millis() / 90) % 2;
  float side = phase ? 0.35f : -0.35f;
  omniDrive(side, -0.10f, 0.0f);

  if (millis() >= timedStateUntil) {
    setStrategy(returnAfterWiggle);
  }
}

// ============================================================================
// 10. STRATEGY FSM
// ============================================================================
void chooseNextState() {
  if (shouldTriggerLineEscape() && strat != ST_LINE_ESCAPE) {
    enterLineEscape();
    return;
  }

  SeenObj ball = bestBall();
  SeenObj goal = opponentGoal();
  bool frontBall = fresh(vision.ballFront);
  bool rearBall = fresh(vision.ballRear);
  bool anyBall = fresh(ball);
  bool held = ballHeldMemory();
  bool goalSeen = fresh(goal);

  if (strat == ST_BOOT_HOLD) {
    if (stateAge() > 250) setStrategy(ST_SEARCH_SPIN);
    return;
  }

  if (strat == ST_LINE_ESCAPE || strat == ST_WIGGLE_UNSTUCK || strat == ST_CAPTURE_SETTLE) return;

  if (held) {
    if (!goalSeen) setStrategy(ST_HAS_BALL_FIND_GOAL);
    else if (abs(goal.angle) <= GOAL_CENTER_DEG) setStrategy(ST_DRIVE_TO_GOAL);
    else setStrategy(ST_ALIGN_GOAL);
    return;
  }

  if (frontBall) {
    if (frontBallCloseAndCentered() || vision.ballFront.dist >= BALL_VERY_CLOSE_DIST) {
      lastHeldMs = millis();
      enterTimedState(ST_CAPTURE_SETTLE, CAPTURE_SETTLE_MS);
    } else if (abs(vision.ballFront.angle) <= BALL_NEAR_CENTER_DEG) {
      setStrategy(ST_APPROACH_BALL);
    } else {
      setStrategy(ST_ALIGN_BALL);
    }
    return;
  }

  if (rearBall || anyBall) {
    setStrategy(ST_REAR_BALL_TURN);
    return;
  }

  if (millis() - lastBallFreshMs < BALL_LOST_TO_SEARCH_MS) {
    setStrategy(ST_LOST_BALL_RECOVER);
    return;
  }

  if (stateAge() > 2200) setStrategy(ST_SEARCH_SWEEP);
  else setStrategy(ST_SEARCH_SPIN);
}

void executeState() {
  SeenObj ball = bestBall();
  SeenObj goal = opponentGoal();

  switch (strat) {
    case ST_BOOT_HOLD: motorKill(); dribblerSet(0); break;
    case ST_SEARCH_SPIN: searchSpin(); maybeWiggle(ST_SEARCH_SPIN); break;
    case ST_SEARCH_SWEEP: searchSweep(); maybeWiggle(ST_SEARCH_SPIN); break;
    case ST_REAR_BALL_TURN: turnToRearBall(ball); maybeWiggle(ST_SEARCH_SPIN); break;
    case ST_ALIGN_BALL: alignToFrontBall(vision.ballFront); maybeWiggle(ST_SEARCH_SPIN); break;
    case ST_APPROACH_BALL: approachBall(vision.ballFront); maybeWiggle(ST_LOST_BALL_RECOVER); break;
    case ST_CAPTURE_SETTLE:
      captureSettle();
      if (millis() >= timedStateUntil) {
        lastHeldMs = millis();
        setStrategy(ST_HAS_BALL_FIND_GOAL);
      }
      break;
    case ST_HAS_BALL_FIND_GOAL: findGoalWithBall(); maybeWiggle(ST_SEARCH_SPIN); break;
    case ST_ALIGN_GOAL: alignGoalWithBall(goal); maybeWiggle(ST_HAS_BALL_FIND_GOAL); break;
    case ST_DRIVE_TO_GOAL:
      driveThroughGoal(goal);
      if (!fresh(goal) && stateAge() > GOAL_LOST_SCAN_MS) setStrategy(ST_HAS_BALL_FIND_GOAL);
      break;
    case ST_LOST_BALL_RECOVER:
      lostBallRecover();
      if (stateAge() > BALL_LOST_TO_SEARCH_MS) setStrategy(ST_SEARCH_SWEEP);
      break;
    case ST_LINE_ESCAPE: lineEscape(); break;
    case ST_WIGGLE_UNSTUCK: wiggleUnstuck(); break;
    default: setStrategy(ST_SEARCH_SPIN); break;
  }
}

void strategyTick() {
  readAllEsp();
  colourScan();
  chooseNextState();
  executeState();
}

void testTick() {
  readAllEsp();
  colourScan();

  if (lastTestDriveCmdMs && millis() - lastTestDriveCmdMs > TEST_DEADMAN_MS) {
    motorKill();
    lastTestDriveCmdMs = 0;
  }
}

// ============================================================================
// 11. TELEMETRY / SETUP / LOOP
// ============================================================================
const char *stateName(StrategyState s) {
  switch (s) {
    case ST_BOOT_HOLD: return "BOOT";
    case ST_SEARCH_SPIN: return "SEARCH_SPIN";
    case ST_SEARCH_SWEEP: return "SEARCH_SWEEP";
    case ST_REAR_BALL_TURN: return "REAR_TURN";
    case ST_ALIGN_BALL: return "ALIGN_BALL";
    case ST_APPROACH_BALL: return "APPROACH";
    case ST_CAPTURE_SETTLE: return "CAPTURE";
    case ST_HAS_BALL_FIND_GOAL: return "FIND_GOAL";
    case ST_ALIGN_GOAL: return "ALIGN_GOAL";
    case ST_DRIVE_TO_GOAL: return "DRIVE_GOAL";
    case ST_LOST_BALL_RECOVER: return "RECOVER";
    case ST_LINE_ESCAPE: return "LINE";
    case ST_WIGGLE_UNSTUCK: return "WIGGLE";
    default: return "?";
  }
}

void telemetry() {
  if (millis() - lastTelemetryMs < 220) return;
  lastTelemetryMs = millis();

  SeenObj ball = bestBall();
  SeenObj goal = opponentGoal();
  int st = testMode ? ROBOT_STATE_TEST : (robotState == SYS_GAME ? ROBOT_STATE_GAME : ROBOT_STATE_READY);
  int ballVis = fresh(ball) ? 1 : 0;
  int goalAng = fresh(goal) ? goal.angle : -1;

  // Old TEST webapp-friendly line shape.
  Serial.printf("TLM:%d,0,%d,%d,%u,%d,%d,0,%d\n",
                st, ballVis, ballVis ? ball.angle : -1, ballVis ? ball.dist : 0,
                ballHeldMemory(), lineDetected, goalAng);

  Serial.printf(
    "LOG:ST:%s bf=%d ba=%d bd=%u br=%d ra=%d rd=%u held=%d goal=%d ga=%d gd=%u line=%d mask=0x%02X esc=%.0f\n",
    stateName(strat),
    fresh(vision.ballFront), vision.ballFront.angle, vision.ballFront.dist,
    fresh(vision.ballRear), vision.ballRear.angle, vision.ballRear.dist,
    ballHeldMemory(),
    fresh(goal), goal.angle, goal.dist,
    lineDetected, lineMask, lineEscapeBearing
  );
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] RoboCap Teensy camera FSM + TEST mode: no gyro, no IR, no kicker");

  for (int i = 0; i < 4; i++) {
    ESP_PORT[i]->begin(UART_BAUD);
  }

  pinMode(PIN_RCJ_RUN, INPUT_PULLDOWN);

  Wire.begin();
  motorsInit();
  colourInit();

  stateSinceMs = millis();
  robotState = AUTO_GAME_ON_BOOT ? SYS_GAME : SYS_READY;

  Serial.printf("[BOOT] AUTO_GAME_ON_BOOT=%d ATTACK=%s\n",
                AUTO_GAME_ON_BOOT,
                ATTACK_YELLOW_GOAL ? "YELLOW" : "BLUE");
}

void loop() {
  readAllEsp();

  if (testMode) {
    robotState = SYS_TEST;
    testTick();
    telemetry();
    return;
  }

  bool go = AUTO_GAME_ON_BOOT || (digitalRead(PIN_RCJ_RUN) == HIGH);

  if (!go) {
    robotState = SYS_READY;
    setStrategy(ST_BOOT_HOLD);
    motorKill();
    dribblerSet(0);
    telemetry();
    return;
  }

  robotState = SYS_GAME;
  strategyTick();
  telemetry();
}
