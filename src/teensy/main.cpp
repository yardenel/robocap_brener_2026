// ============================================================================
// RoboCap 2026 - Teensy 4.1 MAIN
// Strategy: CAMERA ONLY + TCS34725 colour sensors + dribbler.
// No gyro. No IR. No kicker.
// Front camera is centered above the dribbler. Rear camera is used for search.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "robot_protocol.h"

#ifndef OBJ_BALL
#define OBJ_BALL 0x05
#endif

// ============================================================================
// USER TUNING
// ============================================================================
static constexpr bool AUTO_GAME_ON_BOOT = true;   // true = runs immediately for testing. Set false for RCJ Run/Stop.
static constexpr bool ATTACK_YELLOW_GOAL = false; // false = attack blue goal, true = attack yellow goal.

static constexpr float DRIVE_MAX = 0.85f;
static constexpr uint8_t DRIBBLER_PWM = 225;
static constexpr uint32_t VISION_TIMEOUT_MS = 450;
static constexpr uint32_t LINE_ESCAPE_MS = 280;

// Ball capture by camera only. Tune on field.
static constexpr uint8_t BALL_CLOSE_DIST = 125;   // ESP distance proxy 0..255
static constexpr int BALL_CENTER_DEG = 13;

// Goal charge. Tune on field.
static constexpr uint8_t GOAL_CLOSE_DIST = 95;
static constexpr int GOAL_CENTER_DEG = 16;

// Camera proportional rotation.
static constexpr float BALL_TURN_KP = 0.018f;
static constexpr float GOAL_TURN_KP = 0.014f;

// TCS white-line threshold. Tune by printing C values above green/white.
static constexpr uint16_t LINE_CLEAR_MIN = 1500;
static constexpr uint16_t LINE_RGB_SPREAD_MAX = 420;

// ============================================================================
// PIN MAP from current PCB/code
// ============================================================================
// Motors: ENG1=RF, ENG2=RR, ENG3=LR, ENG4=LF
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

// TCA9548A + TCS34725
#define TCA_ADDR 0x70
#define TCA_A0   4
#define TCA_A1   5
#define TCA_A2   6
#define TCA_RST  10
#define NUM_COLOUR 4

const uint8_t COLOUR_CH[NUM_COLOUR] = {0, 1, 2, 3};

// If a wheel drives opposite, flip its value.
const int MOTOR_INV[5] = {
  0,
  1,    // ENG1 RF
  -1,   // ENG2 RR
  -1,   // ENG3 LR
  1     // ENG4 LF
};

// ESP UART ports: front/rear are enough, right/left ignored if absent.
HardwareSerial *const ESP_PORT[4] = {&Serial1, &Serial2, &Serial3, &Serial4};
const int MOUNT_DEG[5] = {0, 0, 90, 180, 270}; // ESP_ID index

// Colour sensor direction assumptions.
// 0=front-left, 1=front-right, 2=rear-right, 3=rear-left.
// Change only if the robot escapes lines in the wrong direction.
const float COLOUR_BEARING[NUM_COLOUR] = {315, 45, 135, 225};

// ============================================================================
// BASIC TYPES / STATE
// ============================================================================
enum RobotState : uint8_t {
  S_READY,
  S_GAME,
  S_PAUSED
};

enum StrategyState : uint8_t {
  ST_SEARCH_BALL,
  ST_FACE_BALL,
  ST_CAPTURE_BALL,
  ST_DRIVE_TO_GOAL,
  ST_LINE_ESCAPE
};

struct SeenObj {
  bool seen = false;
  int angle = 0;          // robot-relative, -180..180, 0=front, +CW
  uint8_t dist = 0;       // 0=far/small, 255=close/large
  uint32_t t = 0;
};

struct Vision {
  SeenObj ballFront;
  SeenObj ballRear;
  SeenObj yellowGoal;
  SeenObj blueGoal;
  SeenObj whiteLine;
};

struct Parser {
  uint8_t phase = 0;
  uint8_t id = 0;
  uint8_t obj = 0;
  int8_t angle = 0;
};

RobotState robotState = S_READY;
StrategyState strat = ST_SEARCH_BALL;
Vision vision;
Parser parser[4];

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
bool colourGood[NUM_COLOUR] = {false};
bool lineDetected = false;
uint8_t lineMask = 0;
float lineEscapeBearing = 180;
uint32_t lineEscapeUntil = 0;
uint32_t lastTelemetryMs = 0;

// ============================================================================
// MATH HELPERS
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

bool fresh(const SeenObj &o) {
  return o.seen && (millis() - o.t <= VISION_TIMEOUT_MS);
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ============================================================================
// MOTORS / DRIBBLER
// ============================================================================
void setMotor(int d1, int d2, int pwm, int speed) {
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
  setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, 0);
  setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, 0);
  setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, 0);
  setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, 0);
}

void omniDrive(float vx, float vy, float omega) {
  // vx=right, vy=front, omega=CCW. Inputs -1..1.
  float vRF = vy - vx - omega; // ENG1
  float vRR = vy + vx - omega; // ENG2
  float vLR = vy - vx + omega; // ENG3
  float vLF = vy + vx + omega; // ENG4

  float m = max(max(fabs(vRF), fabs(vRR)), max(fabs(vLR), fabs(vLF)));
  if (m > 1.0f) {
    vRF /= m; vRR /= m; vLR /= m; vLF /= m;
  }

  setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, (int)(vRF * 255 * DRIVE_MAX * MOTOR_INV[1]));
  setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, (int)(vRR * 255 * DRIVE_MAX * MOTOR_INV[2]));
  setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, (int)(vLR * 255 * DRIVE_MAX * MOTOR_INV[3]));
  setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, (int)(vLF * 255 * DRIVE_MAX * MOTOR_INV[4]));
}

void driveBearing(float bearingDeg, float speed, float omega) {
  float r = bearingDeg * DEG_TO_RAD;
  float vx = sinf(r) * speed;
  float vy = cosf(r) * speed;
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
    analogWriteFrequency(sp[i], 1000);
  }

  pinMode(PIN_DRIBBLER, OUTPUT);
  analogWriteFrequency(PIN_DRIBBLER, 20000);
  dribblerSet(0);
  motorKill();
}

// ============================================================================
// COLOUR / LINE SENSORS
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

// ============================================================================
// CAMERA UART PARSER
// ============================================================================
void updateVisionFromPacket(uint8_t espID, uint8_t obj, int8_t camAngle, uint8_t dist) {
  if (espID < ESP_ID_FRONT || espID > ESP_ID_LEFT) return;

  int robotAngle = wrap180i((int)camAngle + MOUNT_DEG[espID]);
  uint32_t now = millis();

  if (obj == OBJ_BALL) {
    SeenObj &b = (espID == ESP_ID_REAR) ? vision.ballRear : vision.ballFront;
    b.seen = true;
    b.angle = robotAngle;
    b.dist = dist;
    b.t = now;
  } else if (obj == OBJ_GOAL_YELLOW || obj == OBJ_GOAL) {
    vision.yellowGoal.seen = true;
    vision.yellowGoal.angle = robotAngle;
    vision.yellowGoal.dist = dist;
    vision.yellowGoal.t = now;
  } else if (obj == OBJ_GOAL_BLUE) {
    vision.blueGoal.seen = true;
    vision.blueGoal.angle = robotAngle;
    vision.blueGoal.dist = dist;
    vision.blueGoal.t = now;
  } else if (obj == OBJ_LINE_WHITE) {
    vision.whiteLine.seen = true;
    vision.whiteLine.angle = robotAngle;
    vision.whiteLine.dist = dist;
    vision.whiteLine.t = now;
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
}

SeenObj opponentGoal() {
  if (ATTACK_YELLOW_GOAL) return vision.yellowGoal;
  return vision.blueGoal;
}

bool hasCameraBall() {
  return fresh(vision.ballFront) &&
         abs(vision.ballFront.angle) <= BALL_CENTER_DEG &&
         vision.ballFront.dist >= BALL_CLOSE_DIST;
}

SeenObj bestBall() {
  if (fresh(vision.ballFront)) return vision.ballFront;
  if (fresh(vision.ballRear)) return vision.ballRear;
  return SeenObj();
}

// ============================================================================
// STRATEGY
// ============================================================================
void enterLineEscape() {
  strat = ST_LINE_ESCAPE;
  lineEscapeUntil = millis() + LINE_ESCAPE_MS;
  dribblerSet(0);
}

void strategyTick() {
  readAllEsp();
  colourScan();

  bool cameraLineClose = fresh(vision.whiteLine) && vision.whiteLine.dist >= 150;
  if ((lineDetected || cameraLineClose) && strat != ST_LINE_ESCAPE) {
    enterLineEscape();
  }

  if (strat == ST_LINE_ESCAPE) {
    dribblerSet(0);
    driveBearing(lineEscapeBearing, 0.45f, 0.0f);
    if (millis() > lineEscapeUntil) {
      strat = ST_SEARCH_BALL;
    }
    return;
  }

  SeenObj ball = bestBall();
  bool held = hasCameraBall();
  SeenObj goal = opponentGoal();
  bool goalSeen = fresh(goal);

  if (held) {
    strat = ST_DRIVE_TO_GOAL;
  } else if (fresh(vision.ballFront)) {
    if (abs(vision.ballFront.angle) <= BALL_CENTER_DEG && vision.ballFront.dist > 70) {
      strat = ST_CAPTURE_BALL;
    } else {
      strat = ST_FACE_BALL;
    }
  } else if (fresh(vision.ballRear)) {
    strat = ST_FACE_BALL;
  } else {
    strat = ST_SEARCH_BALL;
  }

  switch (strat) {
    case ST_SEARCH_BALL: {
      dribblerSet(0);

      // scanning pattern: rotate, with a tiny forward drift to avoid getting stuck.
      static uint32_t phaseStart = 0;
      static int dir = 1;
      if (millis() - phaseStart > 1800) {
        phaseStart = millis();
        dir = -dir;
      }
      omniDrive(0.0f, 0.08f, 0.38f * dir);
      break;
    }

    case ST_FACE_BALL: {
      dribblerSet(150);

      int a = ball.angle;
      float omega = clampf(-a * BALL_TURN_KP, -0.55f, 0.55f);

      if (abs(a) > 95) {
        // Ball is behind/side: rotate first, do not drive away from it.
        omniDrive(0.0f, 0.0f, omega);
      } else {
        // Ball is in front half: drive toward it while rotating the front to it.
        float speed = (ball.dist > 100) ? 0.42f : 0.55f;
        driveBearing(a, speed, omega);
      }
      break;
    }

    case ST_CAPTURE_BALL: {
      dribblerSet(DRIBBLER_PWM);

      int a = vision.ballFront.angle;
      float omega = clampf(-a * BALL_TURN_KP, -0.35f, 0.35f);
      float vx = sinf(a * DEG_TO_RAD) * 0.18f;
      omniDrive(vx, 0.42f, omega);
      break;
    }

    case ST_DRIVE_TO_GOAL: {
      dribblerSet(DRIBBLER_PWM);

      if (goalSeen) {
        int ga = goal.angle;
        float omega = clampf(-ga * GOAL_TURN_KP, -0.45f, 0.45f);
        float side = sinf(ga * DEG_TO_RAD) * 0.25f;

        if (abs(ga) <= GOAL_CENTER_DEG) {
          // Goal centered: keep ball and charge through the goal.
          float speed = (goal.dist >= GOAL_CLOSE_DIST) ? 0.85f : 0.65f;
          omniDrive(side, speed, omega);
        } else {
          // See goal but not centered: rotate/strafe toward it without losing ball.
          omniDrive(side, 0.28f, omega);
        }
      } else {
        // No goal yet: keep ball in dribbler and scan with the front camera.
        omniDrive(0.0f, 0.18f, 0.28f);
      }
      break;
    }

    default:
      strat = ST_SEARCH_BALL;
      break;
  }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
const char *stateName(StrategyState s) {
  switch (s) {
    case ST_SEARCH_BALL: return "SEARCH";
    case ST_FACE_BALL: return "FACE";
    case ST_CAPTURE_BALL: return "CAPTURE";
    case ST_DRIVE_TO_GOAL: return "GOAL";
    case ST_LINE_ESCAPE: return "LINE";
    default: return "?";
  }
}

void telemetry() {
  if (millis() - lastTelemetryMs < 250) return;
  lastTelemetryMs = millis();

  SeenObj ball = bestBall();
  SeenObj goal = opponentGoal();

  Serial.printf("ST:%s ball=%d a=%d d=%u held=%d goal=%d ga=%d gd=%u line=%d mask=0x%02X\n",
                stateName(strat),
                fresh(ball), ball.angle, ball.dist, hasCameraBall(),
                fresh(goal), goal.angle, goal.dist,
                lineDetected, lineMask);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] RoboCap Teensy CAMERA-ONLY strategy: no gyro, no IR");

  for (int i = 0; i < 4; i++) {
    ESP_PORT[i]->begin(UART_BAUD);
  }

  pinMode(PIN_RCJ_RUN, INPUT_PULLDOWN);

  Wire.begin();
  motorsInit();
  colourInit();

  robotState = AUTO_GAME_ON_BOOT ? S_GAME : S_READY;
  Serial.printf("[BOOT] AUTO_GAME_ON_BOOT=%d ATTACK=%s\n",
                AUTO_GAME_ON_BOOT,
                ATTACK_YELLOW_GOAL ? "YELLOW" : "BLUE");
}

void loop() {
  readAllEsp();

  bool go = AUTO_GAME_ON_BOOT || (digitalRead(PIN_RCJ_RUN) == HIGH);

  if (!go) {
    robotState = S_READY;
    motorKill();
    dribblerSet(0);
    telemetry();
    return;
  }

  robotState = S_GAME;
  strategyTick();
  telemetry();
}
