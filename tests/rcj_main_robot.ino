// ============================================================================
// RoboCupJunior Soccer 2026 - MAIN TEENSY 4.1 CODE
// Robot: 4 mecanum wheels + 20 IR sensors + 4 ESP32-S3 cameras
// Author: Team codebase merge
//
// WHAT THIS VERSION DOES:
// - Uses NEW ESP camera protocol from the XIAO ESP32-S3 firmware
// - Ball direction comes ONLY from IR sensors
// - Camera #1 is FRONT camera (aligned with dribbler center)
// - Ball intake/dribbler hole is physically centered between IR sensors 1 and 2
// - Uses gyro stabilization (BNO055)
// - Uses TCS34725 color sensors for white-line escape
// - Handles dribbler + kicker
// - Full mecanum movement
//
// IMPORTANT PHYSICAL ASSUMPTION:
// FRONT OF ROBOT = CAMERA 1 = DRIBBLER SIDE
//
// ============================================================================

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ============================================================================
// ROBOT ID
// ============================================================================

#define THIS_ROBOT_ID 1

// ============================================================================
// MOTOR PINS
// ============================================================================

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

// ============================================================================
// IR SENSOR SYSTEM
// ============================================================================

#define MUX_S0  31
#define MUX_S1  30
#define MUX_S2  29
#define MUX_S3  28
#define MUX_SIG 32

const uint8_t DIRECT_PINS[4] = {24,25,26,27};

#define NUM_IR_SENSORS 20
#define NUM_MUX_CH     16

// IMPORTANT:
// Robot front is BETWEEN sensors 1 and 2 physically.
// So front offset is 9 degrees.

const float IR_SENSOR_ANGLES[NUM_IR_SENSORS] = {
   9,  27,  45,  63,  81,
  99, 117, 135, 153, 171,
 189, 207, 225, 243, 261,
 279, 297, 315, 333, 351
};

// ============================================================================
// COLOR SENSOR SYSTEM
// ============================================================================

#define NUM_COLOR_SENSORS 4

#define TCA_ADDR_A0  4
#define TCA_ADDR_A1  5
#define TCA_ADDR_A2  6
#define TCA_RST_PIN  10
#define TCA_I2C_ADDR 0x70

const float COLOR_SENSOR_ANGLES[NUM_COLOR_SENSORS] = {
  0.0f,
  90.0f,
  180.0f,
  270.0f
};

// ============================================================================
// DRIBBLER / KICKER
// ============================================================================

#define DRIBBLER_PIN 11
#define KICKER_PIN   2

#define DRIBBLER_SPEED 180

#define KICKER_PULSE_MS      80
#define KICKER_COOLDOWN_MS 1500

// ============================================================================
// GYRO
// ============================================================================

#define BNO_RST_PIN 3

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_A, &Wire1);

bool gyroOK = false;

float gyroHeading = 0.0f;
float targetHeading = 0.0f;

// ============================================================================
// CAMERA UARTS
// ============================================================================

// FRONT CAMERA = Serial1
// RIGHT CAMERA = Serial2
// BACK CAMERA  = Serial3
// LEFT CAMERA  = Serial4

#define CAM_FRONT Serial1
#define CAM_RIGHT Serial2
#define CAM_BACK  Serial3
#define CAM_LEFT  Serial4

#define UART_BAUD 115200

// ============================================================================
// NEW CAMERA PROTOCOL
// ============================================================================

#define UART_START_BYTE 0xAA

#define VIS_BALL         0x01
#define VIS_YELLOW_GOAL  0x02
#define VIS_BLUE_GOAL    0x04
#define VIS_WHITE_LINE   0x08
#define VIS_LINE_CLOSE   0x10

#define BALL_NOT_SEEN -32768

struct __attribute__((packed)) CamPacket {
  uint8_t  start;
  uint8_t  len;

  uint8_t  cam_id;
  uint8_t  mount_deg_div10;

  uint32_t frame_seq;

  uint8_t  visible_flags;

  int16_t  ball_angle;
  uint16_t ball_radius_px;

  int16_t  yellow_goal_angle;
  uint16_t yellow_goal_width_px;

  int16_t  blue_goal_angle;
  uint16_t blue_goal_width_px;

  int16_t  line_angle;
  uint8_t  line_dist_band;
  uint8_t  line_row_y;

  uint8_t  ball_confidence;

  uint8_t  checksum;
};

// ============================================================================
// CAMERA STATE
// ============================================================================

struct CameraState {
  bool connected;

  bool blueGoalVisible;
  bool yellowGoalVisible;
  bool lineVisible;

  int blueGoalAngle;
  int yellowGoalAngle;
  int lineAngle;

  uint16_t blueGoalWidth;
  uint16_t yellowGoalWidth;

  uint32_t lastUpdate;
};

CameraState cams[4];

// ============================================================================
// COLOR SENSOR STRUCTS
// ============================================================================

enum SurfaceType : uint8_t {
  SURFACE_UNKNOWN = 0,
  SURFACE_GREEN,
  SURFACE_BLACK,
  SURFACE_WHITE
};

Adafruit_TCS34725 tcs[NUM_COLOR_SENSORS] = {
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X)
};

bool sensorOK[NUM_COLOR_SENSORS];
SurfaceType sensorSurface[NUM_COLOR_SENSORS];

// ============================================================================
// IR GLOBALS
// ============================================================================

bool irSensors[NUM_IR_SENSORS];

float ballAngle = -1.0f;
int ballCount = 0;

bool hasBall = false;

// ============================================================================
// TIMERS
// ============================================================================

IntervalTimer irTimer;
IntervalTimer colorTimer;
IntervalTimer gyroTimer;

volatile bool triggerIR = false;
volatile bool triggerColor = false;
volatile bool triggerGyro = false;

// ============================================================================
// KICKER STATE
// ============================================================================

bool kickerActive = false;

uint32_t kickerStartTime = 0;
uint32_t lastKickTime = 0;

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void setMotor(int dr1, int dr2, int pwmPin, int speed) {

  speed = constrain(speed, -255, 255);

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

// ============================================================================
// MECANUM DRIVE
// ============================================================================

void mecanumDrive(float vx, float vy, float omega) {

  float vLF = vy + vx + omega;
  float vRF = vy - vx - omega;
  float vLR = vy - vx + omega;
  float vRR = vy + vx - omega;

  float mx = max(
    max(fabsf(vLF), fabsf(vRF)),
    max(fabsf(vLR), max(fabsf(vRR), 1.0f))
  );

  setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, (int)(vLF / mx * 255));
  setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, (int)(vRF / mx * 255));
  setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, (int)(vLR / mx * 255));
  setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, (int)(vRR / mx * 255));
}

// ============================================================================
// GYRO
// ============================================================================

float normalizeAngle(float a) {

  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;

  return a;
}

float headingError() {
  return normalizeAngle(gyroHeading - targetHeading);
}

void doGyroRead() {

  if (!gyroOK) return;

  sensors_event_t e;

  bno.getEvent(&e);

  gyroHeading = e.orientation.x;
}

// ============================================================================
// IR SYSTEM
// ============================================================================

inline void setMuxChannel(uint8_t ch) {

  digitalWriteFast(MUX_S0, (ch >> 0) & 1);
  digitalWriteFast(MUX_S1, (ch >> 1) & 1);
  digitalWriteFast(MUX_S2, (ch >> 2) & 1);
  digitalWriteFast(MUX_S3, (ch >> 3) & 1);
}

inline bool waitForBurst(uint8_t pin, uint32_t timeoutUs) {

  uint32_t deadline = micros() + timeoutUs;

  while ((int32_t)(micros() - deadline) < 0) {

    if (!digitalReadFast(pin)) return true;
  }

  return false;
}

void doIRScan() {

  bool results[NUM_IR_SENSORS] = {};

  for (uint8_t ch = 0; ch < NUM_MUX_CH; ch++) {

    setMuxChannel(ch);

    delayMicroseconds(2);

    results[ch] = waitForBurst(MUX_SIG, 200);
  }

  memcpy(irSensors, results, sizeof(results));
}

void processIRData() {

  float sinSum = 0;
  float cosSum = 0;

  ballCount = 0;

  for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {

    if (irSensors[i]) {

      float rad = IR_SENSOR_ANGLES[i] * DEG_TO_RAD;

      sinSum += sinf(rad);
      cosSum += cosf(rad);

      ballCount++;
    }
  }

  if (ballCount > 0) {

    ballAngle = atan2f(sinSum, cosSum) * RAD_TO_DEG;

    if (ballAngle < 0) ballAngle += 360.0f;

  } else {

    ballAngle = -1.0f;
  }

  // Ball is centered in dribbler zone

  hasBall =
    irSensors[0] &&
    irSensors[1] &&
    ballCount >= 6;
}

// ============================================================================
// CAMERA PARSER
// ============================================================================

bool validateChecksum(uint8_t* data, int len, uint8_t checksum) {

  uint8_t x = 0;

  for (int i = 0; i < len; i++) x ^= data[i];

  return x == checksum;
}

void readCamera(HardwareSerial& port, int index) {

  while (port.available() >= sizeof(CamPacket)) {

    if (port.read() != UART_START_BYTE) continue;

    CamPacket pkt;

    ((uint8_t*)&pkt)[0] = UART_START_BYTE;

    port.readBytes(
      ((uint8_t*)&pkt) + 1,
      sizeof(CamPacket) - 1
    );

    cams[index].connected = true;

    cams[index].blueGoalVisible =
      pkt.visible_flags & VIS_BLUE_GOAL;

    cams[index].yellowGoalVisible =
      pkt.visible_flags & VIS_YELLOW_GOAL;

    cams[index].lineVisible =
      pkt.visible_flags & VIS_WHITE_LINE;

    cams[index].blueGoalAngle =
      pkt.blue_goal_angle / 10;

    cams[index].yellowGoalAngle =
      pkt.yellow_goal_angle / 10;

    cams[index].lineAngle =
      pkt.line_angle / 10;

    cams[index].blueGoalWidth =
      pkt.blue_goal_width_px;

    cams[index].yellowGoalWidth =
      pkt.yellow_goal_width_px;

    cams[index].lastUpdate = millis();
  }
}

void processCameras() {

  readCamera(CAM_FRONT, 0);
  readCamera(CAM_RIGHT, 1);
  readCamera(CAM_BACK, 2);
  readCamera(CAM_LEFT, 3);
}

// ============================================================================
// DRIBBLER + KICKER
// ============================================================================

void executeDribbler(bool enable) {

  analogWrite(
    DRIBBLER_PIN,
    enable ? DRIBBLER_SPEED : 0
  );
}

void executeKicker() {

  uint32_t now = millis();

  if (kickerActive) {

    if ((now - kickerStartTime) >= KICKER_PULSE_MS) {

      digitalWrite(KICKER_PIN, LOW);

      kickerActive = false;

      lastKickTime = now;
    }

    return;
  }

  if ((now - lastKickTime) < KICKER_COOLDOWN_MS)
    return;

  digitalWrite(KICKER_PIN, HIGH);

  kickerActive = true;
  kickerStartTime = now;
}

// ============================================================================
// WHITE LINE ESCAPE
// ============================================================================

float getBoundaryEscapeAngle() {

  float sinSum = 0;
  float cosSum = 0;

  int count = 0;

  for (int i = 0; i < NUM_COLOR_SENSORS; i++) {

    if (sensorSurface[i] == SURFACE_WHITE) {

      float rad =
        (COLOR_SENSOR_ANGLES[i] + 180.0f) *
        DEG_TO_RAD;

      sinSum += sinf(rad);
      cosSum += cosf(rad);

      count++;
    }
  }

  if (count == 0) return -1.0f;

  float angle =
    atan2f(sinSum, cosSum) *
    RAD_TO_DEG;

  if (angle < 0) angle += 360.0f;

  return angle;
}

// ============================================================================
// GOAL SEARCH
// ============================================================================

bool findBlueGoal(int& angle, int& width) {

  for (int i = 0; i < 4; i++) {

    if (!cams[i].blueGoalVisible) continue;

    if ((millis() - cams[i].lastUpdate) > 300)
      continue;

    angle = cams[i].blueGoalAngle;
    width = cams[i].blueGoalWidth;

    return true;
  }

  return false;
}

// ============================================================================
// MAIN STRATEGY
// ============================================================================

void runStrategy() {

  // ------------------------------------------------
  // WHITE LINE ESCAPE
  // ------------------------------------------------

  float escape = getBoundaryEscapeAngle();

  if (escape >= 0.0f) {

    executeDribbler(false);

    float rad = escape * DEG_TO_RAD;

    mecanumDrive(
      sinf(rad) * 0.9f,
      cosf(rad) * 0.9f,
      -headingError() / 180.0f
    );

    return;
  }

  // ------------------------------------------------
  // BALL POSSESSION
  // ------------------------------------------------

  if (hasBall) {

    executeDribbler(true);

    int goalAngle;
    int goalWidth;

    bool goalFound =
      findBlueGoal(goalAngle, goalWidth);

    // Goal centered enough -> shoot

    if (goalFound &&
        abs(goalAngle) < 12 &&
        goalWidth > 70) {

      mecanumDrive(0,0,0);

      executeKicker();

      return;
    }

    // Rotate toward goal

    if (goalFound) {

      float omega =
        constrain(goalAngle / 60.0f,
                  -0.8f,
                   0.8f);

      mecanumDrive(
        0,
        0.18f,
        omega
      );

      return;
    }

    // Search for goal while holding ball

    mecanumDrive(0, 0.2f, 0.35f);

    return;
  }

  // ------------------------------------------------
  // SEARCH BALL USING IR ONLY
  // ------------------------------------------------

  executeDribbler(false);

  if (ballAngle >= 0.0f) {

    float rad = ballAngle * DEG_TO_RAD;

    float speed = 0.8f;

    if (ballCount >= 12)
      speed = 0.45f;

    float vx = sinf(rad) * speed;
    float vy = cosf(rad) * speed;

    // Helps center ball into dribbler mouth

    vx += -sinf(rad) * 0.22f;

    float omega =
      -headingError() / 90.0f;

    mecanumDrive(vx, vy, omega);

    return;
  }

  // ------------------------------------------------
  // NO BALL FOUND
  // ------------------------------------------------

  mecanumDrive(0,0,0.35f);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {

  Serial.begin(115200);

  CAM_FRONT.begin(UART_BAUD);
  CAM_RIGHT.begin(UART_BAUD);
  CAM_BACK.begin(UART_BAUD);
  CAM_LEFT.begin(UART_BAUD);

  pinMode(DRIBBLER_PIN, OUTPUT);
  pinMode(KICKER_PIN, OUTPUT);

  analogWrite(DRIBBLER_PIN, 0);
  digitalWrite(KICKER_PIN, LOW);

  // Motor pins

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

  // Gyro

  Wire1.begin();

  if (bno.begin()) {

    gyroOK = true;

    bno.setExtCrystalUse(true);

    sensors_event_t evt;

    bno.getEvent(&evt);

    targetHeading = evt.orientation.x;
  }

  // Timers

  irTimer.begin([]{
    triggerIR = true;
  }, 50000);

  gyroTimer.begin([]{
    triggerGyro = true;
  }, 10000);
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {

  if (triggerGyro) {

    triggerGyro = false;

    doGyroRead();
  }

  if (triggerIR) {

    triggerIR = false;

    doIRScan();

    processIRData();
  }

  processCameras();

  if (kickerActive)
    executeKicker();

  runStrategy();
}