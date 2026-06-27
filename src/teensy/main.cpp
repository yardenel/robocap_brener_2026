// robocap_teensy_main_INTEGRATED_BALL_TRACKING_PLUS_GOOD_COLOUR
// Base: ball-tracking code. Integrated: robust TCA9548A/TCS34725 colour sensor block.
// ============================================================================
//  robocap_teensy_main.ino  ―  RoboCap 2026  Teensy 4.1 MAIN firmware
//  Team Brenner  /  RoboCupJunior Soccer  (identical firmware on both robots)
// ----------------------------------------------------------------------------
//  SOURCES OF TRUTH used to build this file (no guessed pins):
//    - Robocap_upper_PCB_electronics_drawing.pdf  (the wiring authority)
//    - Robocap_teensy_and_buttom_pcb_testing.ino  (motor pins, verified match)
//    - robot_protocol.h (v3.0)                     (UART/ESP-NOW/TEST protocol)
//
//  HARDWARE (per robot):
//    Teensy 4.1 main controller
//    4x L298N -> 4 Omni wheels / X-drive
//    Kicker solenoid (MOSFET) + Dribbler motor (PWM)
//    20x TSOP34838 IR (16 via 74HC4067 mux + 4 direct; U2 = ball-in-pocket)
//    BNO055 IMU over UART Serial5 (TX5=pin20, RX5=pin21)
//    4x TCS34725 colour sensors behind a TCA9548A I2C mux (channels 0..3)
//    4x XIAO ESP32-S3 cameras, one per Serial1..4 (demuxed by ESP_ID byte)
//    RCJ Communication module Run/Stop on pin 9 (3.3V=GO, 0V=STOP, continuous)
//
//  *** THINGS TO VERIFY / TUNE are marked  // TODO(VERIFY)  and  // TODO(TUNE) ***
//  Nothing in this file was invented as fact: every pin is from the schematic;
//  every numeric tuning value is a clearly-labelled placeholder you must test.
//
//  Required libraries (Library Manager):
//    Adafruit BNO055, Adafruit Unified Sensor, Adafruit TCS34725
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_TCS34725.h>
#include "robot_protocol.h" // keep IDENTICAL across ESP + Teensy

/*
POWER_ON - init
    → POST
POST - Power-On Self-Test;
        checks:
        - sensors react
        - get 4 esp ids
        - power motors on 30%: to check electrical continuity – if idle current exceeds the normal range
        - solenoid reacts to small pulse (5ms)
        - gyro returns stable angle
        - IRs return 0
    everything ok: → READY
    problem detected: → FAULT
READY - stand-by;
    - motors 0%
    - ESP-NOW working and searching for other ESP
    - SoftAP working
    - PIN 9 (game module) monitored in high frequency (interupt driven); ready for GAME
    PIN 9 = HIGH → GAME
    "Enter Test" from Web UI → TEST
    * Error detected → FAULT
GAME - active strategy
    - SoftAP off
    - ESP-NOW active for partner comm
    - PIN 9 monitored; ready for STOP
    - strategy FSM ATTACKER/ATTACK SUPPORT
    PIN 9 = LOW → PAUSED
    * penalty (out of lines / ball holding) → TBD
* PAUSED - in between games/breaks/penalties
    - motors 0%
    - kicker off
    - dribbler 0%
    - internal strategy state, ballAngle are saved

TEST - test state
    - SoftAP & Web UI active
    - manual control over motors, dribbler, kicker, color sensors
    - IR & gyro feedback
    PIN 9 = HIGH → GAME
FAULT - loss of sensor, critical error, etc..
    manual restart required
*/
enum SysState : uint8_t
{
  S_POWER_ON,
  S_POST,
  S_READY,
  S_TEST,
  S_GAME,
  S_PAUSED,
  S_FAULT
};

// ============================================================================
//  1.  PIN MAP  (GPIO numbers used in code; verified against the schematic)
// ============================================================================
// ---- Motors: ENG1=RF  ENG2=RR  ENG3=LR  ENG4=LF  (SP=PWM/ENx, DRx=IN1/IN2) --
#define ENG1_SP 23
#define ENG1_DR1 13 // NOTE: also the on-board LED; it will toggle w/ dir
#define ENG1_DR2 41
#define ENG2_SP 22
#define ENG2_DR1 40
#define ENG2_DR2 39
#define ENG3_SP 37
#define ENG3_DR1 38
#define ENG3_DR2 35
#define ENG4_SP 36
#define ENG4_DR1 34
#define ENG4_DR2 33

// ---- Actuators -------------------------------------------------------------
#define PIN_KICKER 2    // -> R1 100ohm -> MOSFET gate
#define PIN_DRIBBLER 11 // PWM speed control
#define PIN_SW2 12      // physical "Operate Engines" switch (active level TODO(VERIFY))

// ---- RCJ Run/Stop ----------------------------------------------------------
#define PIN_RCJ_RUN 9 // INPUT_PULLDOWN + ISR; HIGH=GO, LOW=STOP

// ---- IR ring: 74HC4067 mux (16 ch) select + common, plus 4 direct ----------
#define IR_MUX_S0 31
#define IR_MUX_S1 30
#define IR_MUX_S2 29
#define IR_MUX_S3 28
#define IR_MUX_SIG 32 // common output of the 16 muxed TSOPs (digital)
#define IR_POCKET_MUX_CH 1   // U2 = mux channel 1, BALL-IN-POCKET
#define IR_DIR_17 24  // U17 direct
#define IR_DIR_18 25  // U18 direct
#define IR_DIR_19 26  // U19 direct
#define IR_DIR_20 27  // U20 direct perimeter sensor (not pocket anymore)

// ---- BNO055 (I2C) ----------------------------------------------------------
#define PIN_GYRO_RST 3 // Gyro_RST: BNO055 RESET (active-low) -> drive HIGH to RUN, LOW = held in reset
#define BNO055_ADDR 0x28
// (BNO055 SDA/SCL share the main I2C bus: SDA=18, SCL=19)

// ---- TCA9548A colour-sensor I2C mux ---------------------------------------
#define TCA_ADDR 0x70
#define TCA_A0 4 // mux address-select lines (driven to fix address)
#define TCA_A1 5
#define TCA_A2 6
#define TCA_RST 10   // mux RESET (active LOW)
#define NUM_COLOUR 4 // sensors on mux channels 0..3
const uint8_t COLOUR_CH[NUM_COLOUR] = {0, 1, 2, 3};

// ---- ESP camera UARTs (demuxed in software by the ESP_ID byte) -------------
//  Serial1=U21(0/1) Serial2=U22(7/8) Serial3=U23(15/14) Serial4=U24(16/17)
HardwareSerial *const ESP_PORT[4] = {&Serial1, &Serial2, &Serial3, &Serial4};
// The FORWARD esp (ESP_ID_FRONT) also carries TEST / ESP-NOW traffic. We don't
// assume which physical port it is; we learn it from the first packet whose
// ESP_ID == ESP_ID_FRONT and remember that port for command/telemetry TX.
int forwardPortIdx = -1; // -1 until discovered

// ============================================================================
//  2.  TUNABLES  (placeholders — measure & adjust on the bench / field)
// ============================================================================
// ---- TEMP BENCH MODE ----
// Set true to boot directly into GAME after POST, bypassing READY/RCJ start.
// FOR BENCH ONLY: set false again before normal RCJ use / competition.
static constexpr bool AUTO_GAME_ON_BOOT = true;
static constexpr bool AUTO_GAME_IGNORE_POST_FAIL = true;

static constexpr bool POST_MOTOR_TICK = false;

static constexpr bool REQUIRE_SW2_ENABLE = false;
static constexpr int SW2_ENABLE_LEVEL = HIGH;

// ---- Drive ----
const float DRIVE_MAX = 1.0f;        // global speed cap (0..1)
const float HEADING_KP = 0.012f;     // TODO(TUNE) compass-hold P gain (per deg)
const float HEADING_MAX_CORR = 0.6f; // TODO(TUNE) max rotation from heading PID

// ---- PWM carrier frequencies (Hz) — DRIVER-TYPE DEPENDENT, keep separate ----
//  The 4 wheels are L298N (slow Darlington BJT output stage): the PWM must stay
//  in the low-kHz band or the bridge runs hot / loses torque. 1 kHz is bench-
//  proven on this build. The dribbler is a MOSFET driver: 20 kHz is above the
//  audible range and may go higher if wanted. They sit on different timer
//  peripherals (motor SP pins = FlexPWM; dribbler pin 11 = QuadTimer1), so
//  these two frequencies do NOT collide.  (Was a single 20000 for both before.)
const uint32_t MOTOR_PWM_HZ = 1000;     // L298N wheels  — low kHz, bench-proven
const uint32_t DRIBBLER_PWM_HZ = 20000; // MOSFET dribbler — raise to 25000..32000 to taste

// ---- Motor polarity --------------------------------------------------------
// Use this to fix a motor that is physically wired opposite to the software
// direction. ENG1 / motor 1 is currently reversed.
const int MOTOR_INV[5] = {
  0,
  -1, // motor 1 / ENG1 / RF reversed
   1, // motor 2 / ENG2 / RR normal
   1, // motor 3 / ENG3 / LR normal
   1  // motor 4 / ENG4 / LF normal
};

// ---- Kicker ----  KICK:pct -> duty; pulse length differs per kick type ----
const int KICK_DIRECT_MS = 60;   // TODO(TUNE) direct kick pulse (ms)
const int KICK_BACKSPIN_MS = 18; // TODO(TUNE) light pulse: back-spin release
const int KICK_REARM_MS = 400;   // TODO(TUNE) min time between kicks

// ---- Dribbler ----
const uint8_t DRIBBLER_RUN_PWM = 220; // TODO(TUNE) normal capture speed (0..255)

// ---- IR ring ----
const int IR_SAMPLES_PER_CH = 24; // TODO(TUNE) digital samples per sensor
const int IR_SETTLE_US = 6;       // TODO(TUNE) mux settle after select
const int IR_PRESENT_MIN = 3;     // TODO(TUNE) min strength to count a sensor
const int POCKET_MIN_HITS = 6;    // tuned: U2 hits => ball in pocket/dribbler

// ---- Colour / white-line ----
const uint16_t LINE_CLEAR_MIN = 1500; // TODO(TUNE) clear-channel threshold for white
// (white = bright AND roughly equal R/G/B; tune with CAL on the real field)

// ---- Strategy thresholds (relative IR strength / camera radius) ----
const int BALL_CLOSE_STR = 16; // TODO(TUNE) "ball is near" strength
// (alignment windows now live in section 14: KICK_FACE_DEG / BACKSPIN_DEG / ATTACK_ANGLE_DEG)

// ============================================================================
//  3.  IR RING TABLE   *** VERIFY THIS AGAINST THE PHYSICAL BUILD ***
// ----------------------------------------------------------------------------
//  18/19 perimeter sensors + 1 pocket sensor (U2).
//  ASSUMPTIONS (U1=front=0deg, U2=pocket, 16 muxed + 4 direct):
//    * U1..U16  -> mux channels 0..15
//    * U17..U19 -> direct pins 24,25,26
//    * bearings spread evenly over 360deg, U1=0deg, increasing CLOCKWISE
//      (each TSOP has ~20deg FOV; 19 of them overlap to cover the full circle)
//  If your real layout differs, edit ONLY the .bearing / .ch / .pin below.
// ============================================================================
enum IrSrc : uint8_t
{
  IR_MUX = 0,
  IR_DIRECT = 1
};
struct IrSensor
{
  uint8_t src;
  uint8_t chOrPin;
  float bearing;
};

const int NUM_IR = 19;    // perimeter sensors (pocket handled separately)
IrSensor IR_RING[NUM_IR]; // filled in setup() so bearings are computed

uint8_t irStrength[NUM_IR]; // 0..IR_SAMPLES_PER_CH
bool ballVisible = false;
float ballAngle = -1; // 0..359 robot-relative, -1 if none
int ballStrength = 0; // peak/vector magnitude proxy
bool ballInPocket = false;

void buildIrRing()
{
  int idx = 0;

  // U1..U16 are mux channels 0..15, but U2/pocket is NOT part of the perimeter
  for (int ch = 0; ch < 16; ch++)
  {
    if (ch == IR_POCKET_MUX_CH) continue; // skip U2 pocket

    IR_RING[idx].src = IR_MUX;
    IR_RING[idx].chOrPin = ch;
    IR_RING[idx].bearing = idx * (360.0f / NUM_IR);
    idx++;
  }

  // U17..U20 direct perimeter sensors
  const uint8_t dpin[4] = {IR_DIR_17, IR_DIR_18, IR_DIR_19, IR_DIR_20};
  for (int d = 0; d < 4; d++)
  {
    IR_RING[idx].src = IR_DIRECT;
    IR_RING[idx].chOrPin = dpin[d];
    IR_RING[idx].bearing = idx * (360.0f / NUM_IR);
    idx++;
  }

  if (idx != NUM_IR)
  {
    Serial.printf("[IR] ERROR: built %d sensors, expected %d\n", idx, NUM_IR);
  }
}

// ============================================================================
//  4.  GLOBAL STATE
// ============================================================================
volatile SysState sysState = S_POWER_ON;
SysState gamePrevState = S_GAME; // remembered across PAUSE (rule 2.12)

// ---- RCJ run/stop (set in ISR) ----
volatile bool runSignal = false; // true = GO
volatile bool runEdge = false;   // an edge happened, loop() handles it

// ---- TEST-mode bench controls ---------------------------------------------
// Keep these TRUE while bringing the robot up on the bench. For competition,
// set them FALSE so POST faults remain hard faults and RCJ GO can leave TEST.
static constexpr bool TEST_ALLOW_FROM_FAULT = true;  // lets TEST:ON work after POST failure
static constexpr bool TEST_IGNORE_RCJ_GO = true;     // keeps TEST sticky when pin 9 is noisy/absent
static constexpr bool TEST_DEADMAN_ENABLE = true;    // stops manual drive if UI commands stop
static constexpr uint32_t TEST_DEADMAN_MS = 450;
bool testEnteredFromFault = false;
uint32_t lastTestPhysicalCmdMs = 0;

// ---- Heading / compass ----
// [v5] BNO055 now driven over UART (Serial5), not I2C — see compassInit().
// Adafruit_BNO055 bno(55, BNO055_ADDR, &Wire);   // (was I2C; unused)
bool bnoOK = false;
float headingRaw = 0;            // 0..359 from BNO055
float headingZero = 0;           // offset set by Goal-Lock
float heading = 0;               // headingRaw - headingZero, wrapped
float goalBearing = -1;          // opponent goal bearing (from camera), -1 unknown
uint8_t lockedGoalColour = 0xFF; // OBJ side colour we attack (set by GOAL_LOCK)

// ---- Vision (per camera, latest) ----
struct CamObj
{
  int8_t type;
  int16_t angleRobot;
  uint8_t dist;
  uint32_t t;
};
uint16_t camPkt[5] = {0};  // [v5] per-camera packet counter (ESP_ID 1..4), incl no-detect
uint16_t rawByte[4] = {0}; // [v5] raw bytes per PORT (Serial1..4) — any traffic at all?
// [v5] per-camera latest detection for the app Vision tab (robot-relative angle, -1 via timeout)
struct CamVis
{
  int16_t goalR, goalC, white, black;
  uint8_t goalCol;
  uint8_t goalDist;
  uint32_t tG, tW, tK;
};
CamVis camVis[4] = {};                     // goalCol: 0 none, 1 yellow, 2 blue
const uint32_t VIS_TIMEOUT_MS = 600;       // drop a reading to -1 if not refreshed within this
CamObj lastGoal = {-1, 0, 0, 0};           // most recent goal sighting (any cam)
const int MOUNT[5] = {0, 0, 90, 180, 270}; // index by ESP_ID (0 unused)

// ---- Line ----
bool lineDetected = false;
uint8_t lineMask = 0;

// TODO: לוודא פיזית את הסדר!
// כאן אני מניחה:
// 0 = קדמי, 1 = ימין, 2 = אחורי, 3 = שמאל
const float COLOUR_BEARING[NUM_COLOUR] = {45, 135, 225, 315};

// ---- Roles / partner (auction) ----
RobotRole myRole = ROLE_UNKNOWN;
bool partnerOnline = false;
bool iAmHighMac = false;       // set from EVT_PARTNER_FOUND payload: 1 => this robot holds the higher MAC
int partnerBallStr = -1;       // partner's ball strength (from relay)
uint32_t lastPartnerMsgMs = 0; // set when partner data arrives (ESP-NOW relay)
int partnerScore = 0;          // partner auction score (proxy until ESP wired)

// ---- Kicker timing ----
uint32_t lastKickMs = 0;

// ============================================================================
//  5.  MOTORS  (OMNI / X-drive kinematics + pins from testing.ino & schematic)
// ============================================================================
void setMotor(int dir1, int dir2, int pwmPin, int speed)
{
  if (speed > 0)
  {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(pwmPin, constrain(speed, 0, 255));
  }
  else if (speed < 0)
  {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(pwmPin, constrain(-speed, 0, 255));
  }
  else
  {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void motorKill()
{
  setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, 0);
  setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, 0);
  setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, 0);
  setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, 0);
}

// vx=strafe(+right) vy=forward(+fwd) omega=rotation(+CCW), all -1..+1
void omniDrive(float vx, float vy, float omega)
{
  float vRF = vy - vx - omega; // ENG1
  float vRR = vy + vx - omega; // ENG2
  float vLR = vy - vx + omega; // ENG3
  float vLF = vy + vx + omega; // ENG4
  float m = max(max(fabs(vRF), fabs(vRR)), max(fabs(vLR), fabs(vLF)));
  if (m > 1.0f)
  {
    vRF /= m;
    vRR /= m;
    vLR /= m;
    vLF /= m;
  }
  float g = DRIVE_MAX;
  setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, (int)(vRF * 255 * g * MOTOR_INV[1]));
  setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, (int)(vRR * 255 * g * MOTOR_INV[2]));
  setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, (int)(vLR * 255 * g * MOTOR_INV[3]));
  setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, (int)(vLF * 255 * g * MOTOR_INV[4]));
}

// Drive toward a robot-relative bearing while holding compass heading.
void driveToward(float bearingDeg, float speed)
{
  float r = bearingDeg * DEG_TO_RAD;
  float vx = sinf(r) * speed; // +bearing(CW) -> +right
  float vy = cosf(r) * speed;
  // hold heading (keep facing where we started / our goal reference)
  float err = heading; // want heading -> 0 (forward = goal ref)
  if (err > 180)
    err -= 360;
  float omega = constrain(-err * HEADING_KP, -HEADING_MAX_CORR, HEADING_MAX_CORR);
  omniDrive(vx, vy, omega);
}

void motorsInit()
{
  const int sp[4] = {ENG1_SP, ENG2_SP, ENG3_SP, ENG4_SP};
  const int d1[4] = {ENG1_DR1, ENG2_DR1, ENG3_DR1, ENG4_DR1};
  const int d2[4] = {ENG1_DR2, ENG2_DR2, ENG3_DR2, ENG4_DR2};
  for (int i = 0; i < 4; i++)
  {
    pinMode(sp[i], OUTPUT);
    pinMode(d1[i], OUTPUT);
    pinMode(d2[i], OUTPUT);
    analogWriteFrequency(sp[i], MOTOR_PWM_HZ);
  } // 1 kHz for L298N
  motorKill();
}

// ============================================================================
//  6.  KICKER + DRIBBLER
// ============================================================================
void dribblerSet(uint8_t pwm) { analogWrite(PIN_DRIBBLER, pwm); }

// kind: false = direct kick, true = back-spin release (light pulse)
void fireKicker(int pct, bool backspin)
{
  if (millis() - lastKickMs < (uint32_t)KICK_REARM_MS)
    return;
  int duty = constrain(map(constrain(pct, 0, 100), 0, 100, 0, 255), 0, 255);
  int dur = backspin ? KICK_BACKSPIN_MS : KICK_DIRECT_MS;
  analogWrite(PIN_KICKER, duty);
  delay(dur); // short blocking pulse; acceptable for a kick
  analogWrite(PIN_KICKER, 0);
  lastKickMs = millis();
}

void actuatorsInit()
{
  pinMode(PIN_KICKER, OUTPUT);
  analogWrite(PIN_KICKER, 0);
  pinMode(PIN_DRIBBLER, OUTPUT);
  analogWriteFrequency(PIN_DRIBBLER, DRIBBLER_PWM_HZ);
  dribblerSet(0);
  pinMode(PIN_SW2, INPUT_PULLUP);
}

void markTestPhysicalCmd()
{
  lastTestPhysicalCmdMs = millis();
}

void testDeadmanUpdate()
{
  if (!TEST_DEADMAN_ENABLE || sysState != S_TEST)
    return;
  if (lastTestPhysicalCmdMs && millis() - lastTestPhysicalCmdMs > TEST_DEADMAN_MS)
  {
    motorKill();
    dribblerSet(0);
    analogWrite(PIN_KICKER, 0);
    lastTestPhysicalCmdMs = 0;
  }
}

// ============================================================================
//  7.  IR RING  (digital TSOP reads through the 74HC4067 + direct pins)
// ----------------------------------------------------------------------------
//  TSOP34838 output is active-LOW while it sees the 38kHz ball burst, so we
//  count LOW samples per sensor over a small window => "strength".
// ============================================================================
void muxSelect(uint8_t ch)
{
  digitalWriteFast(IR_MUX_S0, ch & 0x01);
  digitalWriteFast(IR_MUX_S1, (ch >> 1) & 0x01);
  digitalWriteFast(IR_MUX_S2, (ch >> 2) & 0x01);
  digitalWriteFast(IR_MUX_S3, (ch >> 3) & 0x01);
}

uint8_t readSensorStrength(const IrSensor &s)
{
  uint8_t hits = 0;
  if (s.src == IR_MUX)
  {
    muxSelect(s.chOrPin);
    delayMicroseconds(IR_SETTLE_US);
  }
  for (int k = 0; k < IR_SAMPLES_PER_CH; k++)
  {
    int v = (s.src == IR_MUX) ? digitalReadFast(IR_MUX_SIG) : digitalRead(s.chOrPin);
    if (v == LOW)
      hits++; // LOW = burst seen
  }
  return hits;
}

void irScan()
{
  // perimeter: weighted vector sum -> ball angle + strength
  // U2 is used only as the ball-in-pocket sensor, so it is skipped here
  // and does not distort ballAngle / ballStrength.
  float sx = 0, sy = 0;
  int peak = 0;
  for (int i = 0; i < NUM_IR; i++)
  {

    uint8_t st = readSensorStrength(IR_RING[i]);
    irStrength[i] = st;
    if (st > peak)
      peak = st;
    if (st >= IR_PRESENT_MIN)
    {
      float r = IR_RING[i].bearing * DEG_TO_RAD;
      sx += st * sinf(r); // +bearing(CW) on +x(right)
      sy += st * cosf(r);
    }
  }
  if (peak >= IR_PRESENT_MIN)
  {
    float a = atan2f(sx, sy) * RAD_TO_DEG; // note atan2(x,y): 0=fwd, +CW
    if (a < 0)
      a += 360;
    ballAngle = a;
    ballStrength = peak;
    ballVisible = true;
  }
  else
  {
    ballAngle = -1;
    ballStrength = 0;
    ballVisible = false;
  }

  // pocket sensor = U2 on mux channel 1, active-LOW
  muxSelect(IR_POCKET_MUX_CH);
  delayMicroseconds(IR_SETTLE_US);

  uint8_t ph = 0;
  for (int k = 0; k < IR_SAMPLES_PER_CH; k++)
  {
    if (digitalReadFast(IR_MUX_SIG) == LOW)
      ph++;
  }
  ballInPocket = (ph >= POCKET_MIN_HITS);
}

void irInit()
{
  pinMode(IR_MUX_S0, OUTPUT);
  pinMode(IR_MUX_S1, OUTPUT);
  pinMode(IR_MUX_S2, OUTPUT);
  pinMode(IR_MUX_S3, OUTPUT);
  pinMode(IR_MUX_SIG, INPUT);
  pinMode(IR_DIR_17, INPUT);
  pinMode(IR_DIR_18, INPUT);
  pinMode(IR_DIR_19, INPUT);
  pinMode(IR_DIR_20, INPUT);
}

// ============================================================================
//  8.  BNO055 heading  —  UART on Serial5 (TX5=pin20 -> Gyro_RX, RX5=pin21 <- Gyro_TX)
//  This board wires the CJMCU-055 for UART, not I2C. Module mode pads: S1->"+"
//  (PS1=1), S0->"-" (PS0=0) select UART. Link: 115200 8N1.
// ============================================================================
#define BNO_UART Serial5
#define BNO_BAUD 115200
#define BNO_REG_CHIP_ID 0x00 // -> 0xA0
#define BNO_REG_PAGE_ID 0x07
#define BNO_REG_EUL_H_LSB 0x1A // heading (yaw) LSB; value is 1/16 degree
#define BNO_REG_OPR_MODE 0x3D
#define BNO_REG_PWR_MODE 0x3E
#define BNO_REG_SYS_TRIG 0x3F
#define BNO_OPR_CONFIG 0x00
#define BNO_OPR_NDOF 0x0C
// [v5] External 32kHz crystal: many cheap CJMCU/clone BNO055 modules do NOT have it.
// Telling the chip to use a missing crystal freezes the fusion clock -> heading stuck
// at 0 while register reads still work. Default 0 = internal clock (safe for clones).
// Set to 1 only if your module definitely has the crystal (e.g. genuine Adafruit).
#define BNO_USE_EXT_CRYSTAL 0
// [v5] bring-up aid: set to 1 to continuously ping CHIP_ID on Serial5 (~5 Hz) and
// dump the response, so you can move a scope/probe around the module pins and
// always see live activity. Set back to 0 for normal play.
#define BNO_PING_DEBUG 0

static void bnoFlush()
{
  while (BNO_UART.available())
    BNO_UART.read();
}
static bool bnoWait(int n, uint32_t ms)
{
  uint32_t t0 = millis();
  while ((int)BNO_UART.available() < n)
  {
    if (millis() - t0 > ms)
      return false;
  }
  return true;
}
// Write len bytes to reg. Returns true on ACK (0xEE 0x01). Retries on bus over-run.
static bool bnoWriteReg(uint8_t reg, const uint8_t *data, uint8_t len)
{
  for (int a = 0; a < 3; a++)
  {
    bnoFlush();
    BNO_UART.write(0xAA);
    BNO_UART.write(0x00);
    BNO_UART.write(reg);
    BNO_UART.write(len);
    for (uint8_t i = 0; i < len; i++)
      BNO_UART.write(data[i]);
    if (!bnoWait(2, 12))
      continue;
    uint8_t r0 = BNO_UART.read(), r1 = BNO_UART.read();
    if (r0 == 0xEE && r1 == 0x01)
      return true;
    if (r0 == 0xEE && r1 == 0x07)
    {
      delay(2);
      continue;
    } // over-run -> retry
    return false;
  }
  return false;
}
static bool bnoWriteByte(uint8_t reg, uint8_t v) { return bnoWriteReg(reg, &v, 1); }
// Read len bytes from reg into buf. Returns true on success.
static bool bnoReadReg(uint8_t reg, uint8_t *buf, uint8_t len)
{
  for (int a = 0; a < 3; a++)
  {
    bnoFlush();
    BNO_UART.write(0xAA);
    BNO_UART.write(0x01);
    BNO_UART.write(reg);
    BNO_UART.write(len);
    if (!bnoWait(2, 12))
      continue;
    uint8_t hdr = BNO_UART.read();
    if (hdr == 0xBB)
    {
      uint8_t n = BNO_UART.read();
      if (n != len || !bnoWait(n, 12))
        continue;
      for (uint8_t i = 0; i < n; i++)
        buf[i] = BNO_UART.read();
      return true;
    }
    uint8_t err = BNO_UART.read(); // hdr==0xEE
    if (err == 0x07)
    {
      delay(2);
      continue;
    } // over-run -> retry
    return false;
  }
  return false;
}

void compassInit()
{
  // [v5] Release the BNO055 from RESET. Gyro_RST = GPIO3, active-low: the chip was
  // being held in reset (RES=0) before, which is why it was totally silent. Pulse it.
  pinMode(PIN_GYRO_RST, OUTPUT);
  digitalWrite(PIN_GYRO_RST, LOW);
  delay(20);                        // assert reset
  digitalWrite(PIN_GYRO_RST, HIGH); // release -> chip runs
  BNO_UART.begin(BNO_BAUD);
  delay(700); // BNO055 boots ~650ms after reset release; retry loop adds margin
  bnoFlush();
  // [v5] DIAG: send a CHIP_ID read and dump whatever comes back on Serial5.
  BNO_UART.write(0xAA);
  BNO_UART.write(0x01);
  BNO_UART.write(0x00);
  BNO_UART.write(0x01);
  Serial.print("[BNO] raw resp on Serial5:");
  {
    int cnt = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 60)
    {
      if (BNO_UART.available())
      {
        Serial.printf(" %02X", BNO_UART.read());
        cnt++;
      }
    }
    if (!cnt)
      Serial.print(" (NOTHING) -> check: S1->\"+\" (UART), crossover Gyro_TX->pin21 & Gyro_RX->pin20, VIN=3.3V");
    Serial.println();
  }
  bnoFlush();
  uint8_t id = 0;
  bnoOK = false;
  for (int i = 0; i < 30 && !bnoOK; i++)
  { // wait for CHIP_ID=0xA0 (cold boot can need >1s)
    if (bnoReadReg(BNO_REG_CHIP_ID, &id, 1) && id == 0xA0)
      bnoOK = true;
    else
      delay(50);
  }
  Serial.printf("[BNO] init: bnoOK=%d (CHIP_ID=0x%02X, expect 0xA0)\n", bnoOK, id);
  if (!bnoOK)
    return;
  bnoWriteByte(BNO_REG_PAGE_ID, 0x00);
  bnoWriteByte(BNO_REG_OPR_MODE, BNO_OPR_CONFIG);
  delay(25);
  bnoWriteByte(BNO_REG_PWR_MODE, 0x00); // normal power
#if BNO_USE_EXT_CRYSTAL
  bnoWriteByte(BNO_REG_SYS_TRIG, 0x80);
  delay(10); // external 32kHz crystal
#endif
  uint8_t m = 0xFF;
  for (int i = 0; i < 5; i++)
  { // set NDOF and verify it took
    bnoWriteByte(BNO_REG_OPR_MODE, BNO_OPR_NDOF);
    delay(30);
    if (bnoReadReg(BNO_REG_OPR_MODE, &m, 1) && m == BNO_OPR_NDOF)
      break;
  }
  Serial.printf("[BNO] OPR_MODE readback=0x%02X (expect 0x0C=NDOF), extXtal=%d\n", m, BNO_USE_EXT_CRYSTAL);
}

void compassUpdate()
{
  if (!bnoOK)
    return;
  static uint32_t last = 0;
  if (millis() - last < 10)
    return; // ~100 Hz (matches BNO fusion rate)
  last = millis();
  uint8_t b[2];
  if (!bnoReadReg(BNO_REG_EUL_H_LSB, b, 2))
    return; // keep last heading on a glitch
  int16_t raw = (int16_t)((uint16_t)b[1] << 8 | b[0]);
  headingRaw = raw / 16.0f; // 1/16 deg -> deg
  float h = headingRaw - headingZero;
  while (h < 0)
    h += 360;
  while (h >= 360)
    h -= 360;
  heading = h;
}

#if BNO_PING_DEBUG
// Continuously ping CHIP_ID and dump the raw response — for scope/probe bring-up.
void bnoPingDebug()
{
  static uint32_t t = 0;
  if (millis() - t < 200)
    return;
  t = millis();
  bnoFlush();
  BNO_UART.write(0xAA);
  BNO_UART.write(0x01);
  BNO_UART.write(0x00);
  BNO_UART.write(0x01);
  Serial.print("[BNO-PING] resp:");
  int cnt = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 40)
  {
    if (BNO_UART.available())
    {
      Serial.printf(" %02X", BNO_UART.read());
      cnt++;
    }
  }
  Serial.println(cnt ? "" : " (none)");
}
#endif

void goalLock() { headingZero = headingRaw; } // current facing becomes 0deg

// ============================================================================
//  9.  COLOUR SENSORS (TCA9548A) + white-line detection
// ============================================================================
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
bool colourGood[NUM_COLOUR] = {false}; // [v5] per-channel: a TCS34725 ACKed at init

void tcaSelect(uint8_t ch)
{
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}
void tcaDeselect()
{ // [v5] disconnect ALL downstream channels
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}
void tcaReset()
{ // [v5] hardware reset: channel reg -> 0 (all off).
  digitalWrite(TCA_RST, LOW);
  delayMicroseconds(20); // Recovers the bus when a dead sensor on a
  digitalWrite(TCA_RST, HIGH);
  delayMicroseconds(50); // selected channel clamps SDA low.
}

void colourInit()
{
  pinMode(TCA_A0, OUTPUT);
  pinMode(TCA_A1, OUTPUT);
  pinMode(TCA_A2, OUTPUT);
  digitalWrite(TCA_A0, LOW);
  digitalWrite(TCA_A1, LOW);
  digitalWrite(TCA_A2, LOW); // addr 0x70
  pinMode(TCA_RST, OUTPUT);
  digitalWrite(TCA_RST, HIGH);
  tcaReset(); // clean start: all channels off
  // Probe each channel. A dead sensor (shorted SDA) clamps the whole bus while its
  // channel is selected, so RESET after EACH channel to release it before the next.
  for (int i = 0; i < NUM_COLOUR; i++)
  {
    tcaSelect(COLOUR_CH[i]);
    Wire.beginTransmission(0x29);
    colourGood[i] = (Wire.endTransmission() == 0); // does a TCS34725 ACK on this channel?
    if (colourGood[i])
      tcs.begin();
    tcaReset(); // deselect + recover from any SDA clamp
  }
  Serial.print("[COLOUR] good channels:");
  bool any = false;
  for (int i = 0; i < NUM_COLOUR; i++)
    if (colourGood[i])
    {
      Serial.printf(" ch%d", COLOUR_CH[i]);
      any = true;
    }
  Serial.println(any ? "" : " NONE");
}

// [v5] I2C bus scanner — call once at boot. Prints every address that ACKs on
// the main bus (expect 0x28 BNO055, 0x70 TCA9548A), then selects each mux
// channel and checks for a TCS34725 @0x29. Pure diagnostic; safe to leave in.
void i2cScan()
{
  // [v5] NOTE: GPIO3 is Gyro_RST (BNO reset), NOT a mode pin — do NOT drive it here.
  // compassInit() owns it and holds it HIGH (run). Driving it LOW killed the BNO.
  pinMode(TCA_A0, OUTPUT);
  pinMode(TCA_A1, OUTPUT);
  pinMode(TCA_A2, OUTPUT);
  digitalWrite(TCA_A0, LOW);
  digitalWrite(TCA_A1, LOW);
  digitalWrite(TCA_A2, LOW);
  pinMode(TCA_RST, OUTPUT);
  digitalWrite(TCA_RST, HIGH); // release mux reset
  delay(5);
  Serial.println("[I2C] scanning main bus (SDA=18 SCL=19)...");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++)
  {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0)
    {
      const char *who = (a == BNO055_ADDR) ? "  <- BNO055"
                        : (a == TCA_ADDR)  ? "  <- TCA9548A mux"
                        : (a == 0x29)      ? "  <- TCS34725 (on main bus?!)"
                                           : "";
      Serial.printf("[I2C]   0x%02X ACK%s\n", a, who);
      found++;
    }
  }
  if (!found)
    Serial.println("[I2C]   NONE found -> check 3V3 power, SDA=18 SCL=19, 4.7k pullups to 3V3");
  for (int ch = 0; ch < NUM_COLOUR; ch++)
  {
    tcaSelect(COLOUR_CH[ch]);
    Wire.beginTransmission(0x29);
    Serial.printf("[I2C]   mux ch%d: TCS34725@0x29 %s\n", ch, (Wire.endTransmission() == 0) ? "OK" : "-- absent");
    tcaReset(); // [v5] release the channel (recover if a dead sensor clamped SDA) before the next
  }
}

void lineUpdate()
{
  lineMask = 0;

  for (int i = 0; i < NUM_COLOUR; i++)
  {
    if (!colourGood[i])
      continue;

    tcaSelect(COLOUR_CH[i]);

    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    bool white = false;

    if (c > LINE_CLEAR_MIN)
    {
      uint16_t mx = max(r, max(g, b));
      uint16_t mn = min(r, min(g, b));

      if (mx == 0 || (mx - mn) * 100 / mx < 30)
        white = true;
    }

    if (white)
      lineMask |= (1 << i);
  }

  tcaDeselect();
  lineDetected = (lineMask != 0);
}

// ============================================================================
//  10.  RCJ RUN/STOP  (pin 9 ISR, rule 2.12)
// ============================================================================
void rcjISR()
{
  runSignal = (digitalReadFast(PIN_RCJ_RUN) == HIGH);
  runEdge = true;
}

void rcjInit()
{
  pinMode(PIN_RCJ_RUN, INPUT_PULLDOWN); // LOW (=STOP) when module absent
  runSignal = (digitalRead(PIN_RCJ_RUN) == HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_RCJ_RUN), rcjISR, CHANGE);
}

// ============================================================================
//  11.  ESP UART PROTOCOL  (RX parse for 4 cameras + TX helpers)
// ============================================================================
// Per-port byte-stream parser. First byte classifies the frame:
//   0x01..0x04  -> detection packet (2 or 4 bytes)
//   0xB0/0xB1   -> async event from forward ESP
//   0x20..0x7E / \n -> ASCII line (CAL replies, relayed telemetry, etc.)
struct PortParser
{
  uint8_t buf[72];
  int len;
  int need;
  uint8_t kind;
  char ascii[96];
  int aidx;
};
PortParser parser[4];

void handleDetection(const uint8_t *p, int n)
{
  uint8_t espId = p[0];
  if (espId >= 1 && espId <= 4)
    camPkt[espId]++; // [v5] count EVERY packet (detect or no-detect) -> liveness
  if (espId == ESP_ID_FRONT && forwardPortIdx < 0)
  { /* learned elsewhere */
  }
  if (n == 2 && p[1] == PACKET_NO_DETECT)
    return; // nothing seen this cam
  if (n < 4)
    return;
  int8_t type = (int8_t)p[1];
  int8_t camAng = (int8_t)p[2];
  uint8_t dist = p[3];
  int robotAng = camAng + MOUNT[espId];
  while (robotAng < 0)
    robotAng += 360;
  robotAng %= 360;
  if (espId >= 1 && espId <= 4)
  { // [v5] per-camera latest -> app Vision tab
    CamVis &cv = camVis[espId - 1];
    uint32_t now = millis();
    if (isGoalType(type))
    { // goal (any colour)
      cv.goalR = robotAng;
      cv.goalC = camAng;
      cv.goalCol = (type == OBJ_GOAL_YELLOW) ? 1 : (type == OBJ_GOAL_BLUE) ? 2
                                                                           : 0;
      cv.goalDist = dist;
      cv.tG = now;
    }
    else if (type == OBJ_LINE_WHITE)
    {
      cv.white = robotAng;
      cv.tW = now;
    }
    else if (type == OBJ_LINE_BLACK)
    {
      cv.black = robotAng;
      cv.tK = now;
    }
  }
  if (isGoalType(type))
  {
    lastGoal = {type, (int16_t)robotAng, dist, millis()};
    goalBearing = robotAng; // most recent goal bearing
  }
  // OBJ_LINE_* from cameras could refine line handling; colour sensors are primary.
}

void handleAsciiFromEsp(int portIdx, const char *line); // fwd decl
void handleWifiData(const uint8_t *data, uint8_t len);     // partner RobotMsg relay

void parseByte(int idx, uint8_t b)
{
  PortParser &P = parser[idx];
  if (P.need > 0)
  { // mid binary frame
    P.buf[P.len++] = b;
    // [FIX] Detection frame length is data-dependent: a no-detect packet is
    // 2 bytes ([ESP_ID][0xFF]) while a real detection is 4. Decide the moment
    // the 2nd byte arrives, so we never over-consume bytes that belong to a
    // following ASCII command (that bug dropped the 1st char: TEST:ON -> EST:ON).
    if (P.kind == 'D' && P.len == 2)
    {
      if (P.buf[1] == PACKET_NO_DETECT)
      {
        handleDetection(P.buf, P.len);
        P.need = 0;
        P.len = 0;
        return;
      }
      P.need = 4; // real detection -> 2 more bytes
    }
    // EVT_WIFI_DATA is variable length: [EVT_WIFI_DATA][len][payload...].
    // The old 2-byte event parser dropped the RobotMsg payload, so partner
    // strategy could never really work. Keep EVT_PARTNER_FOUND as 2 bytes.
    if (P.kind == 'E' && P.buf[0] == EVT_WIFI_DATA && P.len == 2)
    {
      uint8_t payloadLen = P.buf[1];
      if (payloadLen > sizeof(P.buf) - 2)
      {
        P.need = 0;
        P.len = 0;
        return;
      }
      P.need = 2 + payloadLen;
    }
    if (P.len >= P.need)
    {
      if (P.kind == 'D')
        handleDetection(P.buf, P.len);
      else if (P.kind == 'E')
      {
        lastPartnerMsgMs = millis();
        if (P.buf[0] == EVT_PARTNER_FOUND)
        {
          partnerOnline = (P.buf[1] != 0); // 0 = not paired
          iAmHighMac = (P.buf[1] == 1);    // 1 = we hold the higher MAC
        }
        else if (P.buf[0] == EVT_WIFI_DATA)
        {
          handleWifiData(P.buf + 2, P.buf[1]);
        }
      }
      P.need = 0;
      P.len = 0;
    }
    return;
  }
  if (b >= ESP_ID_FRONT && b <= ESP_ID_LEFT)
  { // detection start
    if (b == ESP_ID_FRONT)
    {
      if (forwardPortIdx != idx)
        Serial.printf("[FWD] learned forwardPortIdx=%d (Serial%d)\n", idx, idx + 1);
      forwardPortIdx = idx;
    } // remember forward port
    P.buf[0] = b;
    P.len = 1;
    P.kind = 'D';
    P.need = 2; // start at 2 (no-detect);
    // the mid-frame block above extends need->4 only if byte[1] != 0xFF (real
    // detection). Old code forced need=4 here and corrupted command framing.
    return;
  }
  if (b == EVT_PARTNER_FOUND)
  {
    P.buf[0] = b;
    P.len = 1;
    P.kind = 'E';
    P.need = 2;
    return;
  }
  if (b == EVT_WIFI_DATA)
  {
    P.buf[0] = b;
    P.len = 1;
    P.kind = 'E';
    P.need = 2;
    return;
  } // +len handled crudely
  if (b == '\n' || b == '\r')
  { // ASCII line complete
    if (P.aidx > 0)
    {
      P.ascii[P.aidx] = 0;
      // [FIX] The forward ESP is also the source of ASCII commands. Learn the
      // forward port from a command too (not only from a binary detection
      // packet), so TLM can flow even when the camera/vision is down — no
      // need for the ESP to flood no-detect packets just to be discovered.
      if (forwardPortIdx != idx)
      {
        Serial.printf("[FWD] learned forwardPortIdx=%d (Serial%d) via ASCII\n", idx, idx + 1);
        forwardPortIdx = idx;
      }
      handleAsciiFromEsp(idx, P.ascii);
      P.aidx = 0;
    }
    return;
  }
  if (b >= 0x20 && b <= 0x7E)
  { // ASCII char
    if (P.aidx < (int)sizeof(P.ascii) - 1)
      P.ascii[P.aidx++] = (char)b;
  }
}

// NOTE on detection length: a clean fix is for the ESP to always send 4 bytes
// (pad no-detect). If it sends 2-byte no-detect, the extra 2 bytes here will be
// re-synced on the next frame start. TODO(VERIFY) packet length with ESP side.

void espRxPump()
{
  for (int i = 0; i < 4; i++)
    while (ESP_PORT[i]->available())
    {
      uint8_t b = ESP_PORT[i]->read();
      rawByte[i]++;
      parseByte(i, b);
    }
}

// ---- TX to forward ESP ----
void espSendByte(uint8_t b)
{
  if (forwardPortIdx >= 0)
    ESP_PORT[forwardPortIdx]->write(b);
}
void espSendAscii(const char *s)
{
  if (forwardPortIdx >= 0)
  {
    ESP_PORT[forwardPortIdx]->print(s);
    ESP_PORT[forwardPortIdx]->print('\n');
  }
}
void espPushRobotState(SysState s)
{
  uint8_t v = (s == S_GAME) ? ROBOT_STATE_GAME : (s == S_TEST) ? ROBOT_STATE_TEST
                                                               : ROBOT_STATE_READY;
  espSendByte(CMD_ROBOT_STATE);
  espSendByte(v);
}

// ============================================================================
//  12.  TELEMETRY (ASCII to forward ESP -> phone via SSE)
// ============================================================================
void sendTLM()
{
  char b[160];
  snprintf(b, sizeof(b), "TLM:%d,%d,%d,%d,%d,%d,%d,%d,%d",
           (sysState == S_GAME) ? 1 : (sysState == S_TEST) ? 2
                                                           : 0,
           0 /*batt: not readable on this PCB*/,
           ballVisible ? 1 : 0, (int)ballAngle, ballStrength, ballInPocket ? 1 : 0,
           lineDetected ? 1 : 0, (int)heading, (int)goalBearing);
  espSendAscii(b);
}
void sendIR()
{
  char b[160];
  int n = snprintf(b, sizeof(b), "IR:");
  for (int i = 0; i < NUM_IR; i++)
    n += snprintf(b + n, sizeof(b) - n, "%d,", irStrength[i]);
  snprintf(b + n, sizeof(b) - n, "%d", ballInPocket ? 1 : 0);
  espSendAscii(b);
}
void sendCMP()
{
  char b[48];
  snprintf(b, sizeof(b), "CMP:%d,%d", (int)heading, bnoOK ? 1 : 0);
  espSendAscii(b);
}

void sendCOL()
{
  for (int i = 0; i < NUM_COLOUR; i++)
  {
    char b[96];

    if (!colourGood[i])
    {
      snprintf(b, sizeof(b), "COL:%d,ABSENT", i);
      espSendAscii(b);
      continue;
    }

    tcaSelect(COLOUR_CH[i]);
    uint16_t r, g, bl, c;
    tcs.getRawData(&r, &g, &bl, &c);
    snprintf(b, sizeof(b), "COL:%d,%u,%u,%u,%u", i, r, g, bl, c);
    espSendAscii(b);
  }

  tcaDeselect();
}

void sendVIS()
{ // [v5] per-camera detections -> app Vision tab
  uint32_t now = millis();
  for (int c = 0; c < 4; c++)
  {
    CamVis &cv = camVis[c];
    bool gOK = cv.tG && now - cv.tG < VIS_TIMEOUT_MS;
    int gcol = gOK ? cv.goalCol : 0; // 0 none, 1 yellow, 2 blue
    int gR = gOK ? cv.goalR : -1;    // goal robot-relative angle
    int gC = gOK ? cv.goalC : -1;    // goal camera-local angle
    int w = (cv.tW && now - cv.tW < VIS_TIMEOUT_MS) ? cv.white : -1;
    int k = (cv.tK && now - cv.tK < VIS_TIMEOUT_MS) ? cv.black : -1;
    char b[56];
    snprintf(b, sizeof(b), "VIS:%d,%d,%d,%d,%d,%d", c, gcol, gR, gC, w, k);
    espSendAscii(b);
  }
}

// ============================================================================
//  13.  TEST-MODE COMMAND PARSER  (ASCII from forward ESP)
//       Executed ONLY while sysState == S_TEST (the safety gate).
// ============================================================================
void handleAsciiFromEsp(int portIdx, const char *line)
{
  Serial.printf("[RX-ESP p%d] \"%s\"  state=%d\n", portIdx, line, (int)sysState);
  // TEST entry/exit are allowed only from READY; everything physical is gated.
  if (!strcmp(line, "TEST:ON"))
  {
    bool allowFaultTest = TEST_ALLOW_FROM_FAULT && (sysState == S_FAULT);
    if (sysState == S_READY || allowFaultTest)
    {
      motorKill();
      dribblerSet(0);
      analogWrite(PIN_KICKER, 0);
      testEnteredFromFault = allowFaultTest;
      lastTestPhysicalCmdMs = 0;
      sysState = S_TEST;
      espPushRobotState(sysState);
      espSendAscii(allowFaultTest ? "ACK:TEST_ON_FAULT_BYPASS" : "ACK:TEST_ON");
    }
    else
      espSendAscii("ERR:NOT_READY");
    return;
  }
  if (!strcmp(line, "TEST:OFF"))
  {
    motorKill();
    dribblerSet(0);
    analogWrite(PIN_KICKER, 0);
    lastTestPhysicalCmdMs = 0;
    sysState = testEnteredFromFault ? S_FAULT : S_READY;
    testEnteredFromFault = false;
    espPushRobotState(sysState);
    espSendAscii((sysState == S_FAULT) ? "ACK:TEST_OFF_FAULT" : "ACK:TEST_OFF");
    return;
  }
  if (!strcmp(line, "GAME:ON"))
  {
    if (sysState == S_READY || sysState == S_TEST || sysState == S_PAUSED)
    {
      motorKill();
      dribblerSet(0);
      analogWrite(PIN_KICKER, 0);
      lastTestPhysicalCmdMs = 0;
      testEnteredFromFault = false;
      sysState = S_GAME;
      espPushRobotState(sysState);
      espSendAscii("ACK:GAME_ON");
    }
    else
      espSendAscii("ERR:GAME_NOT_READY");
    return;
  }
  if (!strcmp(line, "GAME:OFF"))
  {
    if (sysState == S_GAME || sysState == S_PAUSED || sysState == S_TEST)
    {
      motorKill();
      dribblerSet(0);
      analogWrite(PIN_KICKER, 0);
      lastTestPhysicalCmdMs = 0;
      testEnteredFromFault = false;
      sysState = S_READY;
      espPushRobotState(sysState);
      espSendAscii("ACK:GAME_OFF");
    }
    else
      espSendAscii("ERR:NOT_GAME");
    return;
  }
  if (!strcmp(line, "ESTOP"))
  {
    motorKill();
    dribblerSet(0);
    analogWrite(PIN_KICKER, 0);
    espSendAscii("ACK:ESTOP");
    return;
  }
  if (!strcmp(line, "QUERY:STATUS"))
  {
    sendTLM();
    return;
  }
  if (!strcmp(line, "IR:RAW"))
  {
    irScan();
    sendIR();
    return;
  } // [v5] fresh read in any state
  if (!strcmp(line, "COMPASS:READ"))
  {
    sendCMP();
    return;
  }
  if (!strcmp(line, "VISION:READ"))
  {
    sendVIS();
    return;
  } // [v5] per-camera detections (ungated)
  if (!strcmp(line, "COLOUR:RAW") || !strcmp(line, "COLOR:RAW"))
  {
    sendCOL();
    return;
  }
  if (!strncmp(line, "CALCAM:", 7))
  {                                        // [v5] forward HSV cal -> camera ESP UART
    int cam = atoi(line + 7);              // cam = 1..4
    const char *p = strchr(line + 7, ':'); // payload begins after the cam's ':'
    if (cam >= 1 && cam <= 4 && p)
    {
      ESP_PORT[cam - 1]->print("CAL:");
      ESP_PORT[cam - 1]->println(p + 1); // e.g. CAL:YELLOW:20,35,130,255,100,255
      espSendAscii("ACK:CALCAM");
    }
    else
      espSendAscii("ERR:CALCAM");
    return;
  }

  if (sysState != S_TEST)
  {
    espSendAscii("ERR:TEST_GATE");
    return;
  } // physical cmds gated

  int a, b, c;
  if (sscanf(line, "MOTOR:%d:%d:%d", &a, &b, &c) == 3)
  { // n dir pwm(0..100)
    int spd = map(constrain(c, 0, 100), 0, 100, 0, 255);
    if (b == 0)
      spd = -spd;
    int sp[5] = {0, ENG1_SP, ENG2_SP, ENG3_SP, ENG4_SP}, d1[5] = {0, ENG1_DR1, ENG2_DR1, ENG3_DR1, ENG4_DR1}, d2[5] = {0, ENG1_DR2, ENG2_DR2, ENG3_DR2, ENG4_DR2};
    if (a >= 1 && a <= 4)
    {
      spd *= MOTOR_INV[a];
      setMotor(d1[a], d2[a], sp[a], spd);
      markTestPhysicalCmd();
      espSendAscii("ACK:MOTOR");
    }
    else
      espSendAscii("ERR:MOTOR");
    return;
  }
  if (sscanf(line, "OMNI:%d:%d:%d", &a, &b, &c) == 3)
  { // vx vy r (-100..100)
    omniDrive(a / 100.0f, b / 100.0f, c / 100.0f);
    markTestPhysicalCmd();
    espSendAscii("ACK:OMNI");
    return;
  }
  if (sscanf(line, "KICK:%d", &a) == 1)
  {
    fireKicker(a, false);
    espSendAscii("ACK:KICK");
    return;
  }
  if (sscanf(line, "DRIBBLER:%d", &a) == 1)
  {
    dribblerSet(map(constrain(a, 0, 100), 0, 100, 0, 255));
    markTestPhysicalCmd();
    espSendAscii("ACK:DRIBBLER");
    return;
  }
  if (!strncmp(line, "GOAL_LOCK:", 10))
  {
    lockedGoalColour = (strstr(line, "yellow") ? 0 : 1);
    goalLock();
    espSendAscii("ACK:GOAL_LOCK");
    return;
  }
  espSendAscii("WARN:UNKNOWN_CMD");
}

// ============================================================================
//  14.  STRATEGY / BEHAVIOR  -- integrated from rcj_main_robot
// ----------------------------------------------------------------------------
//  Hardware, pin map, UART parser, BNO055 UART code, colour sensors, POST,
//  telemetry, and TEST safety gates remain from robocap_teensy_main.
//
//  Game logic is now the rcj_main_robot style:
//    * TWO ATTACKERS, no fixed goalie
//    * higher ball score becomes PRIMARY_ATTACKER
//    * lower score becomes SUPPORT_ATTACKER / rebound lane
//    * if no kicker is installed/enabled, finish with a dribbler push-shot
//    * line escape has top priority inside GAME
// ============================================================================

#ifndef THIS_ROBOT_ID
#define THIS_ROBOT_ID 1
#endif

// Set true only when the physical kicker is installed and legal for the robot.
// false = use the rcj_main_robot dribbler push-shot finisher.
static constexpr bool USE_KICKER_FINISH = false;

// Default target if the app did not send GOAL_LOCK yet.
// true  = attack blue goal
// false = attack yellow goal
static constexpr bool TARGET_GOAL_BLUE = true;

// ---- rcj_main_robot tuning values, adapted to robocap_teensy_main variables ----
static constexpr float KP_GOAL = 0.025f;
static constexpr float KP_BALL_TURN = 0.010f;
static constexpr uint32_t STRATEGY_PERIOD_MS = 20;
static constexpr uint32_t PARTNER_TIMEOUT_MS = 600;
static constexpr float SEARCH_ROT = 0.36f;
static constexpr float CHASE_SPEED_FAST = 0.88f;
static constexpr float CHASE_SPEED_SLOW = 0.48f;
static constexpr float CAPTURE_SPEED = 0.42f;
static constexpr float SUPPORT_SPEED = 0.45f;
static constexpr float LINE_ESCAPE_SPEED = 0.88f;

// Push-shot tuning: hold with dribbler, align to goal, release while driving.
static constexpr uint8_t PUSH_GOAL_MIN_DIST = 45;
static constexpr float PUSH_ALIGN_DEG = 14.0f;
static constexpr float PUSH_TURN_KP = 0.030f;
static constexpr float PUSH_ALIGN_FORWARD = 0.12f;
static constexpr float PUSH_DRIVE_SPEED = 0.92f;
static constexpr uint32_t PUSH_HOLD_MS = 170;
static constexpr uint32_t PUSH_RELEASE_MS = 260;
static constexpr uint32_t PUSH_RECOVER_MS = 180;

// Spin-pushback finisher: fast controlled spin while holding the ball, then release.
// This imitates the aggressive circular “pushback” style seen in strong RCJ teams.
static constexpr bool USE_SPIN_PUSHBACK = true;
static constexpr float SPIN_OMEGA_FAST = 0.78f;       // max spin while holding ball
static constexpr float SPIN_OMEGA_SLOW = 0.46f;       // safer spin when near line / no goal
static constexpr float SPIN_FORWARD = 0.16f;          // small forward pressure into ball
static constexpr float SPIN_SIDE = 0.24f;             // orbit sideways while spinning
static constexpr float SPIN_RELEASE_SPEED = 1.0f;     // release drive power toward goal
static constexpr float SPIN_RELEASE_OMEGA = 0.18f;    // keep a little angular follow-through
static constexpr float SPIN_GOAL_WINDOW_DEG = 22.0f;  // release when goal crosses front
static constexpr uint8_t SPIN_MIN_GOAL_DIST = 35;     // do not release on weak/far detection
static constexpr uint32_t SPIN_MIN_HOLD_MS = 220;
static constexpr uint32_t SPIN_MAX_HOLD_MS = 900;
static constexpr uint32_t SPIN_RELEASE_MS = 240;
static constexpr uint32_t SPIN_RECOVER_MS = 160;

// Line/recovery timings from rcj_main_robot.
static constexpr uint32_t LINE_ESCAPE_DRIVE_MS = 380;
static constexpr uint32_t LINE_ESCAPE_ROT_MS = 620;
static constexpr uint32_t RECOVER_MS = 250;

// RobotMsg transmit pacing through the forward ESP relay.
static constexpr uint32_t ROBOTMSG_TX_MS = 50;

static inline float wrap360f(float a)
{
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;
  return a;
}

static inline float wrap180f(float a)
{
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static inline float clampF(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float headingHoldOmega0()
{
  // robocap_teensy_main keeps heading relative to goalLock(); 0 means opponent goal.
  float err = heading;
  if (err > 180.0f) err -= 360.0f;
  return clampF(-err * HEADING_KP, -0.55f, 0.55f);
}

void vectorDriveAngle(float angleDeg, float speed, float omega)
{
  float rad = angleDeg * DEG_TO_RAD;
  float vx = sinf(rad) * speed;
  float vy = cosf(rad) * speed;
  omniDrive(vx, vy, omega);
}

enum class StrategyState : uint8_t
{
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

enum class AttackRole : uint8_t
{
  PRIMARY_ATTACKER,
  SUPPORT_ATTACKER,
  SOLO_ATTACKER
};

struct TeamState
{
  bool valid = false;
  uint8_t partnerId = 0;
  bool partnerBallVisible = false;
  bool partnerHasBall = false;
  bool partnerNearLine = false;
  int partnerBallAngle = 0; // -180..180
  RobotRole partnerRole = ROLE_UNKNOWN;
  uint32_t lastRx = 0;
};

StrategyState strategyState = StrategyState::SEARCH_BALL;
AttackRole attackRole = AttackRole::SOLO_ATTACKER;
TeamState team;
uint32_t msgSeq = 0;
uint32_t lastStrategyTick = 0;
uint32_t lineEscapeStart = 0;
uint32_t recoverStart = 0;
uint32_t pushShotStart = 0;
bool pushShotReleasing = false;
uint32_t spinStart = 0;
bool spinReleasing = false;
int spinDir = 1;

// ---- Goal helpers ----------------------------------------------------------
static bool goalMatchesLockedColour(int8_t type)
{
  if (!isGoalType(type)) return false;
  if (lockedGoalColour == 0xFF)
  {
    return TARGET_GOAL_BLUE ? (type == OBJ_GOAL_BLUE) : (type == OBJ_GOAL_YELLOW);
  }
  if (lockedGoalColour == 0) return type == OBJ_GOAL_YELLOW;
  if (lockedGoalColour == 1) return type == OBJ_GOAL_BLUE;
  return true;
}

bool getBestGoal(int &angleOut, uint8_t &distOut)
{
  bool found = false;
  uint8_t bestDist = 0;
  int bestAngle = 0;
  uint32_t now = millis();

  // Prefer front camera when it sees the target goal; it sits above the dribbler.
  CamVis &front = camVis[0];
  if (front.tG && now - front.tG < VIS_TIMEOUT_MS)
  {
    int8_t typ = (front.goalCol == 1) ? OBJ_GOAL_YELLOW : (front.goalCol == 2) ? OBJ_GOAL_BLUE : -1;
    if (goalMatchesLockedColour(typ))
    {
      angleOut = wrap180f(front.goalR);
      distOut = front.goalDist;
      return true;
    }
  }

  for (int i = 0; i < 4; i++)
  {
    CamVis &cv = camVis[i];
    if (!cv.tG || now - cv.tG >= VIS_TIMEOUT_MS) continue;
    int8_t typ = (cv.goalCol == 1) ? OBJ_GOAL_YELLOW : (cv.goalCol == 2) ? OBJ_GOAL_BLUE : -1;
    if (!goalMatchesLockedColour(typ)) continue;

    uint8_t dist = cv.goalDist;
    if (!found || dist > bestDist)
    {
      found = true;
      bestDist = dist;
      bestAngle = wrap180f(cv.goalR);
    }
  }

  if (!found && (millis() - lastGoal.t < 500) && goalMatchesLockedColour(lastGoal.type))
  {
    found = true;
    bestAngle = wrap180f(lastGoal.angleRobot);
    bestDist = lastGoal.dist;
  }

  if (!found) return false;
  angleOut = bestAngle;
  distOut = bestDist;
  return true;
}

// Returns an escape direction, robot-relative 0..359, away from detected line.
bool getLineThreat(int &escapeAngleOut)
{
  bool threat = false;
  float sx = 0.0f, sy = 0.0f;
  uint32_t now = millis();

  if (lineMask)
  {
    threat = true;

    for (int i = 0; i < NUM_COLOUR; i++)
    {
      if (!(lineMask & (1 << i))) continue;

      float escape = wrap360f(COLOUR_BEARING[i] + 180.0f);
      float rad = escape * DEG_TO_RAD;

      sx += sinf(rad);
      sy += cosf(rad);
    }
  }

  for (int i = 0; i < 4; i++)
  {
    CamVis &cv = camVis[i];
    if (!cv.tW || now - cv.tW >= VIS_TIMEOUT_MS) continue;
    threat = true;
    float escape = wrap360f(cv.white + 180.0f);
    float rad = escape * DEG_TO_RAD;
    sx += sinf(rad);
    sy += cosf(rad);
  }

  if (!threat) return false;
  escapeAngleOut = (int)wrap360f(atan2f(sx, sy) * RAD_TO_DEG);
  return true;
}

// ---- Team comms: parse/send RobotMsg through forward ESP ------------------
void handleWifiData(const uint8_t *data, uint8_t len)
{
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

  partnerOnline = true;
  lastPartnerMsgMs = team.lastRx;
  partnerScore = (int)constrain(msg.ball_radius_px, 0, 1000);
}

void sendRobotMsg()
{
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
  msg.role = ROLE_ATTACKER; // two-attacker system: both are attackers, with primary/support locally
  msg.battery_pct = 100;    // no battery ADC wired on this PCB

  if (ballVisible) msg.flags |= ENMSG_BALL_VISIBLE;
  if (ballInPocket) msg.flags |= ENMSG_HAS_BALL;

  int goalAng = 0;
  uint8_t goalDist = 0;
  if (getBestGoal(goalAng, goalDist))
  {
    if (lockedGoalColour == 0) msg.flags |= ENMSG_YGOAL_VISIBLE;
    else msg.flags |= ENMSG_BGOAL_VISIBLE;
  }

  int esc = 0;
  if (getLineThreat(esc)) msg.flags |= ENMSG_NEAR_LINE;

  msg.ball_angle = ballVisible ? (int16_t)(wrap180f(ballAngle) * 10.0f) : -32768;
  msg.ball_radius_px = (uint16_t)constrain(ballStrength * 20, 0, 65535);

  ESP_PORT[forwardPortIdx]->write((uint8_t)CMD_RELAY_DATA);
  ESP_PORT[forwardPortIdx]->write((uint8_t)sizeof(RobotMsg));
  ESP_PORT[forwardPortIdx]->write((uint8_t *)&msg, sizeof(RobotMsg));
}

void updateTeamFreshness()
{
  if (team.valid && millis() - team.lastRx > PARTNER_TIMEOUT_MS) team.valid = false;
  if (!team.valid) partnerOnline = false;
}

// ---- Role auction: TWO ATTACKERS, NO GOALIE -------------------------------
float myBallScore()
{
  if (!ballVisible) return 0.0f;
  float rel = fabsf(wrap180f(ballAngle));
  float angleQuality = (rel < 25.0f) ? 1.4f : (rel < 70.0f) ? 1.0f : 0.55f;
  float strength = constrain((float)ballStrength / (float)IR_SAMPLES_PER_CH, 0.1f, 1.4f);
  if (ballInPocket) return 999.0f;
  return strength * angleQuality;
}

float partnerBallScore()
{
  if (!team.valid || !team.partnerBallVisible) return 0.0f;
  if (team.partnerHasBall) return 998.0f;
  float rel = fabsf((float)team.partnerBallAngle);
  float angleQuality = (rel < 25.0f) ? 1.4f : (rel < 70.0f) ? 1.0f : 0.55f;
  return angleQuality;
}

void updateAttackRole()
{
  if (!team.valid)
  {
    attackRole = AttackRole::SOLO_ATTACKER;
    return;
  }

  if (team.partnerHasBall && !ballInPocket)
  {
    attackRole = AttackRole::SUPPORT_ATTACKER;
    return;
  }
  if (ballInPocket)
  {
    attackRole = AttackRole::PRIMARY_ATTACKER;
    return;
  }

  attackRole = (myBallScore() >= partnerBallScore()) ? AttackRole::PRIMARY_ATTACKER : AttackRole::SUPPORT_ATTACKER;
}

// ---- Behaviours ------------------------------------------------------------
void enterLineEscape()
{
  strategyState = StrategyState::AVOID_LINE;
  lineEscapeStart = millis();
  dribblerSet(0);
}

void behaviorAvoidLine()
{
  int escapeAngle = 180;
  getLineThreat(escapeAngle);
  uint32_t elapsed = millis() - lineEscapeStart;

  if (elapsed < LINE_ESCAPE_DRIVE_MS)
  {
    vectorDriveAngle(escapeAngle, LINE_ESCAPE_SPEED, 0);
    return;
  }
  if (elapsed < LINE_ESCAPE_ROT_MS)
  {
    omniDrive(0, 0, 0.45f);
    return;
  }
  strategyState = StrategyState::RECOVER;
  recoverStart = millis();
}

void behaviorRecover()
{
  dribblerSet(0);
  if (millis() - recoverStart < RECOVER_MS)
  {
    omniDrive(0, 0, headingHoldOmega0());
    return;
  }
  strategyState = StrategyState::SEARCH_BALL;
}

void behaviorSearchBall()
{
  dribblerSet(0);

  // No ball: rotate fast in place until an IR sensor sees the ball.
  // If the partner robot sees it, rotate while drifting slightly toward its angle.
  if (team.valid && team.partnerBallVisible && !ballVisible)
  {
    vectorDriveAngle((float)team.partnerBallAngle, 0.20f, 0.55f);
    return;
  }

  omniDrive(0.0f, 0.0f, 0.75f);
}

void behaviorChaseBall()
{
  dribblerSet(0);

  if (!ballVisible)
  {
    strategyState = StrategyState::SEARCH_BALL;
    return;
  }

  float err = wrap180f(ballAngle);
  float absErr = fabsf(err);

  // Step 1: rotate toward the ball first.
  // ballAngle positive = ball is clockwise/right.
  // omniDrive omega positive = CCW, so the sign is opposite.
  if (absErr > 15.0f)
  {
    float omega;

    if (absErr > 90.0f)
      omega = 1.0f;
    else if (absErr > 45.0f)
      omega = 0.85f;
    else
      omega = 0.65f;

    if (err > 0.0f)
      omega = -omega;

    omniDrive(0.0f, 0.0f, omega);
    return;
  }

  // Step 2: the ball is almost in front, so drive straight at it.
  strategyState = StrategyState::CAPTURE_BALL;
}

void behaviorCaptureBall()
{
  dribblerSet(DRIBBLER_RUN_PWM);

  if (ballInPocket)
  {
    strategyState = StrategyState::ATTACK_GOAL;
    return;
  }

  if (!ballVisible)
  {
    strategyState = StrategyState::SEARCH_BALL;
    return;
  }

  float err = wrap180f(ballAngle);
  float absErr = fabsf(err);

  // If the ball moved away from the front, stop driving and rotate to it again.
  if (absErr > 22.0f)
  {
    strategyState = StrategyState::CHASE_BALL;
    return;
  }

  // Ball is in front: drive straight forward fast.
  omniDrive(0.0f, 1.0f, 0.0f);
}

void startPushShot()
{
  pushShotReleasing = false;
  pushShotStart = millis();
  strategyState = StrategyState::PUSH_SHOT;
}

bool shouldPushShot()
{
  if (!ballInPocket) return false;
  int goalAngle = 0;
  uint8_t goalDist = 0;
  if (!getBestGoal(goalAngle, goalDist)) return false;
  return goalDist >= PUSH_GOAL_MIN_DIST;
}

void behaviorPushShot()
{
  int goalAngle = 0;
  uint8_t goalDist = 0;

  if (!ballInPocket && ballVisible)
  {
    pushShotReleasing = false;
    strategyState = StrategyState::CAPTURE_BALL;
    return;
  }

  if (!getBestGoal(goalAngle, goalDist))
  {
    dribblerSet(DRIBBLER_RUN_PWM);
    omniDrive(0.0f, 0.10f, 0.42f);
    return;
  }

  float err = wrap180f((float)goalAngle);
  if (!pushShotReleasing)
  {
    dribblerSet(DRIBBLER_RUN_PWM);
    if (fabsf(err) > PUSH_ALIGN_DEG)
    {
      float omega = clampF(-err * PUSH_TURN_KP, -0.78f, 0.78f);
      omniDrive(0.0f, PUSH_ALIGN_FORWARD, omega);
      return;
    }
    pushShotReleasing = true;
    pushShotStart = millis();
  }

  uint32_t elapsed = millis() - pushShotStart;
  if (elapsed < PUSH_HOLD_MS)
  {
    dribblerSet(DRIBBLER_RUN_PWM);
    vectorDriveAngle((float)goalAngle, PUSH_DRIVE_SPEED, 0.0f);
    return;
  }
  if (elapsed < PUSH_HOLD_MS + PUSH_RELEASE_MS)
  {
    dribblerSet(0);
    vectorDriveAngle((float)goalAngle, 1.0f, 0.0f);
    return;
  }
  if (elapsed < PUSH_HOLD_MS + PUSH_RELEASE_MS + PUSH_RECOVER_MS)
  {
    dribblerSet(0);
    vectorDriveAngle((float)goalAngle, 0.55f, 0.0f);
    return;
  }

  motorKill();
  dribblerSet(0);
  pushShotReleasing = false;
  strategyState = StrategyState::RECOVER;
  recoverStart = millis();
}


void startSpinPushback()
{
  spinStart = millis();
  spinReleasing = false;

  int goalAngle = 0;
  uint8_t goalDist = 0;
  if (getBestGoal(goalAngle, goalDist))
  {
    // Choose the shorter rotation that brings the goal through the front.
    float err = wrap180f((float)goalAngle);
    spinDir = (err >= 0.0f) ? -1 : 1;
  }
  else if (ballVisible)
  {
    // If goal is unknown, choose a direction that helps wrap around the ball.
    float b = wrap180f(ballAngle);
    spinDir = (b >= 0.0f) ? 1 : -1;
  }
  else
  {
    // Different default for each robot to avoid mirrored collisions.
    spinDir = (THIS_ROBOT_ID == 1) ? 1 : -1;
  }

  if (spinDir == 0) spinDir = 1;
  strategyState = StrategyState::SPIN_PUSHBACK;
}

void behaviorSpinPushback()
{
  int escapeAngle = 0;
  if (getLineThreat(escapeAngle))
  {
    spinReleasing = false;
    enterLineEscape();
    return;
  }

  if (!ballInPocket && ballVisible)
  {
    // Ball slipped out but is still visible: immediately re-capture.
    spinReleasing = false;
    strategyState = StrategyState::CAPTURE_BALL;
    return;
  }

  int goalAngle = 0;
  uint8_t goalDist = 0;
  bool goalFound = getBestGoal(goalAngle, goalDist);
  uint32_t elapsed = millis() - spinStart;

  if (!spinReleasing)
  {
    dribblerSet(DRIBBLER_RUN_PWM);

    bool goodReleaseWindow = goalFound &&
                             goalDist >= SPIN_MIN_GOAL_DIST &&
                             elapsed >= SPIN_MIN_HOLD_MS &&
                             fabsf(wrap180f((float)goalAngle)) <= SPIN_GOAL_WINDOW_DEG;

    bool timeoutRelease = goalFound &&
                          goalDist >= SPIN_MIN_GOAL_DIST &&
                          elapsed >= SPIN_MAX_HOLD_MS;

    if (goodReleaseWindow || timeoutRelease)
    {
      spinReleasing = true;
      spinStart = millis();
      return;
    }

    // Controlled aggressive spin: side + forward + rotation.
    // Slightly reduce spin when we do not have a reliable goal yet.
    float spinOmega = goalFound ? SPIN_OMEGA_FAST : SPIN_OMEGA_SLOW;
    omniDrive(SPIN_SIDE * spinDir, SPIN_FORWARD, spinOmega * spinDir);
    return;
  }

  elapsed = millis() - spinStart;

  if (elapsed < SPIN_RELEASE_MS)
  {
    dribblerSet(0);
    float shootAngle = goalFound ? (float)goalAngle : 0.0f;
    vectorDriveAngle(shootAngle, SPIN_RELEASE_SPEED, SPIN_RELEASE_OMEGA * spinDir);
    return;
  }

  if (elapsed < SPIN_RELEASE_MS + SPIN_RECOVER_MS)
  {
    dribblerSet(0);
    float shootAngle = goalFound ? (float)goalAngle : 0.0f;
    vectorDriveAngle(shootAngle, 0.55f, 0.0f);
    return;
  }

  motorKill();
  dribblerSet(0);
  spinReleasing = false;
  strategyState = StrategyState::RECOVER;
  recoverStart = millis();
}

void behaviorAttackGoal()
{
  dribblerSet(DRIBBLER_RUN_PWM);

  if (!ballInPocket && ballVisible)
  {
    pushShotReleasing = false;
    strategyState = StrategyState::CAPTURE_BALL;
    return;
  }

  if (!USE_KICKER_FINISH && USE_SPIN_PUSHBACK && ballInPocket)
  {
    int a = 0;
    uint8_t d = 0;

    if (getBestGoal(a, d) && d >= SPIN_MIN_GOAL_DIST)
    {
      startSpinPushback();
      return;
    }
  }

  if (!USE_KICKER_FINISH && shouldPushShot())
  {
    startPushShot();
    return;
  }

  int goalAngle = 0;
  uint8_t goalDist = 0;
  bool goalFound = getBestGoal(goalAngle, goalDist);

  if (goalFound)
  {
    float err = wrap180f((float)goalAngle);
    float omega = clampF(-err * KP_GOAL, -0.65f, 0.65f);

    if (fabsf(err) < 8.0f && goalDist > 70 && ballInPocket)
    {
      motorKill();
      if (USE_KICKER_FINISH)
      {
        fireKicker(100, false);
        strategyState = StrategyState::RECOVER;
        recoverStart = millis();
      }
      else
      {
        startPushShot();
      }
      return;
    }

    float forward = (fabsf(err) < 28.0f) ? 0.78f : 0.32f;
    omniDrive(0.0f, forward, omega);
    return;
  }

  // No goal visible: rotate slowly while holding the ball until any camera sees it.
  omniDrive(0.0f, 0.18f, 0.38f);
}

void behaviorSupportAttacker()
{
  dribblerSet(0);

  if (ballInPocket)
  {
    attackRole = AttackRole::PRIMARY_ATTACKER;
    strategyState = StrategyState::ATTACK_GOAL;
    return;
  }

  if (ballVisible && (!team.valid || !team.partnerBallVisible))
  {
    attackRole = AttackRole::PRIMARY_ATTACKER;
    strategyState = StrategyState::CHASE_BALL;
    return;
  }

  if (ballVisible)
  {
    float rel = wrap180f(ballAngle);
    float supportAngle = wrap360f(ballAngle + ((THIS_ROBOT_ID == 1) ? 65.0f : -65.0f));
    vectorDriveAngle(supportAngle, SUPPORT_SPEED, headingHoldOmega0());

    if (ballStrength >= (IR_SAMPLES_PER_CH / 2) && fabsf(rel) < 45.0f && !team.partnerHasBall)
      strategyState = StrategyState::CHASE_BALL;
    return;
  }

  omniDrive((THIS_ROBOT_ID == 1) ? 0.32f : -0.32f, 0.15f, 0.22f);
}

void updateStrategyState()
{
  int escapeAngle = 0;
  if (getLineThreat(escapeAngle) && strategyState != StrategyState::AVOID_LINE)
  {
    enterLineEscape();
    return;
  }

  updateAttackRole();

  if (strategyState == StrategyState::AVOID_LINE ||
      strategyState == StrategyState::RECOVER ||
      strategyState == StrategyState::SPIN_PUSHBACK ||
      strategyState == StrategyState::PUSH_SHOT)
    return;

  if (!USE_KICKER_FINISH && USE_SPIN_PUSHBACK && ballInPocket)
  {
    int a = 0;
    uint8_t d = 0;

    if (getBestGoal(a, d) && d >= SPIN_MIN_GOAL_DIST)
    {
      startSpinPushback();
      return;
    }
  }

  if (!USE_KICKER_FINISH && shouldPushShot())
  {
    startPushShot();
    return;
  }

  if (ballInPocket)
  {
    strategyState = StrategyState::ATTACK_GOAL;
    return;
  }

  if (attackRole == AttackRole::SUPPORT_ATTACKER)
  {
    strategyState = StrategyState::SUPPORT_ATTACKER;
    return;
  }

  if (ballVisible)
  {
    strategyState = (fabsf(wrap180f(ballAngle)) < 25.0f && ballStrength >= IR_PRESENT_MIN + 1)
                    ? StrategyState::CAPTURE_BALL
                    : StrategyState::CHASE_BALL;
    return;
  }

  strategyState = StrategyState::SEARCH_BALL;
}

void runStrategy()
{
  uint32_t now = millis();
  if (now - lastStrategyTick < STRATEGY_PERIOD_MS) return;
  lastStrategyTick = now;

  updateTeamFreshness();
  sendRobotMsg();
  updateStrategyState();

  switch (strategyState)
  {
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
  case StrategyState::SPIN_PUSHBACK:
    behaviorSpinPushback();
    break;
  case StrategyState::PUSH_SHOT:
    behaviorPushShot();
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
//  15.  POST  (power-on self test; battery check removed — not wired to CPU)
// ============================================================================
bool runPOST()
{
  bool ok = true;
  // I2C sanity: BNO055 must answer
  if (!bnoOK)
  {
    espSendAscii("LOG:POST BNO055 FAIL");
    Serial.println("[POST] BNO055 FAIL - compass not answering on UART (Serial5, 115200; check S1->+ for UART mode, TX/RX crossover)");
    // Keep GAME locked on a compass fault. TEST:ON can still enter TEST from
    // S_FAULT when TEST_ALLOW_FROM_FAULT=true, so hardware can be bench-tested
    // while the BNO055 wiring/config is fixed.
    ok = false;
    Serial.println("[POST] BNO055 fault: GAME locked; TEST bypass allowed only if TEST_ALLOW_FROM_FAULT=true");
  }
  // brief motor continuity tick (10%) - disabled unless explicitly enabled
  if (POST_MOTOR_TICK)
  {
    omniDrive(0, 0.10f, 0);
    delay(120);
    motorKill();
  }
  // kicker test pulse (very short) - only when a physical kicker is enabled
  if (USE_KICKER_FINISH)
  {
    analogWrite(PIN_KICKER, 60);
    delay(5);
    analogWrite(PIN_KICKER, 0);
  }
  // U20 pocket should read "empty" with no ball
  irScan();
  if (ballInPocket)
  {
    espSendAscii("LOG:POST POCKET STUCK"); /* warn only */
  }
  espSendAscii(ok ? "LOG:POST OK" : "LOG:POST FAIL");
  return ok;
}

// ============================================================================
//  16.  SETUP / LOOP
// ============================================================================
void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] robocap_teensy_main (DEBUG build)");
  for (int i = 0; i < 4; i++)
    ESP_PORT[i]->begin(UART_BAUD);
  Wire.begin();

  buildIrRing();
  motorsInit();
  actuatorsInit();
  irInit();
  colourInit();
  compassInit();
  rcjInit();
  i2cScan(); // [v5] print the I2C bus map at boot (BNO055 / TCA9548A / TCS34725)
  for (int i = 0; i < 4; i++)
  {
    parser[i].need = 0;
    parser[i].len = 0;
    parser[i].aidx = 0;
  }

  sysState = S_POST;
  bool ok = runPOST();
  
  //sysState = ok ? S_GAME : S_FAULT;  // TEMP: bypass RCJ Start/Stop
  if (AUTO_GAME_ON_BOOT && (ok || AUTO_GAME_IGNORE_POST_FAIL))
  {
    sysState = S_GAME;
    Serial.println("[BOOT] *** TEMP AUTO GAME ENABLED - bench only ***");
    if (!ok)
      Serial.println("[BOOT] *** POST failed but AUTO_GAME_IGNORE_POST_FAIL is true ***");
  }
  else
  {
    sysState = ok ? S_READY : S_FAULT;
  }

  espPushRobotState(sysState);
  Serial.printf("[BOOT] POST=%s  bnoOK=%d  -> state=%s\n",
                ok ? "OK" : "FAIL", bnoOK,
                (sysState == S_GAME) ? "GAME" : (sysState == S_READY) ? "READY" : "FAULT");
}

uint32_t lastTlm = 0;

void loop()
{
  espRxPump();
#if BNO_PING_DEBUG
  bnoPingDebug();
#endif
  compassUpdate();
  static uint32_t lastCam = 0;
  if (millis() - lastCam > 2000)
  {
    lastCam = millis();
    Serial.printf("[CAM] pkts(byID) F=%u R=%u Rr=%u L=%u | rawBytes(byPort) S1=%u S2=%u S3=%u S4=%u\n",
                  camPkt[1], camPkt[2], camPkt[3], camPkt[4], rawByte[0], rawByte[1], rawByte[2], rawByte[3]);
    camPkt[1] = camPkt[2] = camPkt[3] = camPkt[4] = 0;
    rawByte[0] = rawByte[1] = rawByte[2] = rawByte[3] = 0;
  }

  // ---- RCJ run/stop edges (rule 2.12) ----

  if (runEdge)
  {
    runEdge = false;
    Serial.printf("[RCJ] edge! runSignal=%d  state=%d\n", runSignal, (int)sysState);
    if (AUTO_GAME_ON_BOOT)
    {
      // TEMP: keep GAME sticky during bench auto-game testing.
      return;
    }
    // In bench TEST mode, ignore GO while already in TEST so pin-9 EMI or an
    // absent RCJ module cannot kick the robot into GAME during joystick tests.
    // For competition, set TEST_IGNORE_RCJ_GO=false so GO overrides TEST.
#if 1
    if (sysState==S_GAME && !runSignal) { gamePrevState=S_GAME; sysState=S_PAUSED; motorKill(); dribblerSet(0); espPushRobotState(sysState); }
    else if (sysState==S_PAUSED && runSignal) { sysState=S_GAME; espPushRobotState(sysState); }
    else if (sysState==S_READY && runSignal) { sysState=S_GAME; espPushRobotState(sysState); }
    else if (sysState==S_TEST  && runSignal)
    {
      if (TEST_IGNORE_RCJ_GO || testEnteredFromFault)
      {
        motorKill();
        dribblerSet(0);
        analogWrite(PIN_KICKER, 0);
        espSendAscii(testEnteredFromFault ? "ERR:POST_FAIL_GAME_LOCKED" : "LOG:RCJ_GO_IGNORED_IN_TEST");
      }
      else
      {
        motorKill();
        dribblerSet(0);
        analogWrite(PIN_KICKER, 0);
        sysState=S_GAME;
        espPushRobotState(sysState);
      }
    }
#endif
  }


  switch (sysState)
  {
  case S_GAME:
    // hardware enable switch must also be ON to drive (schematic SW2)
    if (REQUIRE_SW2_ENABLE && digitalRead(PIN_SW2) != SW2_ENABLE_LEVEL)
    {
      motorKill();
      break;
    }
    
    irScan();
    lineUpdate();
    runStrategy();
    break;

  case S_TEST:
    // physical actions handled in command parser; keep sensors fresh for UI
    irScan();
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

 // TEMP: GAME ONLY MODE
//sysState = S_GAME;
//irScan();
//lineUpdate(); 
//runStrategy();

  // periodic telemetry to phone (READY/TEST/GAME)
  if (millis() - lastTlm > 200)
  {
    lastTlm = millis();
    if (sysState != S_FAULT)
      sendTLM();
  }
}