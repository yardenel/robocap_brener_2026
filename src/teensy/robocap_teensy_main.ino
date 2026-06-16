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
//    4x L298N -> 4 Mecanum wheels (X config)
//    Kicker solenoid (MOSFET) + Dribbler motor (PWM)
//    20x TSOP34838 IR (16 via 74HC4067 mux + 4 direct; U20 = ball-in-pocket)
//    BNO055 IMU on I2C @0x28   (mode pin driven LOW = I2C)
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
#define IR_DIR_17 24  // U17 direct
#define IR_DIR_18 25  // U18 direct
#define IR_DIR_19 26  // U19 direct
#define IR_DIR_20 27  // U20 direct = BALL-IN-POCKET (dribbler) sensor

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
const int POCKET_MIN_HITS = 12;   // TODO(TUNE) U20 hits => ball in pocket

// ---- Colour / white-line ----
const uint16_t LINE_CLEAR_MIN = 1500; // TODO(TUNE) clear-channel threshold for white
// (white = bright AND roughly equal R/G/B; tune with CAL on the real field)

// ---- Strategy thresholds (relative IR strength / camera radius) ----
const int BALL_CLOSE_STR = 140; // TODO(TUNE) "ball is near" strength
// (alignment windows now live in section 14: KICK_FACE_DEG / BACKSPIN_DEG / ATTACK_ANGLE_DEG)

// ============================================================================
//  3.  IR RING TABLE   *** VERIFY THIS AGAINST THE PHYSICAL BUILD ***
// ----------------------------------------------------------------------------
//  19 perimeter sensors (U1..U19) + 1 pocket sensor (U20).
//  ASSUMPTIONS (you confirmed U1=front=0deg, U20=pocket, 16 muxed + 4 direct):
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
  // 16 muxed
  for (int i = 0; i < 16; i++)
  {
    IR_RING[i].src = IR_MUX;
    IR_RING[i].chOrPin = i;                     // mux channel i
    IR_RING[i].bearing = i * (360.0f / NUM_IR); // even spread
  }
  // 3 direct (U17..U19)
  const uint8_t dpin[3] = {IR_DIR_17, IR_DIR_18, IR_DIR_19};
  for (int i = 16; i < 19; i++)
  {
    IR_RING[i].src = IR_DIRECT;
    IR_RING[i].chOrPin = dpin[i - 16];
    IR_RING[i].bearing = i * (360.0f / NUM_IR); // TODO(VERIFY)
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
  uint32_t tG, tW, tK;
};
CamVis camVis[4] = {};                     // goalCol: 0 none, 1 yellow, 2 blue
const uint32_t VIS_TIMEOUT_MS = 600;       // drop a reading to -1 if not refreshed within this
CamObj lastGoal = {-1, 0, 0, 0};           // most recent goal sighting (any cam)
const int MOUNT[5] = {0, 0, 90, 180, 270}; // index by ESP_ID (0 unused)

// ---- Line ----
bool lineDetected = false;

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
//  5.  MOTORS  (verified kinematics + pins from testing.ino & schematic)
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
void mecanumDrive(float vx, float vy, float omega)
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
  setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, (int)(vRF * 255 * g));
  setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, (int)(vRR * 255 * g));
  setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, (int)(vLR * 255 * g));
  setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, (int)(vLF * 255 * g));
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
  mecanumDrive(vx, vy, omega);
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

  // pocket sensor (U20 direct, active-LOW)
  uint8_t ph = 0;
  for (int k = 0; k < IR_SAMPLES_PER_CH; k++)
    if (digitalRead(IR_DIR_20) == LOW)
      ph++;
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
  bool seen = false;
  for (int i = 0; i < NUM_COLOUR && !seen; i++)
  {
    if (!colourGood[i])
      continue; // [v5] skip dead channels (won't clamp the bus)
    tcaSelect(COLOUR_CH[i]);
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    // white = bright clear AND low colour spread (tune on real field via CAL)
    if (c > LINE_CLEAR_MIN)
    {
      uint16_t mx = max(r, max(g, b)), mn = min(r, min(g, b));
      if (mx == 0 || (mx - mn) * 100 / mx < 30)
        seen = true; // TODO(TUNE) spread
    }
  }
  tcaDeselect(); // [v5] leave the bus free between scans
  lineDetected = seen;
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
    if (P.len >= P.need)
    {
      if (P.kind == 'D')
        handleDetection(P.buf, P.len);
      // events 0xB0/0xB1 carry partner data; mark freshness for the auction (ch.7.1)
      else if (P.kind == 'E')
      {
        lastPartnerMsgMs = millis();
        if (P.buf[0] == EVT_PARTNER_FOUND)
        {
          partnerOnline = (P.buf[1] != 0); // 0 = not paired
          iAmHighMac = (P.buf[1] == 1);    // 1 = we hold the higher MAC (attacker on a tie)
        }
        // TODO(ESP): when EVT_WIFI_DATA carries the partner RobotMsg, parse its
        //            ball strength into partnerScore here (v3 struct has no explicit
        //            auction_score field -> use ball_radius_px as the proxy).
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
    if (sysState == S_READY)
    {
      sysState = S_TEST;
      espPushRobotState(sysState);
      espSendAscii("ACK:TEST_ON");
    }
    else
      espSendAscii("ERR:NOT_READY");
    return;
  }
  if (!strcmp(line, "TEST:OFF"))
  {
    motorKill();
    dribblerSet(0);
    sysState = S_READY;
    espPushRobotState(sysState);
    espSendAscii("ACK:TEST_OFF");
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
      setMotor(d1[a], d2[a], sp[a], spd);
    espSendAscii("ACK:MOTOR");
    return;
  }
  if (sscanf(line, "OMNI:%d:%d:%d", &a, &b, &c) == 3)
  { // vx vy r (-100..100)
    mecanumDrive(a / 100.0f, b / 100.0f, c / 100.0f);
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
//  14.  STRATEGY / BEHAVIOR  ―  faithful to strategy doc v2.2 (ch.6,7,8)
// ----------------------------------------------------------------------------
//  Runs ONLY inside S_GAME (ch.6 intro). Compass convention: heading 0deg =
//  OPPONENT goal, 180deg = our goal (set by Goal-Lock at kick-off, ch.2/5.3).
//  Ball "distance" is the IR signal strength proxy (higher = closer, ch.5.2).
// ============================================================================

// ---- doc-derived thresholds (ch.6.3 / 7.1 / 8) ; tune on the field ----
const int ATTACK_ANGLE_DEG = 15;         // ch.6.3 ATTACK entry: ball within +-15deg of front
const int KICK_FACE_DEG = 10;            // ch.6.3 cond2: FacingGoal < +-10deg (compass)
const int BACKSPIN_DEG = 20;             // ch.7.3 Back-Spin: back-to-goal +-20deg
const int BACKSPIN_PCT = 50;             // TODO(TUNE) light release pulse force
const uint8_t GOAL_DIST_NEAR = 60;       // ch.6.3 cond3 (<1.5m) proxy on camera dist byte
                                         // TODO(VERIFY) byte meaning: assume bigger=closer
const int ORBIT_STR_TARGET = 90;         // TODO(TUNE) IR strength at ~30cm orbit radius
const int CHARGE_STR = 150;              // TODO(TUNE) ch.8 danger-zone proxy (ball near goal)
const uint32_t AUCTION_TIMEOUT_MS = 500; // ch.7.1 no-heartbeat -> SOLO
const float SEARCH_SPIN = 0.45f;         // ch.6.1 spin 60-90 deg/s (tune to that rate)
const int LINE_RETREAT_MS = 350;         // ch.5.4 retreat ~10cm (tune duration)

// ---- behaviour states (ch.6 striker + ch.8 goalie) ----
enum Beh : uint8_t
{
  B_SEARCH,
  B_ORBIT,
  B_ATTACK,
  B_GOALIE_TRACK,
  B_GOALIE_CHARGE
};
Beh beh = B_SEARCH;

// shortest absolute angular difference 0..180
static inline float angDiff(float a, float b)
{
  float d = fabsf(a - b);
  if (d > 180)
    d = 360 - d;
  return d;
}

// ch.6.3 kick gates (compass-based; heading 0 = opponent goal)
bool facingGoalDirect() { return angDiff(heading, 0) <= KICK_FACE_DEG; }
bool backToGoal() { return angDiff(heading, 180) <= BACKSPIN_DEG; }                                                        // ch.7.3
bool goalClose() { return (millis() - lastGoal.t < 500) && isGoalType(lastGoal.type) && lastGoal.dist >= GOAL_DIST_NEAR; } // <1.5m proxy

// ---- ch.5.4 white-line: highest-priority override ----
void lineAvoid()
{
  motorKill();
  delay(20);
  uint32_t t0 = millis();
  while (millis() - t0 < (uint32_t)LINE_RETREAT_MS)
  {                            // retreat ~10cm (backwards)
    mecanumDrive(0, -0.5f, 0); // TODO(VERIFY) true retreat dir
    espRxPump();               // keep RCJ/telemetry responsive
  }
  // then rotate front back toward opponent goal (heading -> 0 = "to centre/forward")
  float e = heading;
  if (e > 180)
    e -= 360;
  mecanumDrive(0, 0, constrain(-e * HEADING_KP, -0.6f, 0.6f));
}

// ---- ch.7.1 dynamic role assignment (auction) + SOLO failsafe ----
void decideRole()
{
  bool partnerFresh = partnerOnline && (millis() - lastPartnerMsgMs <= AUCTION_TIMEOUT_MS);
  if (!partnerFresh)
  {
    myRole = ROLE_ATTACKER;
    return;
  } // ch.7.1 SOLO: play full game
  float q = ((ballAngle >= 0) && (ballAngle < 30 || ballAngle > 330)) ? 1.0f : 0.5f; // angle_quality
  int myScore = ballVisible ? (int)(ballStrength * q) : 0;                           // Score = (1/dist)*q ~ strength*q
  // ch.7.1 (CONFIRMED with team): higher Score -> STRIKER. On an exact tie
  // (e.g. equal strength, or neither robot sees the ball -> both score 0),
  // the robot holding the higher MAC attacks and the other plays goalie, so
  // the two robots never both pick the same role.
  if (myScore > partnerScore)
    myRole = ROLE_ATTACKER;
  else if (myScore < partnerScore)
    myRole = ROLE_GOALIE;
  else
    myRole = iAmHighMac ? ROLE_ATTACKER : ROLE_GOALIE; // MAC tie-break
}

// ---- ch.6 striker FSM: SEARCH -> ORBIT -> ATTACK ----
void runStriker()
{
  if (lineDetected)
  {
    lineAvoid();
    return;
  } // ch.5.4 top priority
  dribblerSet(DRIBBLER_RUN_PWM); // ch.6.3 capture spin always on in play

  if (ballInPocket)
  { // we hold the ball -> kick logic (ch.6.3)
    beh = B_ATTACK;
    if (facingGoalDirect() && goalClose())
    { // all 4 conditions -> Direct Kick (50ms)
      fireKicker(100, false);
      beh = B_SEARCH;
    }
    else if (backToGoal())
    { // ch.7.3 Back-Spin Release, no 180 turn
      fireKicker(BACKSPIN_PCT, true);
      beh = B_SEARCH;
    }
    else
    { // rotate shortest way to face the goal
      float e = heading;
      if (e > 180)
        e -= 360;
      mecanumDrive(0, 0, constrain(-e * 0.01f, -0.5f, 0.5f));
    }
    return;
  }

  if (!ballVisible)
  {
    beh = B_SEARCH;
    mecanumDrive(0, 0, SEARCH_SPIN);
    return;
  } // ch.6.1 scan-spin

  // ball visible, not captured. ATTACK entry (ch.6.3): close + within +-15deg + facing goal
  bool aligned = (angDiff(ballAngle, 0) <= ATTACK_ANGLE_DEG) && facingGoalDirect();
  if (ballStrength >= BALL_CLOSE_STR && aligned)
  {
    beh = B_ATTACK;
    driveToward(ballAngle, 1.0f); // ch.6.3 max accel to capture
  }
  else
  { // ch.6.2 ORBIT: get behind ball, face goal
    beh = B_ORBIT;
    float e = heading;
    if (e > 180)
      e -= 360;                                                     // +e: opponent goal is to our left
    float off = constrain(e, -60.0f, 60.0f);                        // bias approach toward the goal side
    float speed = (ballStrength < ORBIT_STR_TARGET) ? 0.7f : 0.45f; // close to ~30cm then circle
    driveToward(ballAngle + off, speed);                            // driveToward also holds heading->0
  }
}

// ---- ch.8 goalie FSM: HOME/TRACK (strafe to ball_x) + CHARGE (clear danger) ----
void runGoalie()
{
  dribblerSet(0);
  if (lineDetected)
  {
    lineAvoid();
    return;
  }

  if (ballVisible && ballStrength >= CHARGE_STR)
  { // ch.8 CHARGE: ball in danger zone
    beh = B_GOALIE_CHARGE;
    driveToward(ballAngle, 0.8f);
    if (ballInPocket)
      fireKicker(100, false); // push/clear away from goal
    return;
  }
  // HOME/TRACK: face the field (heading->0), strafe to match ball lateral position (ball_x)
  beh = B_GOALIE_TRACK;
  float vx = ballVisible ? constrain(sinf(ballAngle * DEG_TO_RAD), -1.0f, 1.0f) * 0.5f : 0.0f;
  float e = heading;
  if (e > 180)
    e -= 360;
  mecanumDrive(vx, 0, constrain(-e * HEADING_KP, -0.5f, 0.5f));
}

void runStrategy()
{
  decideRole();
  if (myRole == ROLE_GOALIE)
    runGoalie();
  else
    runStriker();
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
    // *** TEMP BENCH BYPASS *** compass fault downgraded to a warning so motors
    // can be tested via the app while the BNO055 hardware is fixed.
    // RESTORE the next line (remove the //) before real play -- gameplay needs the compass.
    // ok = false;
    Serial.println("[POST] *** BNO055 BYPASSED - bench test only, NOT for competition ***");
  }
  // brief motor continuity tick (10%)
  mecanumDrive(0, 0.10f, 0);
  delay(120);
  motorKill();
  // kicker test pulse (very short)
  analogWrite(PIN_KICKER, 60);
  delay(5);
  analogWrite(PIN_KICKER, 0);
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
  sysState = ok ? S_READY : S_FAULT;
  espPushRobotState(sysState);
  Serial.printf("[BOOT] POST=%s  bnoOK=%d  -> state=%s\n", ok ? "OK" : "FAIL", bnoOK, ok ? "READY" : "FAULT");
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
    // *** TEMP BENCH BYPASS *** No RCJ module on the bench. Once the wheels spin,
    // pin 9 (INPUT_PULLDOWN) can pick up motor EMI and fire spurious edges that
    // would knock S_TEST -> S_GAME ("escaping" TEST mid-joystick). Ignore the
    // transitions for bench so TEST stays sticky; the [RCJ] print above still
    // fires, so we can SEE any edges. RESTORE for competition (change `#if 0`
    // to `#if 1`) — rule 2.12 needs the real RUN/STOP handling below.
#if 0
    if (sysState==S_GAME && !runSignal) { gamePrevState=S_GAME; sysState=S_PAUSED; motorKill(); dribblerSet(0); espPushRobotState(sysState); }
    else if (sysState==S_PAUSED && runSignal) { sysState=S_GAME; espPushRobotState(sysState); }
    else if (sysState==S_READY && runSignal) { sysState=S_GAME; espPushRobotState(sysState); }
    else if (sysState==S_TEST  && runSignal) { motorKill(); dribblerSet(0); sysState=S_GAME; espPushRobotState(sysState); } // GO overrides TEST
#endif
  }

  switch (sysState)
  {
  case S_GAME:
    // hardware enable switch must also be ON to drive (schematic SW2)
    if (digitalRead(PIN_SW2) == LOW)
    {
      motorKill();
      break;
    } // TODO(VERIFY) active level
    irScan();
    lineUpdate();
    runStrategy();
    break;

  case S_TEST:
    // physical actions handled in command parser; keep sensors fresh for UI
    irScan();
    lineUpdate();
    break;

  case S_PAUSED:
  case S_READY:
  case S_FAULT:
  default:
    motorKill();
    dribblerSet(0);
    break;
  }

  // periodic telemetry to phone (READY/TEST/GAME)
  if (millis() - lastTlm > 200)
  {
    lastTlm = millis();
    if (sysState != S_FAULT)
      sendTLM();
  }
}
