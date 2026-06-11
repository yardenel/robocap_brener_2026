#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

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
// DRIBBLER / KICKER
// ============================================================================
// Uses single PWM pins. This avoids conflicts with the IR mux pins 30, 31, 32.

#define DRIBBLER_PIN 11
#define KICKER_PIN   2

#define DRIBBLER_SPEED 180

#define KICKER_PULSE_MS      80
#define KICKER_COOLDOWN_MS 1500

#define ESP_SERIAL Serial1

#define GAME_COMMAND 9 // external module to start/stop game.

// ============================================================================
// IR SENSOR SYSTEM
// ============================================================================

#define MUX_S0  31
#define MUX_S1  30
#define MUX_S2  29
#define MUX_S3  28
#define MUX_SIG 32

const uint8_t DIRECT_PINS[4] = {24, 25, 26, 27};

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
// IR GLOBALS
// ============================================================================

bool irSensors[NUM_IR_SENSORS];

float ballAngle = -1.0f;
int ballCount = 0;
bool hasBall = false;

// ============================================================================
// GYRO
// ============================================================================

#define BNO_RST_PIN 3

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_A, &Wire1);

bool gyroOK = false;

float gyroHeading = 0.0f;
float targetHeading = 0.0f;

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
// GLOBALS
// ============================================================================

String espLine = "";

int lastMotorSpeed[5] = {0, 0, 0, 0, 0}; // indexes 1..4
int dribblerPower = 0;
uint32_t lastKickMs = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void readEsp();
void handleEspCommand(String cmd);
void printParsedCommand(String cmd);

void setupMotorPins();
void setMotor(int dr1, int dr2, int pwmPin, int speed);
void setMotorByNumber(int motorNum, int dir, int pwm);
void stopAllMotors();
void setDribblerPower(int power);
bool kick();

void setupIRPins();
inline void setMuxChannel(uint8_t ch);
inline bool waitForBurst(uint8_t pin, uint32_t timeoutUs);
void doIRScan();
void processIRData();
void sendIRData();

void setupGyro();
float normalizeAngle(float a);
float headingError();
void doGyroRead();
String buildGyroDataMessage();
void sendGyroData();
void printGyroToSerialAndLog();
void zeroGyroTarget();
void setGyroTarget(float heading);

void logLine(const String& msg);
void logReceived(const String& msg);
void logSent(const String& msg);
void sendToEsp(const String& msg);

void sendFakeCompass();

void setExternalModulePin(int pin);
int readExternalModulePin(int pin);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);      // USB Serial Monitor
  ESP_SERIAL.begin(115200);  // ESP <-> Teensy UART

  Serial.println("test");

  setExternalModulePin(GAME_COMMAND);

  setupMotorPins();
  setupIRPins();
  setupGyro();

  stopAllMotors();
  setDribblerPower(0);
  digitalWrite(KICKER_PIN, LOW);

  delay(1000);

  logLine("SYSTEM: Teensy command reader started");
  logLine("SYSTEM: Waiting for commands from ESP/app");
  sendToEsp("LOG:Teensy online");
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  // doIRScan();
  // processIRData();
  // sendIRData();
  doGyroRead();
  sendGyroData();
  readEsp();

  // if (!readExternalModulePin(GAME_COMMAND)) {
  //   // STATE = READY // Motors shut-down
  // } else {
  //   // STATE = RUNNING // Motors can be controlled by commands from ESP/app
  // }
}

// ============================================================================
// SERIAL READING
// ============================================================================

void readEsp() {
  while (ESP_SERIAL.available()) {
    char ch = ESP_SERIAL.read();

    if (ch == '\n') {
      espLine.trim();

      if (espLine.length() > 0) {
        handleEspCommand(espLine);
      }

      espLine = "";
    } else if (ch != '\r') {
      espLine += ch;
    }
  }
}

void handleEspCommand(String cmd) {
  cmd.trim();

  logReceived(cmd);
  printParsedCommand(cmd);
}

// ============================================================================
// COMMAND PARSING
// ============================================================================

void printParsedCommand(String cmd) {
  // --------------------------------------------------------------------------
  // Emergency stop
  // --------------------------------------------------------------------------
  if (cmd == "ESTOP" || cmd == "estop") {
    stopAllMotors();
    setDribblerPower(0);
    digitalWrite(KICKER_PIN, LOW);

    logLine("ACTION: ESTOP -> stopped all motors, dribbler, and kicker");
    sendToEsp("ACK:ESTOP");
    return;
  }

  // --------------------------------------------------------------------------
  // Normal stop
  // --------------------------------------------------------------------------
  if (cmd == "STOP" || cmd == "stop") {
    stopAllMotors();
    setDribblerPower(0);
    digitalWrite(KICKER_PIN, LOW);

    logLine("ACTION: STOP -> stopped all motors, dribbler, and kicker");
    sendToEsp("ACK:STOP");
    return;
  }

  // --------------------------------------------------------------------------
  // Motor command
  // Expected format:
  // MOTOR:1:1:58
  // MOTOR:<motor number>:<direction>:<pwm>
  //
  // direction:
  //  1  = forward
  // -1  = backward
  //  0  = stop
  // --------------------------------------------------------------------------
  if (cmd.startsWith("MOTOR:")) {
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    int thirdColon = cmd.indexOf(':', secondColon + 1);

    if (firstColon > 0 && secondColon > 0 && thirdColon > 0) {
      int motorNum = cmd.substring(firstColon + 1, secondColon).toInt();
      int dir = cmd.substring(secondColon + 1, thirdColon).toInt();
      int pwm = cmd.substring(thirdColon + 1).toInt();

      setMotorByNumber(motorNum, dir, pwm);

      logLine(
        "ACTION: MOTOR motor=" + String(motorNum) +
        " dir=" + String(dir) +
        " pwm=" + String(pwm)
      );

      sendToEsp("ACK:MOTOR");
    } else {
      logLine("ERROR: Bad MOTOR command format: " + cmd);
      sendToEsp("ERR:BAD_MOTOR_FORMAT");
    }

    return;
  }

  // --------------------------------------------------------------------------
  // Dribbler slider command
  // Expected format:
  // DRIBBLER:0
  // DRIBBLER:50
  // DRIBBLER:100
  // --------------------------------------------------------------------------
  if (cmd.startsWith("DRIBBLER:")) {
    int power = cmd.substring(9).toInt();
    setDribblerPower(power);

    logLine("ACTION: DRIBBLER power=" + String(dribblerPower));
    sendToEsp("ACK:DRIBBLER:" + String(dribblerPower));
    return;
  }

  // --------------------------------------------------------------------------
  // Kick command
  // Expected:
  // KICK
  // KICK:70
  // --------------------------------------------------------------------------
  if (cmd == "KICK" || cmd.startsWith("KICK:")) {
    int power = 100;

    if (cmd.startsWith("KICK:")) {
      power = cmd.substring(5).toInt();
      power = constrain(power, 0, 100);
    }

    logLine("ACTION: KICK requested power=" + String(power));

    if (kick()) {
      sendToEsp("ACK:KICK");
    } else {
      sendToEsp("ERR:KICK_COOLDOWN");
    }

    return;
  }

  // --------------------------------------------------------------------------
  // IR request
  // This now sends REAL IR data, not fake/hardcoded data.
  // --------------------------------------------------------------------------
  if (cmd == "IR:RAW") {
    logLine("ACTION: IR raw requested");

    doIRScan();
    processIRData();
    sendIRData();

    return;
  }

  // --------------------------------------------------------------------------
  // Gyro / compass request
  // Sends REAL BNO055 heading data to the ESP/app.
  //
  // Supported commands:
  //   GYRO:READ       -> send current heading/target/error/calibration
  //   COMPASS:READ    -> alias for GYRO:READ
  //   GYRO:ZERO       -> set the current heading as target/zero
  //   GYRO:TARGET:90  -> set target heading in field degrees
  //   GYRO:STATUS     -> send gyro OK + calibration state
  // --------------------------------------------------------------------------
  if (cmd == "GYRO:READ" || cmd == "COMPASS:READ" || cmd == "GYRO:STATUS") {
    logLine("ACTION: Gyro requested");

    doGyroRead();
    sendGyroData();

    return;
  }

  if (cmd == "GYRO:ZERO" || cmd == "COMPASS:ZERO") {
    logLine("ACTION: Gyro zero requested");

    doGyroRead();
    zeroGyroTarget();
    sendToEsp("ACK:GYRO:ZERO");
    sendGyroData();

    return;
  }

  if (cmd.startsWith("GYRO:TARGET:") || cmd.startsWith("COMPASS:TARGET:")) {
    int lastColon = cmd.lastIndexOf(':');
    float newTarget = cmd.substring(lastColon + 1).toFloat();

    setGyroTarget(newTarget);

    logLine("ACTION: Gyro target=" + String(targetHeading, 1));
    sendToEsp("ACK:GYRO:TARGET:" + String(targetHeading, 1));
    sendGyroData();

    return;
  }

  // --------------------------------------------------------------------------
  // Goal lock
  // Expected:
  // GOAL_LOCK:yellow
  // GOAL_LOCK:blue
  // --------------------------------------------------------------------------
  if (cmd.startsWith("GOAL_LOCK:")) {
    String color = cmd.substring(10);

    logLine("ACTION: GOAL_LOCK color=" + color);
    sendToEsp("ACK:GOAL_LOCK:" + color);
    return;
  }

  // --------------------------------------------------------------------------
  // Unknown command
  // --------------------------------------------------------------------------
  logLine("ERROR: Unknown command: " + cmd);
  sendToEsp("ERR:UNKNOWN_CMD");
}

// ============================================================================
// EXTERNAL MODULE PIN START/STOP GAME
// ============================================================================

void setExternalModulePin(int pin) {
  pinMode(pin, INPUT);
}

int readExternalModulePin(int pin) {
  return digitalRead(pin);
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void setupMotorPins() {
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

  pinMode(DRIBBLER_PIN, OUTPUT);
  pinMode(KICKER_PIN, OUTPUT);

  analogWrite(DRIBBLER_PIN, 0);
  digitalWrite(KICKER_PIN, LOW);
}

void setMotorByNumber(int motorNum, int dir, int pwm) {
  Serial.println("motor moved");
  pwm = constrain(pwm, 0, 255);

  int speed = 0;

  if (dir > 0) {
    speed = pwm;
  } else if (dir < 0) {
    speed = -pwm;
  } else {
    speed = 0;
  }

  if (motorNum == 1) {
    setMotor(ENG1_DR1, ENG1_DR2, ENG1_SP, speed);
  } else if (motorNum == 2) {
    setMotor(ENG2_DR1, ENG2_DR2, ENG2_SP, speed);
  } else if (motorNum == 3) {
    setMotor(ENG3_DR1, ENG3_DR2, ENG3_SP, speed);
  } else if (motorNum == 4) {
    setMotor(ENG4_DR1, ENG4_DR2, ENG4_SP, speed);
  } else {
    logLine("ERROR: Invalid motor number: " + String(motorNum));
    return;
  }

  lastMotorSpeed[motorNum] = speed;
}

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

void stopAllMotors() {
  setMotorByNumber(1, 0, 0);
  setMotorByNumber(2, 0, 0);
  setMotorByNumber(3, 0, 0);
  setMotorByNumber(4, 0, 0);
}

// ============================================================================
// DRIBBLER / KICKER CONTROL
// ============================================================================

void setDribblerPower(int power) {
  power = constrain(power, 0, 100);
  dribblerPower = power;

  int pwm = map(power, 0, 100, 0, DRIBBLER_SPEED);
  analogWrite(DRIBBLER_PIN, pwm);
}

bool kick() {
  uint32_t now = millis();

  if (now - lastKickMs < KICKER_COOLDOWN_MS) {
    logLine("ACTION: KICK blocked by cooldown");
    return false;
  }

  lastKickMs = now;

  digitalWrite(KICKER_PIN, HIGH);
  delay(KICKER_PULSE_MS);
  digitalWrite(KICKER_PIN, LOW);

  logLine("ACTION: KICK pulse sent");
  return true;
}

// ============================================================================
// IR SYSTEM
// ============================================================================

void setupIRPins() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG, INPUT_PULLUP);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(DIRECT_PINS[i], INPUT_PULLUP);
  }
}

inline void setMuxChannel(uint8_t ch) {
  digitalWriteFast(MUX_S0, (ch >> 0) & 1);
  digitalWriteFast(MUX_S1, (ch >> 1) & 1);
  digitalWriteFast(MUX_S2, (ch >> 2) & 1);
  digitalWriteFast(MUX_S3, (ch >> 3) & 1);
}

inline bool waitForBurst(uint8_t pin, uint32_t timeoutUs) {
  uint32_t deadline = micros() + timeoutUs;

  while ((int32_t)(micros() - deadline) < 0) {
    // IR receivers normally idle HIGH and pulse LOW when they see the ball.
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

  for (uint8_t i = 0; i < 4; i++) {
    results[NUM_MUX_CH + i] = waitForBurst(DIRECT_PINS[i], 200);
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

  // Ball is centered in dribbler zone.
  hasBall = irSensors[0] && irSensors[1] && ballCount >= 6;
}

void sendIRData() {
  String msg = "IR:";

  for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) {
    if (i > 0) msg += ",";
    msg += irSensors[i] ? "1" : "0";
  }

  msg += ",ANGLE:";
  msg += String(ballAngle, 1);
  msg += ",COUNT:";
  msg += String(ballCount);
  msg += ",HAS:";
  msg += hasBall ? "1" : "0";
  msg += ",MS:";
  msg += String(millis());

  sendToEsp(msg);
}

// ============================================================================
// GYRO
// ============================================================================

void setupGyro() {
  pinMode(BNO_RST_PIN, OUTPUT);

  // Hardware reset for the BNO055.
  digitalWrite(BNO_RST_PIN, LOW);
  delay(20);
  digitalWrite(BNO_RST_PIN, HIGH);
  delay(700);

  Wire1.begin();

  if (!bno.begin()) {
    gyroOK = false;
    gyroHeading = 0.0f;
    targetHeading = 0.0f;

    logLine("ERROR: BNO055 gyro not detected on Wire1 / address A");
    sendGyroData();
    return;
  }

  delay(100);
  bno.setExtCrystalUse(true);

  gyroOK = true;
  doGyroRead();
  targetHeading = gyroHeading;

  logLine("SYSTEM: BNO055 gyro online heading=" + String(gyroHeading, 1));
  sendGyroData();
}

float normalizeAngle(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;

  return a;
}

float normalizeHeading360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;

  return a;
}

float headingError() {
  // Positive value means current heading is clockwise/right from target.
  // Negative value means current heading is counter-clockwise/left from target.
  return normalizeAngle(gyroHeading - targetHeading);
}

void doGyroRead() {
  if (!gyroOK) return;

  sensors_event_t e;
  bno.getEvent(&e);

  gyroHeading = normalizeHeading360(e.orientation.x);
}

void zeroGyroTarget() {
  if (!gyroOK) {
    targetHeading = 0.0f;
    return;
  }

  targetHeading = gyroHeading;
}

void setGyroTarget(float heading) {
  targetHeading = normalizeHeading360(heading);
}

String buildGyroDataMessage() {
  if (!gyroOK) {
    return "GYRO:OK:0,ERR:NO_SENSOR,MS:" + String(millis());
  }

  uint8_t sys = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;

  bno.getCalibration(&sys, &gyro, &accel, &mag);

  String msg = "GYRO:";
  msg += "OK:1";
  msg += ",HEADING:";
  msg += String(gyroHeading, 1);
  msg += ",TARGET:";
  msg += String(targetHeading, 1);
  msg += ",ERROR:";
  msg += String(headingError(), 1);
  msg += ",CAL:";
  msg += String(sys);
  msg += ",";
  msg += String(gyro);
  msg += ",";
  msg += String(accel);
  msg += ",";
  msg += String(mag);
  msg += ",MS:";
  msg += String(millis());

  return msg;
}

void printGyroToSerialAndLog() {
  String msg = buildGyroDataMessage();

  // Human-readable Serial Monitor line + LOG line for the ESP/app log.txt.
  logLine("GYRO DATA: " + msg);
}

void sendGyroData() {
  String msg = buildGyroDataMessage();

  // Send the machine-readable packet to the ESP/app.
  sendToEsp(msg);

  // Also mirror the same gyro packet to Serial Monitor and to LOG for log.txt.
  logLine("GYRO DATA: " + msg);
}

// ============================================================================
// LOGGING
// ============================================================================

void logLine(const String& msg) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] ");
  Serial.println(msg);

  // Also send to ESP/app log area if the ESP forwards LOG lines.
  ESP_SERIAL.print("LOG:");
  ESP_SERIAL.println(msg);
}

void logReceived(const String& msg) {
  logLine("RX from ESP: " + msg);
}

void logSent(const String& msg) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] ");
  Serial.print("TX to ESP: ");
  Serial.println(msg);
}

void sendToEsp(const String& msg) {
  ESP_SERIAL.println(msg);
  logSent(msg);
}
