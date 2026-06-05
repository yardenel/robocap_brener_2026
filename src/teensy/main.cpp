#include <Arduino.h>

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

// Change these when you know the real dribbler pins
#define DRIBBLER_DR1 30
#define DRIBBLER_DR2 31
#define DRIBBLER_SP  32

#define ESP_SERIAL Serial1

// ============================================================================
// GLOBALS
// ============================================================================

String espLine = "";

int lastMotorSpeed[5] = {0, 0, 0, 0, 0}; // indexes 1..4
int dribblerPower = 0;

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

void logLine(const String& msg);
void logReceived(const String& msg);
void logSent(const String& msg);
void sendToEsp(const String& msg);

void sendFakeIr();
void sendFakeCompass();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);      // USB Serial Monitor
  ESP_SERIAL.begin(115200);  // ESP <-> Teensy UART

  setupMotorPins();
  stopAllMotors();
  setDribblerPower(0);

  delay(1000);

  logLine("SYSTEM: Teensy command reader started");
  logLine("SYSTEM: Waiting for commands from ESP/app");
  sendToEsp("LOG:Teensy online");
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  // readEsp();
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

    logLine("ACTION: ESTOP -> stopped all motors and dribbler");
    sendToEsp("ACK:ESTOP");
    return;
  }

  // --------------------------------------------------------------------------
  // Normal stop
  // --------------------------------------------------------------------------
  if (cmd == "STOP" || cmd == "stop") {
    stopAllMotors();
    setDribblerPower(0);

    logLine("ACTION: STOP -> stopped all motors and dribbler");
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
  // New expected format:
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
  // KICK:70
  // --------------------------------------------------------------------------
  if (cmd.startsWith("KICK:")) {
    int power = cmd.substring(5).toInt();
    power = constrain(power, 0, 100);

    logLine("ACTION: KICK power=" + String(power));

    // For now only print it. Later call your real kicker function here.
    sendToEsp("ACK:KICK");
    return;
  }

  // --------------------------------------------------------------------------
  // IR request
  // --------------------------------------------------------------------------
  if (cmd == "IR:RAW") {
    logLine("ACTION: IR raw requested");
    sendFakeIr();
    return;
  }

  // --------------------------------------------------------------------------
  // Compass request
  // --------------------------------------------------------------------------
  if (cmd == "COMPASS:READ") {
    logLine("ACTION: Compass requested");
    sendFakeCompass();
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

  pinMode(DRIBBLER_DR1, OUTPUT);
  pinMode(DRIBBLER_DR2, OUTPUT);
  pinMode(DRIBBLER_SP, OUTPUT);
}

void setMotorByNumber(int motorNum, int dir, int pwm) {
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
// DRIBBLER CONTROL
// ============================================================================

void setDribblerPower(int power) {
  power = constrain(power, 0, 100);
  dribblerPower = power;

  int pwm = map(power, 0, 100, 0, 255);

  if (pwm > 0) {
    digitalWrite(DRIBBLER_DR1, HIGH);
    digitalWrite(DRIBBLER_DR2, LOW);
    analogWrite(DRIBBLER_SP, pwm);
  } else {
    digitalWrite(DRIBBLER_DR1, LOW);
    digitalWrite(DRIBBLER_DR2, LOW);
    analogWrite(DRIBBLER_SP, 0);
  }
}

// ============================================================================
// FAKE SENSOR RESPONSES
// ============================================================================

void sendFakeIr() {
  String msg = "IR:120,180,260,310,420,390,240,150,100,90,80,70,60,50,40,35,45,55,65,75,0";
  sendToEsp(msg);
}

void sendFakeCompass() {
  String msg = "CMP:123,1";
  sendToEsp(msg);
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
