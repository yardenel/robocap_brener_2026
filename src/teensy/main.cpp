#include <Arduino.h>

#define ESP_SERIAL Serial1

String espLine = "";

void readEsp();
void handleEspCommand(String cmd);
void printParsedCommand(String cmd);

void setup() {
  Serial.begin(115200);      // USB Serial Monitor
  ESP_SERIAL.begin(115200);  // ESP <-> Teensy UART

  delay(1000);

  Serial.println("Teensy command reader started");
  Serial.println("Click buttons in the app. Commands should appear here.");
}

void loop() {
  readEsp();
}

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
  Serial.println();
  Serial.println("========== COMMAND FROM ESP ==========");
  Serial.print("Raw command: ");
  Serial.println(cmd);

  printParsedCommand(cmd);

  Serial.println("======================================");

  // Reply to ESP so it knows Teensy got it
  ESP_SERIAL.println("ACK:" + cmd);
}

void printParsedCommand(String cmd) {
  // Emergency stop
  if (cmd == "ESTOP" || cmd == "estop") {
    Serial.println("Action: emergency stop");
    return;
  }

  // Normal stop
  if (cmd == "STOP" || cmd == "stop") {
    Serial.println("Action: stop all motors");
    return;
  }

  // Motor command, expected like:
  // MOTOR:1:1:58
  if (cmd.startsWith("MOTOR:")) {
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    int thirdColon = cmd.indexOf(':', secondColon + 1);

    if (firstColon > 0 && secondColon > 0 && thirdColon > 0) {
      int motorNum = cmd.substring(firstColon + 1, secondColon).toInt();
      int dir = cmd.substring(secondColon + 1, thirdColon).toInt();
      int pwm = cmd.substring(thirdColon + 1).toInt();

      Serial.println("Action: motor command");
      Serial.print("Motor number: ");
      Serial.println(motorNum);
      Serial.print("Direction: ");
      Serial.println(dir);
      Serial.print("PWM: ");
      Serial.println(pwm);
    } else {
      Serial.println("Action: motor command, but format was unexpected");
    }

    return;
  }

  // Kick command, expected like:
  // KICK:70
  if (cmd.startsWith("KICK:")) {
    int power = cmd.substring(5).toInt();

    Serial.println("Action: kick");
    Serial.print("Power: ");
    Serial.println(power);

    return;
  }

  // Dribbler command, expected like:
  // DRIBBLER:1
  // DRIBBLER:0
  if (cmd.startsWith("DRIBBLER:")) {
    int on = cmd.substring(9).toInt();

    Serial.println("Action: dribbler");
    Serial.print("On: ");
    Serial.println(on);

    return;
  }

  // IR request
  if (cmd == "IR:RAW") {
    Serial.println("Action: app requested IR values");

    // Fake reply
    ESP_SERIAL.println("IR:120,180,260,310,420,390,240,150,100,90,80,70,60,50,40,35,45,55,65,75,0");

    return;
  }

  // Compass request
  if (cmd == "COMPASS:READ") {
    Serial.println("Action: app requested compass heading");

    // Fake reply
    ESP_SERIAL.println("CMP:123,1");

    return;
  }

  // Goal lock
  if (cmd.startsWith("GOAL_LOCK:")) {
    String color = cmd.substring(10);

    Serial.println("Action: goal lock");
    Serial.print("Color: ");
    Serial.println(color);

    return;
  }

  // Unknown
  Serial.println("Action: unknown command");
}