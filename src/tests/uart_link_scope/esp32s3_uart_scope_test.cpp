// RoboCap UART test - ESP32-S3 side
// Bench test only. ESP receives T2E lines from Teensy and sends E2T lines back.

#include <Arduino.h>

#define TEENSY_LINK Serial0
#define DBG Serial

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t LINK_BAUD = 115200;
static constexpr uint32_t PING_PERIOD_MS = 500;

static constexpr uint8_t TX_PULSE_PIN = 4;
static constexpr uint8_t RX_PULSE_PIN = 5;

static uint32_t ackSeq = 0;
static uint32_t pingSeq = 0;
static uint32_t lastPingMs = 0;
static char rxLine[96];
static uint8_t rxLen = 0;

static void pulse(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(50);
  digitalWrite(pin, LOW);
}

static bool startsWith(const char *s, const char *prefix) {
  while (*prefix) {
    if (*s++ != *prefix++) return false;
  }
  return true;
}

static const char *findT2E(char *line) {
  for (char *p = line; *p; ++p) {
    if (startsWith(p, "T2E:")) return p;
  }
  return nullptr;
}

static void sendAck(const char *frame) {
  ackSeq++;
  TEENSY_LINK.write((uint8_t)0xA5);
  TEENSY_LINK.write((uint8_t)0x5A);
  TEENSY_LINK.printf("E2T:ACK:%lu:%lu:%s\n",
                     (unsigned long)ackSeq,
                     (unsigned long)millis(),
                     frame);
  TEENSY_LINK.flush();
  pulse(TX_PULSE_PIN);
  DBG.printf("[ESP TX ACK] %lu\n", (unsigned long)ackSeq);
}

static void sendPing() {
  pingSeq++;
  TEENSY_LINK.write((uint8_t)0xA5);
  TEENSY_LINK.write((uint8_t)0x5A);
  TEENSY_LINK.printf("E2T:PING:%lu:%lu\n", (unsigned long)pingSeq, (unsigned long)millis());
  TEENSY_LINK.flush();
  pulse(TX_PULSE_PIN);
  DBG.printf("[ESP TX PING] %lu\n", (unsigned long)pingSeq);
}

static void handleLine(char *line) {
  const char *frame = findT2E(line);
  if (frame) {
    DBG.printf("[ESP RX] %s\n", frame);
    sendAck(frame);
  } else {
    DBG.printf("[ESP RX TEXT] %s\n", line);
  }
}

static void pumpRx() {
  while (TEENSY_LINK.available() > 0) {
    uint8_t b = (uint8_t)TEENSY_LINK.read();
    pulse(RX_PULSE_PIN);

    if (b == '\n' || b == '\r') {
      if (rxLen > 0) {
        rxLine[rxLen] = 0;
        handleLine(rxLine);
        rxLen = 0;
      }
      continue;
    }

    if (b >= 0x20 && b <= 0x7E) {
      if (rxLen < sizeof(rxLine) - 1) rxLine[rxLen++] = (char)b;
      else rxLen = 0;
    }
  }
}

void setup() {
  DBG.begin(USB_BAUD);
  delay(300);

  pinMode(TX_PULSE_PIN, OUTPUT);
  pinMode(RX_PULSE_PIN, OUTPUT);
  digitalWrite(TX_PULSE_PIN, LOW);
  digitalWrite(RX_PULSE_PIN, LOW);

  TEENSY_LINK.begin(LINK_BAUD);

  DBG.println();
  DBG.println("[ESP32-S3 UART TEST]");
  DBG.println("Serial0 link baud 115200");
}

void loop() {
  pumpRx();

  uint32_t now = millis();
  if (now - lastPingMs >= PING_PERIOD_MS) {
    lastPingMs = now;
    sendPing();
  }
}
