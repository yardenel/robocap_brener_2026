// ============================================================================
// RoboCap UART scope test - TEENSY side
//
// Purpose:
//   Simple bench-only test for Teensy <-> ESP UART.
//   Teensy sends a counter to ESP, ESP should answer ACK lines back.
//
// Wiring expected for this test:
//   Teensy Serial1 TX1 pin 1  -> ESP Serial0 RX
//   Teensy Serial1 RX1 pin 0  <- ESP Serial0 TX
//   GND                       -> GND
//
// Scope / logic analyzer:
//   Probe Teensy TX1, Teensy RX1, or the optional pulse pins below.
//   UART decode: 115200 baud, 8 data bits, no parity, 1 stop bit.
// ============================================================================

#include <Arduino.h>

static HardwareSerial &ESP_LINK = Serial1;

static constexpr uint32_t USB_BAUD  = 115200;
static constexpr uint32_t LINK_BAUD = 115200;
static constexpr uint32_t SEND_PERIOD_MS = 200;

// Optional square pulses for a scope. You can ignore these and probe TX/RX only.
static constexpr uint8_t SCOPE_TX_PULSE_PIN = 2;
static constexpr uint8_t SCOPE_RX_PULSE_PIN = 3;
static constexpr uint8_t HEARTBEAT_PIN = LED_BUILTIN;

static uint32_t seq = 0;
static uint32_t lastSendMs = 0;
static uint32_t lastHeartbeatMs = 0;
static char rxLine[120];
static uint8_t rxLen = 0;

static void pulsePin(uint8_t pin) {
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

static const char *findFramePrefix(char *line) {
  // Because the test also sends raw 0x55/0xAA scope bytes, the line can contain
  // junk before the readable frame. Search for the real text prefix.
  for (char *p = line; *p; ++p) {
    if (startsWith(p, "E2T:")) return p;
  }
  return nullptr;
}

static void sendTestFrame() {
  seq++;

  // 0x55 = 01010101 is nice on a scope/logic analyzer.
  // 0xAA is the opposite pattern. Then comes a readable ASCII line.
  ESP_LINK.write((uint8_t)0x55);
  ESP_LINK.write((uint8_t)0xAA);
  ESP_LINK.printf("T2E:%lu:%lu\n", (unsigned long)seq, (unsigned long)millis());
  ESP_LINK.flush();

  pulsePin(SCOPE_TX_PULSE_PIN);
  Serial.printf("[TEENSY TX] seq=%lu ms=%lu\n", (unsigned long)seq, (unsigned long)millis());
}

static void handleRxLine(char *line) {
  const char *frame = findFramePrefix(line);
  if (frame) {
    Serial.printf("[TEENSY RX] %s\n", frame);
  } else {
    Serial.printf("[TEENSY RX junk/text] %s\n", line);
  }
}

static void pumpEspRx() {
  while (ESP_LINK.available() > 0) {
    uint8_t b = (uint8_t)ESP_LINK.read();
    pulsePin(SCOPE_RX_PULSE_PIN);

    if (b == '\n' || b == '\r') {
      if (rxLen > 0) {
        rxLine[rxLen] = 0;
        handleRxLine(rxLine);
        rxLen = 0;
      }
      continue;
    }

    // Keep printable ASCII only. Raw sync bytes are expected and ignored here.
    if (b >= 0x20 && b <= 0x7E) {
      if (rxLen < sizeof(rxLine) - 1) rxLine[rxLen++] = (char)b;
      else rxLen = 0;
    }
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(300);

  pinMode(SCOPE_TX_PULSE_PIN, OUTPUT);
  pinMode(SCOPE_RX_PULSE_PIN, OUTPUT);
  pinMode(HEARTBEAT_PIN, OUTPUT);
  digitalWrite(SCOPE_TX_PULSE_PIN, LOW);
  digitalWrite(SCOPE_RX_PULSE_PIN, LOW);
  digitalWrite(HEARTBEAT_PIN, LOW);

  ESP_LINK.begin(LINK_BAUD);

  Serial.println();
  Serial.println("[TEENSY UART SCOPE TEST]");
  Serial.println("TX: Serial1 pin 1 -> ESP RX");
  Serial.println("RX: Serial1 pin 0 <- ESP TX");
  Serial.println("UART decode: 115200 8N1");
}

void loop() {
  pumpEspRx();

  uint32_t now = millis();
  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;
    sendTestFrame();
  }

  if (now - lastHeartbeatMs >= 500) {
    lastHeartbeatMs = now;
    digitalToggleFast(HEARTBEAT_PIN);
  }
}
