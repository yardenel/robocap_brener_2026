// ============================================================================
//  RoboCap - XIAO ESP32-S3 Sense Combined Firmware  [camera + TEST + ESP-NOW]
//  RoboCupJunior Soccer 2026
//
//  Missions:
//    1. CAMERA OBJECT DETECTION
//       Detects: orange ball, yellow goal, blue goal, white lines, black lines.
//    2. UART COMMUNICATION WITH TEENSY 4.1
//       Binary packets + ASCII CAL/TEST/telemetry on the same UART.
//    3. INTER-ROBOT WIFI
//       Forward ESP only: ESP-NOW bridge + TEST web console + live camera view.
//
//  This restores the old combined architecture and adds OBJ_BALL support.
// ============================================================================

#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"
#include "WiFi.h"
#include "esp_now.h"
#include "Preferences.h"
#include "esp_wifi.h"
#include <ESPAsyncWebServer.h>
#include "robot_protocol.h"
#include "webapp_html.h"

#ifndef OBJ_BALL
#define OBJ_BALL 0x05
#endif

#define TEENSY Serial0
#define DBG    Serial

// ============================================================================
// 1. PER-UNIT CONFIGURATION / STRAPS
// ============================================================================
#define GPIO_STRAP_A      1
#define GPIO_STRAP_B      2

#define WIFI_SSID         "RoboCap_Link"
#define WIFI_PASSWORD     "rcj2026!"
#define WIFI_UDP_PORT     4210

// ============================================================================
// 2. XIAO ESP32-S3 SENSE CAMERA PIN MAP
// ============================================================================
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39
#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

#define FRAME_W           320
#define FRAME_H           240
#define CAM_FOV_DEG       120.0f
#define TICK_MS           100
#define WIFI_BCAST_MS     500

// Live camera: smallest delay web preview. Uses /jpg in a tight browser loop.
#define LIVE_JPEG_QUALITY 35

// ============================================================================
// 3. DEFAULT HSV THRESHOLDS, STORED IN NVS WHEN CAL:SAVE IS USED
// ============================================================================
#define DEF_BALL_H_MIN     3
#define DEF_BALL_H_MAX    24
#define DEF_BALL_S_MIN    80
#define DEF_BALL_S_MAX   255
#define DEF_BALL_V_MIN    55
#define DEF_BALL_V_MAX   255

#define DEF_YEL_H_MIN     20
#define DEF_YEL_H_MAX     35
#define DEF_YEL_S_MIN    130
#define DEF_YEL_S_MAX    255
#define DEF_YEL_V_MIN    100
#define DEF_YEL_V_MAX    255

#define DEF_BLU_H_MIN    100
#define DEF_BLU_H_MAX    130
#define DEF_BLU_S_MIN    100
#define DEF_BLU_S_MAX    255
#define DEF_BLU_V_MIN     60
#define DEF_BLU_V_MAX    255

#define DEF_WHITE_S_MAX   50
#define DEF_WHITE_V_MIN  185
#define DEF_BLACK_S_MAX   60
#define DEF_BLACK_V_MAX   55

#define MIN_BALL_PIX      70
#define MIN_GOAL_PIX     400
#define MIN_WHITE_PIX    150
#define MIN_BLACK_PIX    100

// ============================================================================
// 4. DATA STRUCTURES / GLOBALS
// ============================================================================
struct CameraThresholds {
  uint8_t ballHMin, ballHMax, ballSMin, ballSMax, ballVMin, ballVMax;
  uint8_t yelHMin,  yelHMax,  yelSMin,  yelSMax,  yelVMin,  yelVMax;
  uint8_t bluHMin,  bluHMax,  bluSMin,  bluSMax,  bluVMin,  bluVMax;
  uint8_t whiteSMax, whiteVMin;
  uint8_t blackSMax, blackVMax;
};

struct Detection {
  bool detected = false;
  uint8_t objType = PACKET_NO_DETECT;
  int8_t angle = 0;
  uint8_t distance = 0;
  long pixels = 0;
  int cx = FRAME_W / 2;
};

struct DetectionSet {
  Detection ball;
  Detection yellow;
  Detection blue;
  Detection white;
  Detection black;
};

typedef enum { WS_IDLE, WS_SCANNING, WS_PAIRED, WS_STOPPED } WifiState;

#define ASCII_BUF_SIZE 128
struct AsciiLineBuf {
  char buf[ASCII_BUF_SIZE];
  uint8_t head = 0;
  bool lineReady = false;
  char line[ASCII_BUF_SIZE];
  void push(char c) {
    if (c == '\n') {
      buf[head] = 0;
      memcpy(line, buf, head + 1);
      lineReady = true;
      head = 0;
    } else if (c != '\r' && head < ASCII_BUF_SIZE - 1) {
      buf[head++] = c;
    }
  }
};

static Preferences prefs;
const CameraThresholds DEFAULT_THRESHOLDS = {
  DEF_BALL_H_MIN, DEF_BALL_H_MAX, DEF_BALL_S_MIN, DEF_BALL_S_MAX, DEF_BALL_V_MIN, DEF_BALL_V_MAX,
  DEF_YEL_H_MIN,  DEF_YEL_H_MAX,  DEF_YEL_S_MIN,  DEF_YEL_S_MAX,  DEF_YEL_V_MIN,  DEF_YEL_V_MAX,
  DEF_BLU_H_MIN,  DEF_BLU_H_MAX,  DEF_BLU_S_MIN,  DEF_BLU_S_MAX,  DEF_BLU_V_MIN,  DEF_BLU_V_MAX,
  DEF_WHITE_S_MAX, DEF_WHITE_V_MIN,
  DEF_BLACK_S_MAX, DEF_BLACK_V_MAX
};
static CameraThresholds activeThr;

static int mountAngle = 0;
static uint8_t ESP_UNIQUE_ID = ESP_ID_FRONT;
static uint32_t lastTick = 0;
static bool camOK = false;
static int camErr = 0;

static volatile uint8_t  g_smpH = 0, g_smpS = 0, g_smpV = 0;
static volatile uint16_t g_smpBall = 0, g_smpY = 0, g_smpB = 0;

static DetectionSet g_lastDet;
static uint32_t g_lastDetMs = 0;

// WiFi / ESP-NOW
static WifiState wifiState = WS_IDLE;
static bool isWifiESP = false;
static uint8_t partnerID = 0x00;
static const uint8_t BROADCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint8_t partnerMAC[6] = {0};
static uint8_t g_myMAC[6] = {0};
static bool partnerPeerAdded = false;
static volatile bool g_pairingPending = false;
static uint8_t g_pendingPartnerID = 0;
static uint8_t g_pendingPartnerMAC[6] = {0};

#define RX_QUEUE_DEPTH 4
struct EspNowRxItem { uint8_t len; uint8_t data[64]; };
static volatile EspNowRxItem g_rxQueue[RX_QUEUE_DEPTH];
static volatile uint8_t g_rxHead = 0, g_rxTail = 0;

// TEST console globals
static AsyncWebServer g_http(TEST_HTTP_PORT);
static AsyncEventSource g_sse("/events");
static uint8_t g_robotState = ROBOT_STATE_READY;
static bool g_apUp = false;
static uint32_t g_lastActivity = 0;
static char g_lastTlm[180] = "TLM:0,0,0,-1,0,0,0,0,-1";
static bool g_relaySessionForA = false;

#define RELAY_Q_DEPTH 6
struct RelayQItem { uint8_t type; uint8_t len; char data[RELAY_MAX_PAYLOAD + 1]; };
static volatile RelayQItem g_relayQ[RELAY_Q_DEPTH];
static volatile uint8_t g_relayHead = 0, g_relayTail = 0;

static AsciiLineBuf asciiBuf;

// Forward declarations
bool initCamera();
int readMountAngle();
void resetCalToDefault();
bool loadCalFromNVS();
void saveCalToNVS();
void printCalStatus();
void processCalCommand(const char *line);
void handleTeensyCommands();
void initWiFi();
void wifiTask();
void testConsoleSetup();
void testConsoleLoop();
void testStartAP();
void testStopAP();
void testHandleCmd(AsyncWebServerRequest *req);
void testOnTelemetry(const char *line);
void emitToTeensy(const char *line);
void relaySend(uint8_t type, const char *payload);
void handleFrameBmp(AsyncWebServerRequest *req);
void handleJpg(AsyncWebServerRequest *req);
void handleCamPage(AsyncWebServerRequest *req);
void sampleCenterHSV(camera_fb_t *fb);
void rgb565ToHSV(uint16_t px, uint8_t &h, uint8_t &s, uint8_t &v);
int8_t pixelToAngle(int cx);
uint8_t blobToDistance(long pixelCount, long maxPixels);
DetectionSet detectObjects(camera_fb_t *fb);
void sendPackets(const DetectionSet &d);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onESPNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
#else
void onESPNowRecv(const uint8_t *mac, const uint8_t *data, int len);
#endif

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 1, 0)
void onESPNowSend(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
#else
void onESPNowSend(const uint8_t *mac_addr, esp_now_send_status_t status);
#endif

// ============================================================================
// 5. SETUP / LOOP
// ============================================================================
void setup() {
  TEENSY.begin(UART_BAUD);
  DBG.begin(115200);
  delay(300);

  pinMode(GPIO_STRAP_A, INPUT_PULLUP);
  pinMode(GPIO_STRAP_B, INPUT_PULLUP);
  delay(10);
  mountAngle = readMountAngle();
  ESP_UNIQUE_ID = (uint8_t)(mountAngle / 90) + 1;
  pinMode(GPIO_STRAP_A, INPUT);
  pinMode(GPIO_STRAP_B, INPUT);

  DBG.printf("\n[RoboCap] ID=0x%02X mount=%d deg\n", ESP_UNIQUE_ID, mountAngle);

  if (loadCalFromNVS()) DBG.println("INFO:CAL_LOADED_NVS");
  else { resetCalToDefault(); DBG.println("WARN:CAL_USING_DEFAULTS"); }

  camOK = initCamera();
  DBG.println(camOK ? "[OK] Camera ready. Combined loop starting." : "[FATAL] Camera init failed. Vision disabled.");

  if (mountAngle == 0) {
    isWifiESP = true;
    wifiState = WS_SCANNING;
    partnerID = 0x00;
    initWiFi();
    DBG.println("[WiFi] Auto-started (forward ESP, mountAngle=0)");
    testConsoleSetup();
  }

  lastTick = millis();
}

void loop() {
  handleTeensyCommands();
  if (isWifiESP) wifiTask();
  if (mountAngle == 0) testConsoleLoop();

  if ((millis() - lastTick) < TICK_MS) return;
  lastTick = millis();
  if (!camOK) return;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  DetectionSet det = detectObjects(fb);
  sampleCenterHSV(fb);
  esp_camera_fb_return(fb);

  g_lastDet = det;
  g_lastDetMs = millis();
  sendPackets(det);
}

// ============================================================================
// 6. STRAPS / CAMERA INIT
// ============================================================================
int readMountAngle() {
  delay(10);
  uint8_t a = !digitalRead(GPIO_STRAP_A);
  uint8_t b = !digitalRead(GPIO_STRAP_B);
  uint8_t code = (b << 1) | a;
  return (int)code * 90;
}

bool initCamera() {
  if (!psramFound()) {
    DBG.println("[FATAL] PSRAM not found / disabled.");
    DBG.println("Fix: board_build.psram_type=opi and -DBOARD_HAS_PSRAM");
    return false;
  }

  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM;
  cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM;
  cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM;
  cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_RGB565;
  cfg.frame_size = FRAMESIZE_QVGA;
  cfg.jpeg_quality = 12;
  cfg.fb_count = 2;
  cfg.grab_mode = CAMERA_GRAB_LATEST;
  cfg.fb_location = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&cfg);
  camErr = (int)err;
  if (err != ESP_OK) {
    DBG.printf("[FATAL] esp_camera_init failed: 0x%X\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 1);
    s->set_saturation(s, 1);
    s->set_contrast(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, (gainceiling_t)2);
    s->set_colorbar(s, 0);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
  }
  return true;
}

// ============================================================================
// 7. CALIBRATION STORAGE / PARSER
// ============================================================================
void saveCalToNVS() {
  prefs.begin("rcal", false);
  prefs.putBytes("thr", &activeThr, sizeof(activeThr));
  prefs.end();
  TEENSY.println("OK:SAVE");
}

bool loadCalFromNVS() {
  prefs.begin("rcal", true);
  size_t sz = prefs.getBytesLength("thr");
  if (sz != sizeof(activeThr)) { prefs.end(); return false; }
  prefs.getBytes("thr", &activeThr, sizeof(activeThr));
  prefs.end();
  return true;
}

void resetCalToDefault() {
  activeThr = DEFAULT_THRESHOLDS;
  TEENSY.println("OK:RESET");
}

void printCalStatus() {
  TEENSY.println("=== CAM CALIBRATION STATUS ===");
  TEENSY.printf("BALL   H:[%d-%d] S:[%d-%d] V:[%d-%d]\n", activeThr.ballHMin, activeThr.ballHMax, activeThr.ballSMin, activeThr.ballSMax, activeThr.ballVMin, activeThr.ballVMax);
  TEENSY.printf("YELLOW H:[%d-%d] S:[%d-%d] V:[%d-%d]\n", activeThr.yelHMin, activeThr.yelHMax, activeThr.yelSMin, activeThr.yelSMax, activeThr.yelVMin, activeThr.yelVMax);
  TEENSY.printf("BLUE   H:[%d-%d] S:[%d-%d] V:[%d-%d]\n", activeThr.bluHMin, activeThr.bluHMax, activeThr.bluSMin, activeThr.bluSMax, activeThr.bluVMin, activeThr.bluVMax);
  TEENSY.printf("WHITE  S_max:%d V_min:%d\n", activeThr.whiteSMax, activeThr.whiteVMin);
  TEENSY.printf("BLACK  S_max:%d V_max:%d\n", activeThr.blackSMax, activeThr.blackVMax);
  TEENSY.println("==============================");
}

bool parseHSV6(const char *s, uint8_t *out) {
  char tmp[70];
  strncpy(tmp, s, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = 0;
  uint8_t idx = 0;
  char *tok = strtok(tmp, ",");
  while (tok && idx < 6) {
    int v = atoi(tok);
    if (v < 0 || v > 255) return false;
    out[idx++] = (uint8_t)v;
    tok = strtok(nullptr, ",");
  }
  return idx == 6;
}

bool parseUint8_2(const char *s, uint8_t *a, uint8_t *b) {
  char tmp[32];
  strncpy(tmp, s, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = 0;
  char *tok = strtok(tmp, ",");
  if (!tok) return false;
  *a = (uint8_t)atoi(tok);
  tok = strtok(nullptr, ",");
  if (!tok) return false;
  *b = (uint8_t)atoi(tok);
  return true;
}

void processCalCommand(const char *rawLine) {
  if (strncmp(rawLine, "CAL:", 4) != 0) return;
  const char *cmd = rawLine + 4;

  if (strcmp(cmd, "SAVE") == 0) { saveCalToNVS(); return; }
  if (strcmp(cmd, "LOAD") == 0) { if (loadCalFromNVS()) { TEENSY.println("OK:LOAD"); printCalStatus(); } else TEENSY.println("ERR:NO_NVS_DATA"); return; }
  if (strcmp(cmd, "RESET") == 0) { resetCalToDefault(); return; }
  if (strcmp(cmd, "STATUS") == 0) { printCalStatus(); return; }

  uint8_t v[6];
  if (strncmp(cmd, "BALL:", 5) == 0) {
    if (parseHSV6(cmd + 5, v)) {
      activeThr.ballHMin=v[0]; activeThr.ballHMax=v[1]; activeThr.ballSMin=v[2]; activeThr.ballSMax=v[3]; activeThr.ballVMin=v[4]; activeThr.ballVMax=v[5];
      TEENSY.println("OK:CAL:BALL");
    } else TEENSY.println("ERR:PARSE_BALL");
    return;
  }
  if (strncmp(cmd, "YELLOW:", 7) == 0) {
    if (parseHSV6(cmd + 7, v)) {
      activeThr.yelHMin=v[0]; activeThr.yelHMax=v[1]; activeThr.yelSMin=v[2]; activeThr.yelSMax=v[3]; activeThr.yelVMin=v[4]; activeThr.yelVMax=v[5];
      TEENSY.println("OK:CAL:YELLOW");
    } else TEENSY.println("ERR:PARSE_YELLOW");
    return;
  }
  if (strncmp(cmd, "BLUE:", 5) == 0) {
    if (parseHSV6(cmd + 5, v)) {
      activeThr.bluHMin=v[0]; activeThr.bluHMax=v[1]; activeThr.bluSMin=v[2]; activeThr.bluSMax=v[3]; activeThr.bluVMin=v[4]; activeThr.bluVMax=v[5];
      TEENSY.println("OK:CAL:BLUE");
    } else TEENSY.println("ERR:PARSE_BLUE");
    return;
  }
  if (strncmp(cmd, "WHITE:", 6) == 0) {
    uint8_t a,b;
    if (parseUint8_2(cmd + 6, &a, &b)) { activeThr.whiteSMax=a; activeThr.whiteVMin=b; TEENSY.println("OK:CAL:WHITE"); }
    else TEENSY.println("ERR:PARSE_WHITE");
    return;
  }
  if (strncmp(cmd, "BLACK:", 6) == 0) {
    uint8_t a,b;
    if (parseUint8_2(cmd + 6, &a, &b)) { activeThr.blackSMax=a; activeThr.blackVMax=b; TEENSY.println("OK:CAL:BLACK"); }
    else TEENSY.println("ERR:PARSE_BLACK");
    return;
  }
  TEENSY.println("ERR:UNKNOWN_CAL_CMD");
}

// ============================================================================
// 8. TEENSY UART COMMAND HANDLER
// ============================================================================
uint8_t partnerRankByte() {
  if (partnerID == 0x00 || !partnerPeerAdded) return 0x00;
  return (memcmp(g_myMAC, partnerMAC, 6) > 0) ? 1 : 2;
}

void handleTeensyCommands() {
  while (TEENSY.available() > 0) {
    uint8_t b = (uint8_t)TEENSY.peek();

    if (b >= 0x80) {
      TEENSY.read();
      switch (b) {
        case CMD_ROBOT_STATE: {
          uint32_t t0 = micros();
          while (!TEENSY.available() && micros() - t0 < 2000) {}
          if (TEENSY.available()) g_robotState = TEENSY.read();
          break;
        }
        case CMD_QUERY_ID:
          TEENSY.write((uint8_t)ESP_UNIQUE_ID);
          break;
        case CMD_WIFI_START:
          if (!isWifiESP) { isWifiESP = true; wifiState = WS_SCANNING; partnerID = 0x00; initWiFi(); }
          break;
        case CMD_WIFI_STOP:
          wifiState = WS_STOPPED; isWifiESP = false; partnerID = 0x00; partnerPeerAdded = false; memset(partnerMAC,0,6); esp_now_deinit(); DBG.println("INFO:ESPNOW_STOPPED");
          break;
        case CMD_WIFI_STATUS:
          TEENSY.write(partnerRankByte());
          break;
        case CMD_RELAY_DATA: {
          uint32_t t0 = millis();
          while (!TEENSY.available() && millis() - t0 < 5) {}
          if (!TEENSY.available()) break;
          uint8_t len = TEENSY.read();
          if (len == 0 || len > 64) break;
          uint8_t payload[64];
          uint8_t got = 0;
          t0 = millis();
          while (got < len && millis() - t0 < 10) if (TEENSY.available()) payload[got++] = TEENSY.read();
          if (got == len && wifiState == WS_PAIRED && partnerPeerAdded) esp_now_send(partnerMAC, payload, len);
          break;
        }
        default: break;
      }
    } else if (b >= 0x20 || b == '\r' || b == '\n') {
      char c = (char)TEENSY.read();
      asciiBuf.push(c);
      if (asciiBuf.lineReady) {
        asciiBuf.lineReady = false;
        if (strncmp(asciiBuf.line, "CAL:", 4) == 0) processCalCommand(asciiBuf.line);
        else testOnTelemetry(asciiBuf.line);
      }
    } else {
      TEENSY.read();
    }
  }
}

// ============================================================================
// 9. DETECTION
// ============================================================================
void rgb565ToHSV(uint16_t px, uint8_t &h, uint8_t &s, uint8_t &v) {
  uint8_t r = ((px >> 11) & 0x1F) << 3;
  uint8_t g = ((px >> 5) & 0x3F) << 2;
  uint8_t b = (px & 0x1F) << 3;
  uint8_t maxC = max(r, max(g, b));
  uint8_t minC = min(r, min(g, b));
  int delta = (int)maxC - (int)minC;
  v = maxC;
  s = (maxC == 0) ? 0 : (uint8_t)((long)delta * 255L / maxC);
  if (delta == 0) { h = 0; return; }
  int hue;
  if (maxC == r) hue = 60 * ((int)g - (int)b) / delta;
  else if (maxC == g) hue = 60 * ((int)b - (int)r) / delta + 120;
  else hue = 60 * ((int)r - (int)g) / delta + 240;
  if (hue < 0) hue += 360;
  h = (uint8_t)(hue / 2);
}

static inline bool hInRange(uint8_t h, uint8_t mn, uint8_t mx) {
  return (mn <= mx) ? (h >= mn && h <= mx) : (h >= mn || h <= mx);
}

int8_t pixelToAngle(int cx) {
  float norm = (float)(cx - FRAME_W / 2) / (float)(FRAME_W / 2);
  float angle = norm * (CAM_FOV_DEG / 2.0f);
  return (int8_t)constrain((int)angle, -60, 60);
}

uint8_t blobToDistance(long pixelCount, long maxPixels) {
  return (uint8_t)(constrain(pixelCount, 0L, maxPixels) * 255L / maxPixels);
}

Detection scanHSVBlob(camera_fb_t *fb, uint8_t objType,
                      uint8_t hMin, uint8_t hMax, uint8_t sMin, uint8_t sMax, uint8_t vMin, uint8_t vMax,
                      int row0, int row1, long minPix, long maxPix) {
  Detection res;
  res.objType = objType;
  uint16_t *pix = (uint16_t*)fb->buf;
  const int W = fb->width, H = fb->height;
  row0 = constrain(row0, 0, H - 1);
  row1 = constrain(row1, 0, H - 1);
  long sumX = 0, count = 0;

  for (int row = row0; row <= row1; row += 2) {
    for (int col = 0; col < W; col += 2) {
      uint16_t px = pix[row * W + col];
      px = (uint16_t)((px >> 8) | (px << 8));
      uint8_t h,s,v; rgb565ToHSV(px,h,s,v);
      if (hInRange(h,hMin,hMax) && s >= sMin && s <= sMax && v >= vMin && v <= vMax) {
        sumX += col * 4L;
        count += 4;
      }
    }
  }

  if (count >= minPix) {
    res.detected = true;
    res.pixels = count;
    res.cx = (int)(sumX / count);
    res.angle = pixelToAngle(res.cx);
    res.distance = blobToDistance(count, maxPix);
  }
  return res;
}

Detection scanWhiteLine(camera_fb_t *fb) {
  Detection res;
  res.objType = OBJ_LINE_WHITE;
  uint16_t *pix = (uint16_t*)fb->buf;
  const int W = fb->width, H = fb->height;
  long sumX = 0, sumY = 0, count = 0;
  for (int row = H / 2; row < H; row += 2) {
    for (int col = 0; col < W; col += 2) {
      uint16_t px = pix[row * W + col];
      px = (uint16_t)((px >> 8) | (px << 8));
      uint8_t h,s,v; rgb565ToHSV(px,h,s,v);
      if (s <= activeThr.whiteSMax && v >= activeThr.whiteVMin) {
        sumX += col * 4L; sumY += row * 4L; count += 4;
      }
    }
  }
  if (count >= MIN_WHITE_PIX) {
    int cy = (int)(sumY / count);
    res.detected = true; res.pixels = count; res.cx = (int)(sumX / count); res.angle = pixelToAngle(res.cx);
    res.distance = (uint8_t)map(constrain(cy, H/2, H-1), H/2, H-1, 30, 255);
  }
  return res;
}

Detection scanBlackLine(camera_fb_t *fb) {
  Detection res;
  res.objType = OBJ_LINE_BLACK;
  uint16_t *pix = (uint16_t*)fb->buf;
  const int W = fb->width, H = fb->height;
  long sumX = 0, count = 0;
  for (int row = H / 3; row < (2 * H) / 3; row += 2) {
    for (int col = 0; col < W; col += 2) {
      uint16_t px = pix[row * W + col];
      px = (uint16_t)((px >> 8) | (px << 8));
      uint8_t h,s,v; rgb565ToHSV(px,h,s,v);
      if (s <= activeThr.blackSMax && v <= activeThr.blackVMax) { sumX += col * 4L; count += 4; }
    }
  }
  if (count >= MIN_BLACK_PIX) {
    res.detected = true; res.pixels = count; res.cx = (int)(sumX / count); res.angle = pixelToAngle(res.cx); res.distance = blobToDistance(count, 5000L);
  }
  return res;
}

DetectionSet detectObjects(camera_fb_t *fb) {
  DetectionSet d;
  d.ball = scanHSVBlob(fb, OBJ_BALL,
                       activeThr.ballHMin, activeThr.ballHMax, activeThr.ballSMin, activeThr.ballSMax, activeThr.ballVMin, activeThr.ballVMax,
                       15, fb->height - 1, MIN_BALL_PIX, 7500L);
  d.yellow = scanHSVBlob(fb, OBJ_GOAL_YELLOW,
                         activeThr.yelHMin, activeThr.yelHMax, activeThr.yelSMin, activeThr.yelSMax, activeThr.yelVMin, activeThr.yelVMax,
                         fb->height / 3, fb->height - 1, MIN_GOAL_PIX, 18000L);
  d.blue = scanHSVBlob(fb, OBJ_GOAL_BLUE,
                       activeThr.bluHMin, activeThr.bluHMax, activeThr.bluSMin, activeThr.bluSMax, activeThr.bluVMin, activeThr.bluVMax,
                       fb->height / 3, fb->height - 1, MIN_GOAL_PIX, 18000L);
  d.white = scanWhiteLine(fb);
  d.black = scanBlackLine(fb);
  g_smpBall = (uint16_t)min(d.ball.pixels, 65535L);
  g_smpY = (uint16_t)min(d.yellow.pixels, 65535L);
  g_smpB = (uint16_t)min(d.blue.pixels, 65535L);
  return d;
}

void sampleCenterHSV(camera_fb_t *fb) {
  uint16_t *pix = (uint16_t*)fb->buf;
  const int W = fb->width, H = fb->height;
  long sh=0, ss=0, sv=0, cnt=0;
  for (int row = H/3; row < 2*H/3; row += 2) {
    for (int col = W/3; col < 2*W/3; col += 2) {
      uint16_t px = pix[row * W + col];
      px = (uint16_t)((px >> 8) | (px << 8));
      uint8_t h,s,v; rgb565ToHSV(px,h,s,v);
      sh += h; ss += s; sv += v; cnt++;
    }
  }
  if (cnt) { g_smpH = sh/cnt; g_smpS = ss/cnt; g_smpV = sv/cnt; }
}

void sendOnePacket(const Detection &d) {
  if (!d.detected) return;
  TEENSY.write((uint8_t)ESP_UNIQUE_ID);
  TEENSY.write((uint8_t)d.objType);
  TEENSY.write((uint8_t)d.angle);
  TEENSY.write((uint8_t)d.distance);
}

void sendPackets(const DetectionSet &d) {
  bool sent = false;
  if (d.ball.detected) { sendOnePacket(d.ball); sent = true; }
  if (d.yellow.detected || d.blue.detected) {
    if (d.yellow.pixels >= d.blue.pixels) sendOnePacket(d.yellow);
    else sendOnePacket(d.blue);
    sent = true;
  }
  if (d.white.detected) { sendOnePacket(d.white); sent = true; }
  if (d.black.detected) { sendOnePacket(d.black); sent = true; }
  if (!sent) { TEENSY.write((uint8_t)ESP_UNIQUE_ID); TEENSY.write((uint8_t)PACKET_NO_DETECT); }
}

// ============================================================================
// 10. ESP-NOW WIFI
// ============================================================================
void initWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect();
  delay(50);
  if (esp_now_init() != ESP_OK) { DBG.println("ERR:ESPNOW_INIT_FAILED"); isWifiESP=false; wifiState=WS_IDLE; return; }
  esp_now_register_recv_cb(onESPNowRecv);
  esp_now_register_send_cb(onESPNowSend);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  partnerPeerAdded = false;
  memset(partnerMAC, 0, 6);
  partnerID = 0x00;
  WiFi.macAddress(g_myMAC);
  DBG.printf("INFO:ESPNOW_READY:%02X%02X%02X%02X%02X%02X\n", g_myMAC[0],g_myMAC[1],g_myMAC[2],g_myMAC[3],g_myMAC[4],g_myMAC[5]);
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onESPNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { const uint8_t *srcMac = info->src_addr;
#else
void onESPNowRecv(const uint8_t *mac, const uint8_t *data, int len) { const uint8_t *srcMac = mac;
#endif
  if (wifiState == WS_IDLE || wifiState == WS_STOPPED) return;
  if (len < 2 || len > 64) return;
  uint8_t myMac[6]; WiFi.macAddress(myMac);
  if (memcmp(srcMac, myMac, 6) == 0) return;

  if (len >= 3 && data[0] == RELAY_MAGIC0 && data[1] == RELAY_MAGIC1) {
    uint8_t nh = (uint8_t)((g_relayHead + 1) % RELAY_Q_DEPTH);
    if (nh != g_relayTail) {
      uint8_t pl = (uint8_t)(len - 3);
      if (pl > RELAY_MAX_PAYLOAD) pl = RELAY_MAX_PAYLOAD;
      g_relayQ[g_relayHead].type = data[2];
      memcpy((void*)g_relayQ[g_relayHead].data, data + 3, pl);
      g_relayQ[g_relayHead].data[pl] = 0;
      g_relayQ[g_relayHead].len = pl;
      g_relayHead = nh;
    }
    return;
  }

  if (wifiState == WS_SCANNING) {
    if (len >= 10 && memcmp(data, "WHO_AM_I:", 9) == 0) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, srcMac, 6);
      peer.channel = 0; peer.encrypt = false;
      esp_now_add_peer(&peer);
      char reply[16]; int rlen = snprintf(reply, sizeof(reply), "ROBOT_ID:%d", (int)ESP_UNIQUE_ID);
      esp_now_send(srcMac, (const uint8_t*)reply, rlen);
      return;
    }
    if (len >= 10 && memcmp(data, "ROBOT_ID:", 9) == 0) {
      char idStr[8] = {0};
      uint8_t copyLen = (len - 9 < 7) ? (len - 9) : 7;
      memcpy(idStr, &data[9], copyLen);
      g_pendingPartnerID = (uint8_t)atoi(idStr);
      memcpy(g_pendingPartnerMAC, srcMac, 6);
      g_pairingPending = true;
      return;
    }
    return;
  }

  if (wifiState == WS_PAIRED) {
    if (memcmp(srcMac, partnerMAC, 6) != 0) return;
    uint8_t nextHead = (g_rxHead + 1) % RX_QUEUE_DEPTH;
    if (nextHead == g_rxTail) return;
    g_rxQueue[g_rxHead].len = (uint8_t)len;
    memcpy((void*)g_rxQueue[g_rxHead].data, data, len);
    g_rxHead = nextHead;
  }
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 1, 0)
void onESPNowSend(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) { (void)tx_info; (void)status; }
#else
void onESPNowSend(const uint8_t *mac_addr, esp_now_send_status_t status) { (void)mac_addr; (void)status; }
#endif

void wifiTask() {
  if (wifiState == WS_IDLE || wifiState == WS_STOPPED) return;

  if (wifiState == WS_SCANNING && (millis() - lastTick) >= WIFI_BCAST_MS) {
    char msg[16]; int mlen = snprintf(msg, sizeof(msg), "WHO_AM_I:%d", (int)ESP_UNIQUE_ID);
    esp_now_send(BROADCAST_MAC, (const uint8_t*)msg, mlen);
  }

  static uint32_t lastBcast = 0;
  if (wifiState == WS_SCANNING && (millis() - lastBcast) >= WIFI_BCAST_MS) {
    lastBcast = millis();
    char msg[16]; int mlen = snprintf(msg, sizeof(msg), "WHO_AM_I:%d", (int)ESP_UNIQUE_ID);
    esp_now_send(BROADCAST_MAC, (const uint8_t*)msg, mlen);
  }

  if (wifiState == WS_SCANNING && g_pairingPending) {
    g_pairingPending = false;
    partnerID = g_pendingPartnerID;
    memcpy(partnerMAC, (const void*)g_pendingPartnerMAC, 6);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, partnerMAC, 6);
    peer.channel = 0; peer.encrypt = false;
    esp_now_add_peer(&peer);
    partnerPeerAdded = true;
    wifiState = WS_PAIRED;
    TEENSY.write((uint8_t)EVT_PARTNER_FOUND);
    TEENSY.write(partnerRankByte());
    DBG.printf("INFO:PAIRED_WITH:0x%02X\n", partnerID);
  }

  while (g_rxTail != g_rxHead) {
    uint8_t len = g_rxQueue[g_rxTail].len;
    TEENSY.write((uint8_t)EVT_WIFI_DATA);
    TEENSY.write(len);
    TEENSY.write((const uint8_t*)g_rxQueue[g_rxTail].data, len);
    g_rxTail = (g_rxTail + 1) % RX_QUEUE_DEPTH;
  }
}

// ============================================================================
// 11. TEST WEB CONSOLE + LIVE CAMERA
// ============================================================================
static void buildAPSSID(char *out, size_t n) {
  uint8_t m[6]; WiFi.macAddress(m);
  snprintf(out, n, "%s%02X%02X", TEST_AP_PREFIX, m[4], m[5]);
}

void testStartAP() {
  if (g_apUp) return;
  char ssid[20]; buildAPSSID(ssid, sizeof(ssid));
  bool ok = WiFi.softAP(ssid, TEST_AP_PASSWORD, TEST_AP_CHANNEL);
  esp_wifi_set_max_tx_power(60);
  g_apUp = true;
  DBG.printf("INFO:TEST_AP_UP:%s ok=%d ip=%s ch=%d\n", ssid, ok ? 1 : 0, WiFi.softAPIP().toString().c_str(), TEST_AP_CHANNEL);
}

void testStopAP() {
  if (!g_apUp) return;
  WiFi.softAPdisconnect(true);
  g_apUp = false;
  DBG.println("INFO:TEST_AP_DOWN");
}

void emitToTeensy(const char *line) {
  TEENSY.print(line);
  TEENSY.print('\n');
}

void relaySend(uint8_t type, const char *payload) {
  if (partnerID == 0x00 || !partnerPeerAdded) return;
  uint8_t buf[3 + RELAY_MAX_PAYLOAD];
  buf[0] = RELAY_MAGIC0; buf[1] = RELAY_MAGIC1; buf[2] = type;
  size_t pl = 0;
  while (pl < RELAY_MAX_PAYLOAD && payload[pl]) { buf[3 + pl] = (uint8_t)payload[pl]; pl++; }
  esp_now_send(partnerMAC, buf, 3 + pl);
}

static bool buildAsciiCmd(AsyncWebServerRequest *req, char *out, size_t n) {
  if (req->hasParam("c")) {
    String c = req->getParam("c")->value();
    snprintf(out, n, "%s", c.c_str());
    return true;
  }
  if (!req->hasParam("op")) return false;
  String op = req->getParam("op")->value();
  auto P = [&](const char *k, const char *def) -> String { return req->hasParam(k) ? req->getParam(k)->value() : String(def); };
  if      (op == "enter_test")        snprintf(out, n, "TEST:ON");
  else if (op == "exit_test")         snprintf(out, n, "TEST:OFF");
  else if (op == "enter_game")        snprintf(out, n, "GAME:ON");
  else if (op == "exit_game")         snprintf(out, n, "GAME:OFF");
  else if (op == "estop")             snprintf(out, n, "ESTOP");
  else if (op == "stop")              snprintf(out, n, "OMNI:0:0:0");
  else if (op == "motor")             snprintf(out, n, "MOTOR:%s:%s:%s", P("n","1").c_str(), P("dir","1").c_str(), P("pwm","0").c_str());
  else if (op == "omni")              snprintf(out, n, "OMNI:%s:%s:%s", P("vx","0").c_str(), P("vy","0").c_str(), P("r","0").c_str());
  else if (op == "kick")              snprintf(out, n, "KICK:%s", P("power","50").c_str());
  else if (op == "dribbler")          snprintf(out, n, "DRIBBLER:%s", P("pct","0").c_str());
  else if (op == "goal_lock")         snprintf(out, n, "GOAL_LOCK:%s", P("color","blue").c_str());
  else if (op == "ir_raw")            snprintf(out, n, "IR:RAW");
  else if (op == "color_raw")         snprintf(out, n, "COLOUR:RAW");
  else if (op == "color_dbg")         snprintf(out, n, "COLOUR:RAW");
  else if (op == "compass_read")      snprintf(out, n, "COMPASS:READ");
  else if (op == "vision_read")       snprintf(out, n, "VISION:READ");
  else if (op == "query_status")      snprintf(out, n, "QUERY:STATUS");
  else if (op == "pass_test")         snprintf(out, n, "PASS_TEST");
  else if (op == "cal")               snprintf(out, n, "CALCAM:%s:%s", P("cam","1").c_str(), P("raw","STATUS").c_str());
  else return false;
  return true;
}

void testHandleCmd(AsyncWebServerRequest *req) {
  g_lastActivity = millis();
  char line[120];
  if (!buildAsciiCmd(req, line, sizeof(line))) { req->send(400, "text/plain", "bad op"); return; }
  String tgt = req->hasParam("target") ? req->getParam("target")->value() : String("A");
  if (tgt == "B") relaySend(RELAY_TYPE_CMD, line);
  else emitToTeensy(line);
  req->send(200, "text/plain", String("ok: ") + line);
}

void testOnTelemetry(const char *line) {
  if (strncmp(line, "TLM:", 4) == 0) {
    strncpy(g_lastTlm, line, sizeof(g_lastTlm) - 1);
    g_lastTlm[sizeof(g_lastTlm) - 1] = 0;
  }
  g_sse.send(line, "tlm", millis());
  if (g_relaySessionForA) relaySend(RELAY_TYPE_RESP, line);
}

void handleJpg(AsyncWebServerRequest *req) {
  if (!camOK) {
    char msg[180];
    snprintf(msg, sizeof(msg), "camera offline\npsramFound=%d\nfreeHeap=%u\nfreePsram=%u\ncamErr=0x%X",
             psramFound() ? 1 : 0, ESP.getFreeHeap(), ESP.getFreePsram(), camErr);
    req->send(503, "text/plain", msg);
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    req->send(503, "text/plain", "no frame");
    return;
  }

  uint8_t *jpgBuf = nullptr;
  size_t jpgLen = 0;
  bool ownJpg = false;

  if (fb->format == PIXFORMAT_JPEG) {
    jpgBuf = fb->buf;
    jpgLen = fb->len;
  } else {
    ownJpg = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, fb->format,
                     LIVE_JPEG_QUALITY, &jpgBuf, &jpgLen);
  }

  if (!jpgBuf || jpgLen == 0 || (!ownJpg && fb->format != PIXFORMAT_JPEG)) {
    esp_camera_fb_return(fb);
    req->send(500, "text/plain", "jpg convert failed");
    return;
  }

  AsyncResponseStream *response = req->beginResponseStream("image/jpeg");
  response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  response->addHeader("Pragma", "no-cache");
  response->addHeader("Access-Control-Allow-Origin", "*");
  response->write(jpgBuf, jpgLen);

  if (ownJpg) free(jpgBuf);
  esp_camera_fb_return(fb);
  req->send(response);
}

void handleCamPage(AsyncWebServerRequest *req) {
  const char *page = R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboCap Ultra-Low-Latency Live</title>
<style>
body{font-family:Arial;background:#080808;color:white;text-align:center;margin:0}
h2{margin:10px}.card{background:#181818;margin:8px;padding:8px;border-radius:10px}
img{width:98vw;max-width:900px;border:2px solid #555;image-rendering:auto}
button{font-size:18px;padding:8px 12px;margin:4px;border-radius:8px}
pre{text-align:left;display:inline-block;max-width:96vw;overflow:auto}
.small{opacity:.75;font-size:13px}
</style></head><body>
<h2>RoboCap ESP Live Camera</h2>
<div class="card">
  <img id="cam" src="/jpg">
  <br>
  <button onclick="resumeLive()">LIVE</button>
  <button onclick="pauseLive()">Pause</button>
  <button onclick="singleFrame()">One frame</button>
  <div class="small">Fast mode: browser requests the next JPEG immediately after the previous one arrives.</div>
</div>
<div class="card"><pre id="s">loading detection...</pre></div>
<script>
let running=true;
let seq=0;
let retryMs=60;
const img=document.getElementById('cam');
const stat=document.getElementById('s');

function nextFrame(){
  if(!running) return;
  img.src='/jpg?seq='+(++seq)+'&t='+Date.now();
}
img.onload=function(){ if(running) setTimeout(nextFrame,0); };
img.onerror=function(){ if(running) setTimeout(nextFrame,retryMs); };

function resumeLive(){ if(!running){ running=true; nextFrame(); } }
function pauseLive(){ running=false; }
function singleFrame(){ running=false; img.src='/jpg?single='+Date.now(); }

function updateDetect(){
  fetch('/detect?t='+Date.now(), {cache:'no-store'})
    .then(r=>r.text())
    .then(t=>stat.textContent=t)
    .catch(e=>stat.textContent='detect offline');
}
setInterval(updateDetect,200);
nextFrame();
updateDetect();
</script></body></html>
)HTML";
  req->send(200, "text/html", page);
}

void drawMark(uint8_t *row, int outW, int x, uint8_t r, uint8_t g, uint8_t b) {
  if (x < 0 || x >= outW) return;
  for (int dx = -2; dx <= 2; dx++) {
    int xx = x + dx;
    if (xx < 0 || xx >= outW) continue;
    int dst = xx * 3;
    row[dst + 0] = b;
    row[dst + 1] = g;
    row[dst + 2] = r;
  }
}

int angleToOutX(int8_t a, int outW) {
  float norm = (float)a / (CAM_FOV_DEG / 2.0f);
  return constrain((int)(outW / 2 + norm * outW / 2), 0, outW - 1);
}

void handleFrameBmp(AsyncWebServerRequest *req) {
  if (!camOK) {
    char msg[180];
    snprintf(msg, sizeof(msg), "camera offline\npsramFound=%d\nfreeHeap=%u\nfreePsram=%u\ncamErr=0x%X", psramFound()?1:0, ESP.getFreeHeap(), ESP.getFreePsram(), camErr);
    req->send(503, "text/plain", msg);
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { req->send(503, "text/plain", "no frame"); return; }
  DetectionSet det = detectObjects(fb);
  g_lastDet = det;
  g_lastDetMs = millis();

  const int OUT_W = 160, OUT_H = 120;
  const int SRC_W = fb->width, SRC_H = fb->height;
  const int ROW_SIZE = (OUT_W * 3 + 3) & ~3;
  const int PIX_DATA_SIZE = ROW_SIZE * OUT_H;
  const int FILE_SIZE = 54 + PIX_DATA_SIZE;

  AsyncResponseStream *response = req->beginResponseStream("image/bmp");
  response->addHeader("Cache-Control", "no-store");
  uint8_t header[54] = {0};
  header[0]='B'; header[1]='M';
  auto put32 = [&](int pos, uint32_t v){ header[pos]=v; header[pos+1]=v>>8; header[pos+2]=v>>16; header[pos+3]=v>>24; };
  auto put16 = [&](int pos, uint16_t v){ header[pos]=v; header[pos+1]=v>>8; };
  put32(2, FILE_SIZE); put32(10,54); put32(14,40); put32(18,OUT_W); put32(22,OUT_H); put16(26,1); put16(28,24); put32(34,PIX_DATA_SIZE);
  response->write(header, sizeof(header));

  int xBall = det.ball.detected ? angleToOutX(det.ball.angle, OUT_W) : -1;
  int xYellow = det.yellow.detected ? angleToOutX(det.yellow.angle, OUT_W) : -1;
  int xBlue = det.blue.detected ? angleToOutX(det.blue.angle, OUT_W) : -1;
  int xWhite = det.white.detected ? angleToOutX(det.white.angle, OUT_W) : -1;

  uint8_t row[ROW_SIZE];
  for (int y = OUT_H - 1; y >= 0; y--) {
    memset(row, 0, sizeof(row));
    int srcY = map(y, 0, OUT_H - 1, 0, SRC_H - 1);
    for (int x = 0; x < OUT_W; x++) {
      int srcX = map(x, 0, OUT_W - 1, 0, SRC_W - 1);
      int srcIndex = (srcY * SRC_W + srcX) * 2;
      uint8_t hi = fb->buf[srcIndex];
      uint8_t lo = fb->buf[srcIndex + 1];
      uint16_t px = ((uint16_t)hi << 8) | lo;
      uint8_t r = ((px >> 11) & 0x1F) << 3;
      uint8_t g = ((px >> 5) & 0x3F) << 2;
      uint8_t b = (px & 0x1F) << 3;
      int dst = x * 3;
      row[dst + 0] = b; row[dst + 1] = g; row[dst + 2] = r;
    }
    // Vertical overlays: ball=red/orange, yellow=yellow, blue=blue, white=white.
    if (xBall >= 0) drawMark(row, OUT_W, xBall, 255, 80, 0);
    if (xYellow >= 0) drawMark(row, OUT_W, xYellow, 255, 255, 0);
    if (xBlue >= 0) drawMark(row, OUT_W, xBlue, 0, 80, 255);
    if (xWhite >= 0) drawMark(row, OUT_W, xWhite, 255, 255, 255);
    response->write(row, ROW_SIZE);
  }
  esp_camera_fb_return(fb);
  req->send(response);
}

void testConsoleSetup() {
  if (mountAngle != 0) return;

  WiFi.onEvent([](arduino_event_id_t e, arduino_event_info_t){
    if      (e == ARDUINO_EVENT_WIFI_AP_STACONNECTED)    DBG.println("INFO:AP_CLIENT_CONNECTED");
    else if (e == ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED)   DBG.println("INFO:AP_CLIENT_GOT_IP");
    else if (e == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) DBG.println("INFO:AP_CLIENT_DISCONNECTED");
  });

  g_http.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", INDEX_HTML); });
  g_http.on("/cmd", HTTP_GET, testHandleCmd);
  g_http.on("/cam", HTTP_GET, handleCamPage);
  g_http.on("/live", HTTP_GET, handleCamPage);
  g_http.on("/jpg", HTTP_GET, handleJpg);
  g_http.on("/frame.bmp", HTTP_GET, handleFrameBmp);
  g_http.on("/hsv", HTTP_GET, [](AsyncWebServerRequest *r){ char b[64]; snprintf(b,sizeof(b),"%u,%u,%u,%u,%u",g_smpH,g_smpS,g_smpV,g_smpY,g_smpB); r->send(200,"text/plain",b); });
  g_http.on("/detect", HTTP_GET, [](AsyncWebServerRequest *r){
    char b[320];
    snprintf(b, sizeof(b),
             "ID=%u mount=%d camOK=%d age=%lums\nBALL   seen=%d angle=%d dist=%u pix=%ld\nYELLOW seen=%d angle=%d dist=%u pix=%ld\nBLUE   seen=%d angle=%d dist=%u pix=%ld\nWHITE  seen=%d angle=%d dist=%u pix=%ld\nBLACK  seen=%d angle=%d dist=%u pix=%ld\nlastTlm=%s",
             ESP_UNIQUE_ID, mountAngle, camOK?1:0, millis()-g_lastDetMs,
             g_lastDet.ball.detected, g_lastDet.ball.angle, g_lastDet.ball.distance, g_lastDet.ball.pixels,
             g_lastDet.yellow.detected, g_lastDet.yellow.angle, g_lastDet.yellow.distance, g_lastDet.yellow.pixels,
             g_lastDet.blue.detected, g_lastDet.blue.angle, g_lastDet.blue.distance, g_lastDet.blue.pixels,
             g_lastDet.white.detected, g_lastDet.white.angle, g_lastDet.white.distance, g_lastDet.white.pixels,
             g_lastDet.black.detected, g_lastDet.black.angle, g_lastDet.black.distance, g_lastDet.black.pixels,
             g_lastTlm);
    r->send(200, "text/plain", b);
  });
  g_sse.onConnect([](AsyncEventSourceClient *c){ c->send(g_lastTlm, "tlm", millis()); });
  g_http.addHandler(&g_sse);
  g_http.begin();
  g_lastActivity = millis();
}

void testConsoleLoop() {
  if (mountAngle != 0) return;
  bool wantAP = (g_robotState != ROBOT_STATE_GAME);
  if (wantAP && !g_apUp) testStartAP();
  if (!wantAP && g_apUp) testStopAP();

  while (g_relayTail != g_relayHead) {
    uint8_t type = g_relayQ[g_relayTail].type;
    char buf[RELAY_MAX_PAYLOAD + 1];
    memcpy(buf, (const void*)g_relayQ[g_relayTail].data, RELAY_MAX_PAYLOAD + 1);
    g_relayTail = (uint8_t)((g_relayTail + 1) % RELAY_Q_DEPTH);
    if (type == RELAY_TYPE_CMD) { g_relaySessionForA = true; emitToTeensy(buf); }
    else g_sse.send(buf, "tlmB", millis());
  }

  if (g_robotState == ROBOT_STATE_TEST && (millis() - g_lastActivity) > TEST_IDLE_TIMEOUT_MS) {
    emitToTeensy("TEST:OFF");
    g_lastActivity = millis();
  }
}
