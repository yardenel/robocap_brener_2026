// ============================================================================
// RoboCap 2026 - XIAO ESP32-S3 Sense CAMERA + BASIC TEST WEB CONSOLE
// Detects: orange ball, yellow goal, blue goal, white line.
// Sends compact packets to Teensy:
//   [ESP_ID][OBJ_*][angle int8 camera-local][distance 0..255]
// Forward ESP also opens a small TEST AP and forwards ASCII test commands.
// No gyro. No IR.
// ============================================================================

#include <Arduino.h>
#include "esp_camera.h"
#include "robot_protocol.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#ifndef OBJ_BALL
#define OBJ_BALL 0x05
#endif

#define TEENSY Serial0
#define DBG    Serial

// ============================================================================
// XIAO ESP32-S3 Sense camera pins
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

#define GPIO_STRAP_A 1
#define GPIO_STRAP_B 2

#define FRAME_W 320
#define FRAME_H 240
#define CAM_FOV_DEG 120.0f
#define TICK_MS 60

// ============================================================================
// HSV defaults - tune at field lighting
// H is 0..179, S/V are 0..255.
// ============================================================================
struct HSVRange {
  uint8_t hMin, hMax, sMin, sMax, vMin, vMax;
};

HSVRange BALL   = {  3,  24,  80, 255,  55, 255};
HSVRange YELLOW = { 18,  38, 100, 255,  75, 255};
HSVRange BLUE   = { 92, 135,  70, 255,  45, 255};

static constexpr uint8_t WHITE_S_MAX = 55;
static constexpr uint8_t WHITE_V_MIN = 175;

static constexpr long MIN_BALL_PIX   = 45;
static constexpr long MIN_GOAL_PIX   = 180;
static constexpr long MIN_WHITE_PIX  = 90;

uint8_t ESP_UNIQUE_ID = ESP_ID_FRONT;
int mountAngle = 0;
uint32_t lastTick = 0;
bool camOK = false;

AsyncWebServer server(TEST_HTTP_PORT);
bool apStarted = false;
String apName;
String lastTeensyLine = "no telemetry yet";
char teensyAscii[180] = {0};
uint8_t teensyAsciiLen = 0;

// ============================================================================
// Helpers
// ============================================================================
int readMountAngle() {
  pinMode(GPIO_STRAP_A, INPUT_PULLUP);
  pinMode(GPIO_STRAP_B, INPUT_PULLUP);
  delay(10);
  uint8_t a = !digitalRead(GPIO_STRAP_A);
  uint8_t b = !digitalRead(GPIO_STRAP_B);
  pinMode(GPIO_STRAP_A, INPUT);
  pinMode(GPIO_STRAP_B, INPUT);

  uint8_t code = (b << 1) | a;
  return (int)code * 90;
}

uint8_t idFromMount(int m) {
  if (m == 0) return ESP_ID_FRONT;
  if (m == 90) return ESP_ID_RIGHT;
  if (m == 180) return ESP_ID_REAR;
  return ESP_ID_LEFT;
}

int8_t pixelToAngle(int cx) {
  float norm = ((float)cx - (FRAME_W / 2.0f)) / (FRAME_W / 2.0f);
  int angle = (int)(norm * (CAM_FOV_DEG / 2.0f));
  return (int8_t)constrain(angle, -60, 60);
}

uint8_t blobToDistance(long count, long maxCount) {
  long v = (count * 255L) / maxCount;
  return (uint8_t)constrain(v, 1, 255);
}

void rgb565ToHSV(uint16_t px, uint8_t &h, uint8_t &s, uint8_t &v) {
  uint8_t r = ((px >> 11) & 0x1F) << 3;
  uint8_t g = ((px >> 5)  & 0x3F) << 2;
  uint8_t b = ((px >> 0)  & 0x1F) << 3;

  uint8_t maxC = max(r, max(g, b));
  uint8_t minC = min(r, min(g, b));
  int delta = (int)maxC - (int)minC;

  v = maxC;
  s = (maxC == 0) ? 0 : (uint8_t)((long)delta * 255L / maxC);

  if (delta == 0) {
    h = 0;
    return;
  }

  int hue;
  if (maxC == r) hue = 60 * ((int)g - (int)b) / delta;
  else if (maxC == g) hue = 60 * ((int)b - (int)r) / delta + 120;
  else hue = 60 * ((int)r - (int)g) / delta + 240;

  if (hue < 0) hue += 360;
  h = (uint8_t)(hue / 2);
}

bool hsvInRange(uint8_t h, uint8_t s, uint8_t v, const HSVRange &r) {
  bool hOK = (r.hMin <= r.hMax) ? (h >= r.hMin && h <= r.hMax)
                                : (h >= r.hMin || h <= r.hMax);
  return hOK && s >= r.sMin && s <= r.sMax && v >= r.vMin && v <= r.vMax;
}

struct Detection {
  bool detected = false;
  uint8_t objType = PACKET_NO_DETECT;
  int8_t angle = 0;
  uint8_t distance = 0;
  long pixels = 0;
};

void sendDetection(const Detection &d) {
  if (!d.detected) return;
  TEENSY.write(ESP_UNIQUE_ID);
  TEENSY.write(d.objType);
  TEENSY.write((uint8_t)d.angle);
  TEENSY.write(d.distance);
}

void sendNoDetect() {
  TEENSY.write(ESP_UNIQUE_ID);
  TEENSY.write(PACKET_NO_DETECT);
}

// ============================================================================
// BASIC TEST WEB CONSOLE ON FORWARD ESP
// ============================================================================
String htmlPage() {
  String s;
  s += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<style>body{font-family:Arial;background:#111;color:#eee;padding:14px}button{font-size:18px;margin:5px;padding:10px}input{font-size:18px;width:92%;padding:8px}.card{background:#222;padding:12px;border-radius:12px;margin:10px 0}pre{white-space:pre-wrap}</style></head><body>";
  s += "<h2>RoboCap TEST</h2>";
  s += "<div class='card'><button onclick=cmd('TEST:ON')>TEST ON</button><button onclick=cmd('TEST:OFF')>TEST OFF</button><button onclick=cmd('ESTOP')>ESTOP</button><button onclick=cmd('QUERY:STATUS')>STATUS</button><button onclick=cmd('COLOUR:RAW')>COLOUR RAW</button></div>";
  s += "<div class='card'><h3>Dribbler</h3><button onclick=cmd('DRIBBLER:0')>OFF</button><button onclick=cmd('DRIBBLER:45')>45%</button><button onclick=cmd('DRIBBLER:75')>75%</button><button onclick=cmd('DRIBBLER:100')>100%</button></div>";
  s += "<div class='card'><h3>Omni</h3><button onclick=cmd('OMNI:0:40:0')>Forward</button><button onclick=cmd('OMNI:0:-40:0')>Back</button><button onclick=cmd('OMNI:-40:0:0')>Left</button><button onclick=cmd('OMNI:40:0:0')>Right</button><button onclick=cmd('OMNI:0:0:35')>Rotate L</button><button onclick=cmd('OMNI:0:0:-35')>Rotate R</button><button onclick=cmd('OMNI:0:0:0')>Stop</button></div>";
  s += "<div class='card'><h3>Single motor</h3><button onclick=cmd('MOTOR:1:1:60')>M1 +</button><button onclick=cmd('MOTOR:1:0:60')>M1 -</button><button onclick=cmd('MOTOR:2:1:60')>M2 +</button><button onclick=cmd('MOTOR:2:0:60')>M2 -</button><button onclick=cmd('MOTOR:3:1:60')>M3 +</button><button onclick=cmd('MOTOR:3:0:60')>M3 -</button><button onclick=cmd('MOTOR:4:1:60')>M4 +</button><button onclick=cmd('MOTOR:4:0:60')>M4 -</button></div>";
  s += "<div class='card'><h3>Custom command</h3><input id='c' value='OMNI:0:0:0'><button onclick=cmd(document.getElementById('c').value)>Send</button></div>";
  s += "<div class='card'><h3>Status</h3><pre id='st'>loading...</pre></div>";
  s += "<script>function cmd(c){fetch('/cmd?c='+encodeURIComponent(c)).then(r=>r.text()).then(t=>{document.getElementById('st').textContent=t})}setInterval(()=>fetch('/status').then(r=>r.text()).then(t=>document.getElementById('st').textContent=t),500);</script>";
  s += "</body></html>";
  return s;
}

void sendTestCommandToTeensy(const String &cmd) {
  TEENSY.print(cmd);
  TEENSY.print('\n');
  DBG.print("[TEST->Teensy] ");
  DBG.println(cmd);
}

void startTestAP() {
  if (apStarted || ESP_UNIQUE_ID != ESP_ID_FRONT) return;

  uint64_t mac = ESP.getEfuseMac();
  char suffix[8];
  snprintf(suffix, sizeof(suffix), "%04X", (uint16_t)(mac & 0xFFFF));
  apName = String(TEST_AP_PREFIX) + suffix;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName.c_str(), TEST_AP_PASSWORD, TEST_AP_CHANNEL);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/html", htmlPage());
  });

  server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("c")) {
      req->send(400, "text/plain", "missing ?c=");
      return;
    }
    String c = req->getParam("c")->value();
    sendTestCommandToTeensy(c);
    req->send(200, "text/plain", "sent: " + c + "\nlast: " + lastTeensyLine);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *req) {
    String out;
    out += "AP: " + apName + "\n";
    out += "ID: " + String(ESP_UNIQUE_ID) + " mount=" + String(mountAngle) + "\n";
    out += "camOK: " + String(camOK ? 1 : 0) + "\n";
    out += "lastTeensy: " + lastTeensyLine + "\n";
    req->send(200, "text/plain", out);
  });

  server.begin();
  apStarted = true;
  DBG.printf("[TEST] AP up: %s  ip=%s\n", apName.c_str(), WiFi.softAPIP().toString().c_str());
}

// ============================================================================
// Detection scans
// ============================================================================
Detection scanColorBlob(camera_fb_t *fb, uint8_t obj, const HSVRange &range,
                        int row0, int row1, long minPixels, long maxPixels) {
  Detection out;
  out.objType = obj;

  uint16_t *pix = (uint16_t *)fb->buf;
  long count = 0;
  long sumX = 0;

  row0 = constrain(row0, 0, fb->height - 1);
  row1 = constrain(row1, 0, fb->height - 1);

  for (int y = row0; y <= row1; y += 2) {
    for (int x = 0; x < fb->width; x += 2) {
      uint16_t px = pix[y * fb->width + x];
      px = (uint16_t)((px >> 8) | (px << 8));
      uint8_t h, s, v;
      rgb565ToHSV(px, h, s, v);

      if (hsvInRange(h, s, v, range)) {
        count += 4;
        sumX += x * 4L;
      }
    }
  }

  if (count >= minPixels) {
    out.detected = true;
    out.pixels = count;
    out.angle = pixelToAngle((int)(sumX / count));
    out.distance = blobToDistance(count, maxPixels);
  }

  return out;
}

Detection scanWhiteLine(camera_fb_t *fb) {
  Detection out;
  out.objType = OBJ_LINE_WHITE;

  uint16_t *pix = (uint16_t *)fb->buf;
  long count = 0;
  long sumX = 0;
  long sumY = 0;

  for (int y = fb->height / 2; y < fb->height; y += 2) {
    for (int x = 0; x < fb->width; x += 2) {
      uint16_t px = pix[y * fb->width + x];
      px = (uint16_t)((px >> 8) | (px << 8));
      uint8_t h, s, v;
      rgb565ToHSV(px, h, s, v);

      if (s <= WHITE_S_MAX && v >= WHITE_V_MIN) {
        count += 4;
        sumX += x * 4L;
        sumY += y * 4L;
      }
    }
  }

  if (count >= MIN_WHITE_PIX) {
    int cy = (int)(sumY / count);
    out.detected = true;
    out.pixels = count;
    out.angle = pixelToAngle((int)(sumX / count));
    int nearScore = map(constrain(cy, fb->height / 2, fb->height - 1),
                        fb->height / 2, fb->height - 1,
                        30, 255);
    out.distance = (uint8_t)nearScore;
  }

  return out;
}

void handleIncomingFromTeensy() {
  while (TEENSY.available()) {
    uint8_t b = TEENSY.read();

    if (b == CMD_QUERY_ID) {
      TEENSY.write(ESP_UNIQUE_ID);
      continue;
    }

    if (b == '\r') continue;
    if (b == '\n') {
      teensyAscii[teensyAsciiLen] = 0;
      if (teensyAsciiLen > 0) {
        lastTeensyLine = String(teensyAscii);
        DBG.print("[Teensy] ");
        DBG.println(lastTeensyLine);
      }
      teensyAsciiLen = 0;
      continue;
    }

    if (b >= 32 && b <= 126 && teensyAsciiLen < sizeof(teensyAscii) - 1) {
      teensyAscii[teensyAsciiLen++] = (char)b;
    }
  }
}

bool initCamera() {
  if (!psramFound()) {
    DBG.println("[FATAL] PSRAM not found. In PlatformIO use board_build.psram_type=opi.");
    return false;
  }

  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;
  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;
  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;
  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;
  cfg.pin_d7 = Y9_GPIO_NUM;
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
    s->set_gain_ctrl(s, 1);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
  }

  return true;
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  TEENSY.begin(UART_BAUD);
  DBG.begin(115200);
  delay(300);

  mountAngle = readMountAngle();
  ESP_UNIQUE_ID = idFromMount(mountAngle);

  DBG.printf("\n[BOOT] RoboCap ESP camera + TEST web ID=%u mount=%d\n",
             ESP_UNIQUE_ID, mountAngle);

  camOK = initCamera();
  DBG.println(camOK ? "[OK] Camera ready" : "[ERR] Camera disabled");

  if (ESP_UNIQUE_ID == ESP_ID_FRONT) {
    startTestAP();
  }
}

void loop() {
  handleIncomingFromTeensy();

  if (!camOK) {
    delay(100);
    return;
  }

  if (millis() - lastTick < TICK_MS) return;
  lastTick = millis();

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    sendNoDetect();
    return;
  }

  Detection ball = scanColorBlob(fb, OBJ_BALL, BALL, 20, fb->height - 1,
                                 MIN_BALL_PIX, 7500);
  Detection yellow = scanColorBlob(fb, OBJ_GOAL_YELLOW, YELLOW, 0, fb->height - 1,
                                   MIN_GOAL_PIX, 18000);
  Detection blue = scanColorBlob(fb, OBJ_GOAL_BLUE, BLUE, 0, fb->height - 1,
                                 MIN_GOAL_PIX, 18000);
  Detection line = scanWhiteLine(fb);

  esp_camera_fb_return(fb);

  bool sent = false;

  if (ball.detected) {
    sendDetection(ball);
    sent = true;
  }

  if (yellow.detected || blue.detected) {
    if (yellow.pixels >= blue.pixels) sendDetection(yellow);
    else sendDetection(blue);
    sent = true;
  }

  if (line.detected) {
    sendDetection(line);
    sent = true;
  }

  if (!sent) sendNoDetect();

  static uint32_t lastDbg = 0;
  if (millis() - lastDbg > 500) {
    lastDbg = millis();
    DBG.printf("ID=%u ball=%d a=%d d=%u Y=%d a=%d B=%d a=%d line=%d d=%u\n",
               ESP_UNIQUE_ID,
               ball.detected, ball.angle, ball.distance,
               yellow.detected, yellow.angle,
               blue.detected, blue.angle,
               line.detected, line.distance);
  }
}
