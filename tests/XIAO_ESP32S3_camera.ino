// XIAO_ESP32S3_RoboCup.ino
// ============================================================================
//   RoboCupJunior 2026 — Camera ESP firmware
//   Target:  Seeed XIAO ESP32-S3 Sense (OV2640)
//   Identical binary on all 4 ESPs; mount angle read from solder bridges.
//
//   Per-ESP responsibilities:
//     ID=0 (front, 0°)   — camera detection + ESP-NOW link to teammate robot
//     ID=1 (right, 90°)  — camera detection only
//     ID=2 (rear, 180°)  — camera detection only
//     ID=3 (left, 270°)  — camera detection only
//
//   Detected:
//     - Orange ball (HSV threshold + centroid)
//     - Yellow goal (HSV threshold + bounding box width)
//     - Blue goal   (HSV threshold + bounding box width)
//     - WHITE LINE  (early-warning, NOT the hard-stop path)
//
//   White line philosophy:
//     The TCS34725 sensor array on the robot's BOTTOM is the hard-stop path
//     (Teensy reads it directly, stops motors immediately on white). That
//     fires LATE — the line is already at the robot's feet. The camera-based
//     white-line detection here gives EARLY WARNING so Teensy can:
//       - kick the ball before stopping (near-goal aggressive play)
//       - decelerate gracefully
//       - choose an exit direction in advance
//     This is REPORTED to Teensy via UART, Teensy decides what to do.
//
//   Opponent recognition: INTENTIONALLY NOT IMPLEMENTED.
//
//   Coordinate convention:
//     - All angles in DEGREES * 10
//     - Robot-relative: 0 = front of robot, +CW, range -1800..1800
//     - ESP applies mount offset BEFORE sending to Teensy
// ============================================================================

#include <Arduino.h>
#include <Preferences.h>
#include "esp_camera.h"
#include "esp_wifi.h"
#include <WiFi.h>
#include <esp_now.h>
#include "robot_protocol.h"

// ----------------------------------------------------------------------------
//   Hardware pin map — XIAO ESP32-S3 Sense
// ----------------------------------------------------------------------------
// Camera (OV2640) — fixed by Sense module PCB
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

// Mount-angle solder bridges (read once at boot)
#define MOUNT_BIT0_PIN  1   // GPIO1 — bridge to GND for LOW
#define MOUNT_BIT1_PIN  2   // GPIO2 — bridge to GND for LOW

// UART to Teensy 4.1 — using Serial1 on alternate pins
#define TEENSY_UART     Serial1
#define UART_TX_PIN     43
#define UART_RX_PIN     44

// Status LED — built-in on XIAO is GPIO21
#define STATUS_LED_PIN  21

// ----------------------------------------------------------------------------
//   Frame / detection parameters
// ----------------------------------------------------------------------------
#define FRAME_W         160
#define FRAME_H         120
#define HFOV_DEG        120        // OV2640 with 120° lens

// Color threshold structure
struct ColorRange {
  uint8_t h_min, h_max;
  uint8_t s_min, s_max;
  uint8_t v_min, v_max;
};

// Factory defaults — overwritten by NVS values at boot if present
// White line is special: very low saturation, very high V, hue ignored
static const ColorRange DEFAULT_RANGES[CAL_TARGET_COUNT] = {
  /* BALL        */ {  5,  20, 130, 255, 100, 255 },
  /* YELLOW_GOAL */ { 22,  35, 120, 255, 120, 255 },
  /* BLUE_GOAL   */ { 95, 125, 110, 255,  80, 255 },
  /* WHITE_LINE  */ {  0, 255,   0,  40, 200, 255 },  // hue any, low sat, high val
  /* FIELD       */ { 35,  85,  60, 255,  40, 200 }   // greenish carpet, debug only
};

static ColorRange g_ranges[CAL_TARGET_COUNT];   // live, in-RAM ranges

// White-line distance band thresholds (row Y in frame, 0=top, FRAME_H=bottom)
// Tune these to your camera tilt — assumes camera looks slightly downward and
// the field directly in front of the robot maps to the BOTTOM of the frame.
#define LINE_ROW_NEAR_THRESH   90    // row > 90 (bottom 25% of frame) = NEAR
#define LINE_ROW_MID_THRESH    65    // row > 65 = MID
                                     // anything visible above 65 = FAR

#define MIN_BALL_PIXELS         12
#define MIN_GOAL_PIXELS         60
#define MIN_LINE_PIXELS         20    // line is thin — don't require many pixels

// Loop pacing
#define DETECT_INTERVAL_MS      33     // ~30 Hz target
#define UART_TX_INTERVAL_MS     33
#define ESPNOW_TX_INTERVAL_MS   50     // 20 Hz teammate broadcasts
#define LED_BLINK_INTERVAL_MS   500

// ESP-NOW TX power cap: 60 quarter-dBm = 15 dBm; safe under 20 dBm EIRP rule
#define TX_POWER_QDBM           60

// Calibration parameters
#define CAL_SAMPLE_DURATION_MS  1000  // sample for 1 second
#define CAL_ROI_W               40    // center 40x30 ROI
#define CAL_ROI_H               30
#define CAL_MIN_PIXELS          100   // need at least this many in ROI
#define CAL_PERCENTILE_LOW      10    // discard outliers
#define CAL_PERCENTILE_HIGH     90

// NVS namespace
#define NVS_NAMESPACE           "robocup"
#define NVS_KEY_PREFIX          "cr"   // followed by target id, e.g. "cr0", "cr1"...

// ----------------------------------------------------------------------------
//   Globals
// ----------------------------------------------------------------------------
static uint8_t   g_cam_id        = 0;
static uint16_t  g_mount_deg_x10 = 0;
static uint32_t  g_frame_seq     = 0;
static bool      g_is_master     = false;

struct Detection {
  bool      ball_seen;
  int16_t   ball_angle_x10;
  uint16_t  ball_radius_px;
  uint8_t   ball_confidence;

  bool      yellow_seen;
  int16_t   yellow_angle_x10;
  uint16_t  yellow_width_px;

  bool      blue_seen;
  int16_t   blue_angle_x10;
  uint16_t  blue_width_px;

  bool      line_seen;
  int16_t   line_angle_x10;
  uint8_t   line_dist_band;        // LINE_DIST_*
  uint8_t   line_row_y;            // raw row
};
static volatile Detection g_det = {};

// Calibration job state — non-blocking, runs across multiple detect() cycles
struct CalJob {
  bool      active;
  uint8_t   target;
  uint32_t  start_ms;
  uint16_t  h_hist[256];           // hue histogram across all sampled frames
  uint16_t  s_hist[256];
  uint16_t  v_hist[256];
  uint32_t  total_pixels;
};
static CalJob g_cal = {};

// Teensy-supplied team state (only used by ID=0)
static volatile TeamStatePayload g_team_state = { ROLE_UNKNOWN, 0, 0, 0, 100 };

// ESP-NOW peer state
static const uint8_t BROADCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint32_t g_espnow_seq = 0;

// NVS handle
static Preferences g_prefs;

// ----------------------------------------------------------------------------
//   Forward decls
// ----------------------------------------------------------------------------
static void readMountID();
static bool initCamera();
static bool initEspNow();
static void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len);
static void detectAndUpdate();
static void sendUartFrame();
static void sendEspNowFrame();
static void handleUartRx();
static uint8_t xorChecksum(const uint8_t* p, uint8_t n);
static int16_t pixelToAngleX10(int px);
static int16_t applyMountOffset(int16_t angle_x10);

static void loadRangesFromNvs();
static void saveRangesToNvs();
static void resetRangesToFactory();
static void calBegin(uint8_t target);
static void calAccumulate();
static void calFinish();
static void sendCalReport(uint8_t target, uint8_t status, const ColorRange& r, uint32_t samples);

// ============================================================================
//   setup()
// ============================================================================
void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  Serial.begin(115200);
  uint32_t serial_wait = millis();
  while (!Serial && millis() - serial_wait < 800) { }
  Serial.println("\n[BOOT] XIAO ESP32-S3 RoboCup camera ESP");

  readMountID();
  Serial.printf("[BOOT] cam_id=%u  mount=%u°  master=%s\n",
                g_cam_id, g_mount_deg_x10/10, g_is_master ? "YES" : "no");

  // Load color calibration from NVS (or factory defaults if absent)
  loadRangesFromNvs();

  TEENSY_UART.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.printf("[BOOT] Teensy UART up @ %u baud\n", UART_BAUD);

  if (!initCamera()) {
    Serial.println("[BOOT] CAMERA INIT FAILED — halting");
    while (1) { digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); delay(100); }
  }
  Serial.println("[BOOT] camera OK");

  if (g_is_master) {
    if (initEspNow()) {
      Serial.printf("[BOOT] ESP-NOW master OK, MAC=%s\n", WiFi.macAddress().c_str());
    } else {
      Serial.println("[BOOT] ESP-NOW init FAILED — continuing without teammate link");
    }
  }

  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.println("[BOOT] ready");
}

// ============================================================================
//   loop()
// ============================================================================
void loop() {
  static uint32_t t_detect = 0, t_uart = 0, t_espnow = 0, t_led = 0;
  static bool led_state = false;

  uint32_t now = millis();

  if (now - t_detect >= DETECT_INTERVAL_MS) {
    t_detect = now;
    detectAndUpdate();
    // If calibration job is running, it has finished accumulating this frame
    // inside detectAndUpdate(). Check if it's time to wrap up.
    if (g_cal.active && (now - g_cal.start_ms) >= CAL_SAMPLE_DURATION_MS) {
      calFinish();
    }
  }

  if (now - t_uart >= UART_TX_INTERVAL_MS) {
    t_uart = now;
    sendUartFrame();
  }

  handleUartRx();

  if (g_is_master && now - t_espnow >= ESPNOW_TX_INTERVAL_MS) {
    t_espnow = now;
    sendEspNowFrame();
  }

  if (now - t_led >= LED_BLINK_INTERVAL_MS) {
    t_led = now;
    led_state = !led_state;
    digitalWrite(STATUS_LED_PIN, led_state);
  }
}

// ============================================================================
//   Mount-ID readout
// ============================================================================
static void readMountID() {
  pinMode(MOUNT_BIT0_PIN, INPUT_PULLUP);
  pinMode(MOUNT_BIT1_PIN, INPUT_PULLUP);
  delay(5);
  uint8_t b0 = digitalRead(MOUNT_BIT0_PIN) == LOW ? 1 : 0;
  uint8_t b1 = digitalRead(MOUNT_BIT1_PIN) == LOW ? 1 : 0;
  g_cam_id        = b0 | (b1 << 1);
  g_mount_deg_x10 = g_cam_id * 900;
  g_is_master     = (g_cam_id == 0);
}

// ============================================================================
//   NVS / calibration persistence
// ============================================================================
static void resetRangesToFactory() {
  for (uint8_t i = 0; i < CAL_TARGET_COUNT; i++) g_ranges[i] = DEFAULT_RANGES[i];
}

static void loadRangesFromNvs() {
  resetRangesToFactory();
  if (!g_prefs.begin(NVS_NAMESPACE, true)) {     // read-only
    Serial.println("[NVS] open failed — using factory defaults");
    return;
  }
  char key[8];
  for (uint8_t i = 0; i < CAL_TARGET_COUNT; i++) {
    snprintf(key, sizeof(key), "%s%u", NVS_KEY_PREFIX, i);
    size_t sz = g_prefs.getBytesLength(key);
    if (sz == sizeof(ColorRange)) {
      g_prefs.getBytes(key, &g_ranges[i], sizeof(ColorRange));
      Serial.printf("[NVS] loaded target %u from flash\n", i);
    }
  }
  g_prefs.end();
}

static void saveRangesToNvs() {
  if (!g_prefs.begin(NVS_NAMESPACE, false)) {    // read-write
    Serial.println("[NVS] write open failed");
    return;
  }
  char key[8];
  for (uint8_t i = 0; i < CAL_TARGET_COUNT; i++) {
    snprintf(key, sizeof(key), "%s%u", NVS_KEY_PREFIX, i);
    g_prefs.putBytes(key, &g_ranges[i], sizeof(ColorRange));
  }
  g_prefs.end();
  Serial.println("[NVS] saved all ranges");
}

// ============================================================================
//   Camera init
// ============================================================================
static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_QQVGA;       // 160x120
  config.pixel_format = PIXFORMAT_RGB565;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init err 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 2);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
  }
  return true;
}

// ============================================================================
//   RGB565 → HSV
// ============================================================================
static inline void rgb565ToHsv(uint16_t px, uint8_t* h, uint8_t* s, uint8_t* v) {
  uint16_t p = (px >> 8) | (px << 8);          // byte-swap (DMA big-endian)
  uint8_t r = (p >> 11) & 0x1F; r = (r << 3) | (r >> 2);
  uint8_t g = (p >>  5) & 0x3F; g = (g << 2) | (g >> 4);
  uint8_t b =  p        & 0x1F; b = (b << 3) | (b >> 2);

  uint8_t mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
  uint8_t mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
  *v = mx;
  uint8_t d = mx - mn;
  *s = mx == 0 ? 0 : (uint16_t)d * 255 / mx;

  if (d == 0)        { *h = 0; }
  else if (mx == r)  { *h = (uint8_t)((43 * (g - b) / d) & 0xFF); }
  else if (mx == g)  { *h = (uint8_t)(85 + 43 * (b - r) / d); }
  else               { *h = (uint8_t)(171 + 43 * (r - g) / d); }
}

static inline bool pixelMatches(const ColorRange& r, uint8_t h, uint8_t s, uint8_t v) {
  return h >= r.h_min && h <= r.h_max
      && s >= r.s_min && s <= r.s_max
      && v >= r.v_min && v <= r.v_max;
}

// ============================================================================
//   Detection — single pass over frame; also feeds calibration job if active
// ============================================================================
static void detectAndUpdate() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[DET] fb NULL");
    return;
  }
  if (fb->format != PIXFORMAT_RGB565 || fb->len < (size_t)(FRAME_W * FRAME_H * 2)) {
    esp_camera_fb_return(fb);
    return;
  }

  uint16_t* pix = (uint16_t*)fb->buf;

  // Accumulators
  uint32_t bn=0, bx=0, by=0;                     // ball
  uint32_t yn=0, yx=0;                            // yellow
  uint16_t y_xmin=FRAME_W, y_xmax=0;
  uint32_t un=0, ux=0;                            // blue (u for "unique color")
  uint16_t u_xmin=FRAME_W, u_xmax=0;
  uint32_t ln=0, lx=0, ly=0;                      // white line
  uint8_t  l_row_max = 0;                          // lowest (deepest) row with line

  // Calibration ROI bounds — center of frame
  const int cal_x0 = (FRAME_W - CAL_ROI_W) / 2;
  const int cal_x1 = cal_x0 + CAL_ROI_W;
  const int cal_y0 = (FRAME_H - CAL_ROI_H) / 2;
  const int cal_y1 = cal_y0 + CAL_ROI_H;

  // Skip top 20% — camera looking outward sees ceiling/audience there
  const int Y_START = FRAME_H / 5;
  for (int y = Y_START; y < FRAME_H; y++) {
    int row = y * FRAME_W;
    for (int x = 0; x < FRAME_W; x++) {
      uint8_t h, s, v;
      rgb565ToHsv(pix[row + x], &h, &s, &v);

      // -- regular detection --
      if (pixelMatches(g_ranges[CAL_TARGET_BALL], h, s, v)) {
        bn++; bx += x; by += y;
      }
      else if (pixelMatches(g_ranges[CAL_TARGET_YELLOW_GOAL], h, s, v)) {
        yn++; yx += x;
        if (x < y_xmin) y_xmin = x;
        if (x > y_xmax) y_xmax = x;
      }
      else if (pixelMatches(g_ranges[CAL_TARGET_BLUE_GOAL], h, s, v)) {
        un++; ux += x;
        if (x < u_xmin) u_xmin = x;
        if (x > u_xmax) u_xmax = x;
      }
      else if (pixelMatches(g_ranges[CAL_TARGET_WHITE_LINE], h, s, v)) {
        ln++; lx += x; ly += y;
        if (y > l_row_max) l_row_max = y;
      }

      // -- calibration accumulation (only in center ROI) --
      if (g_cal.active && x >= cal_x0 && x < cal_x1 && y >= cal_y0 && y < cal_y1) {
        g_cal.h_hist[h]++;
        g_cal.s_hist[s]++;
        g_cal.v_hist[v]++;
        g_cal.total_pixels++;
      }
    }
  }

  esp_camera_fb_return(fb);

  // ---- Build Detection result ----
  Detection d = {};

  if (bn >= MIN_BALL_PIXELS) {
    int cx = bx / bn;
    d.ball_seen      = true;
    d.ball_angle_x10 = applyMountOffset(pixelToAngleX10(cx));
    uint32_t r2 = bn * 4 / 13;
    uint16_t r = 0; while ((uint32_t)r*r < r2) r++;
    d.ball_radius_px  = r;
    d.ball_confidence = bn > 200 ? 100 : (uint8_t)(bn * 100 / 200);
  }

  if (yn >= MIN_GOAL_PIXELS) {
    int cx = yx / yn;
    d.yellow_seen      = true;
    d.yellow_angle_x10 = applyMountOffset(pixelToAngleX10(cx));
    d.yellow_width_px  = (y_xmax > y_xmin) ? (y_xmax - y_xmin) : 0;
  }

  if (un >= MIN_GOAL_PIXELS) {
    int cx = ux / un;
    d.blue_seen        = true;
    d.blue_angle_x10   = applyMountOffset(pixelToAngleX10(cx));
    d.blue_width_px    = (u_xmax > u_xmin) ? (u_xmax - u_xmin) : 0;
  }

  // White line — use centroid x for angle, deepest row for distance
  if (ln >= MIN_LINE_PIXELS) {
    int cx = lx / ln;
    d.line_seen      = true;
    d.line_angle_x10 = applyMountOffset(pixelToAngleX10(cx));
    d.line_row_y     = l_row_max;
    if      (l_row_max > LINE_ROW_NEAR_THRESH) d.line_dist_band = LINE_DIST_NEAR;
    else if (l_row_max > LINE_ROW_MID_THRESH)  d.line_dist_band = LINE_DIST_MID;
    else                                        d.line_dist_band = LINE_DIST_FAR;
  } else {
    d.line_dist_band = LINE_DIST_NONE;
  }

  noInterrupts();
  memcpy((void*)&g_det, &d, sizeof(d));
  g_frame_seq++;
  interrupts();
}

static int16_t pixelToAngleX10(int px) {
  int32_t centered = px - FRAME_W / 2;
  return (int16_t)(centered * HFOV_DEG * 10 / FRAME_W);
}

static int16_t applyMountOffset(int16_t angle_x10) {
  int32_t a = (int32_t)angle_x10 + (int32_t)g_mount_deg_x10;
  while (a >  1800) a -= 3600;
  while (a < -1800) a += 3600;
  return (int16_t)a;
}

// ============================================================================
//   Calibration — non-blocking, accumulates over ~1 sec then computes ranges
// ============================================================================
static void calBegin(uint8_t target) {
  if (target >= CAL_TARGET_COUNT) {
    Serial.printf("[CAL] invalid target %u\n", target);
    return;
  }
  Serial.printf("[CAL] BEGIN target=%u (1 sec sample)\n", target);
  memset(&g_cal, 0, sizeof(g_cal));
  g_cal.active   = true;
  g_cal.target   = target;
  g_cal.start_ms = millis();
}

// Find the value v such that `pct`% of weighted samples are <= v
static uint8_t percentile(const uint16_t* hist, uint32_t total, uint8_t pct) {
  uint32_t target = total * pct / 100;
  uint32_t cum = 0;
  for (int i = 0; i < 256; i++) {
    cum += hist[i];
    if (cum >= target) return (uint8_t)i;
  }
  return 255;
}

static void calFinish() {
  g_cal.active = false;
  uint8_t target = g_cal.target;
  uint32_t total = g_cal.total_pixels;

  ColorRange r = {};
  uint8_t status = CAL_STATUS_OK;

  if (total < CAL_MIN_PIXELS) {
    Serial.printf("[CAL] too few pixels: %u\n", total);
    status = CAL_STATUS_TOO_FEW_PIXELS;
    sendCalReport(target, status, r, total);
    return;
  }

  // Compute percentiles for each channel
  r.s_min = percentile(g_cal.s_hist, total, CAL_PERCENTILE_LOW);
  r.s_max = percentile(g_cal.s_hist, total, CAL_PERCENTILE_HIGH);
  r.v_min = percentile(g_cal.v_hist, total, CAL_PERCENTILE_LOW);
  r.v_max = percentile(g_cal.v_hist, total, CAL_PERCENTILE_HIGH);

  if (target == CAL_TARGET_WHITE_LINE) {
    // For white line: ignore hue, require low saturation, high value
    r.h_min = 0;
    r.h_max = 255;
    // Allow some slack on the sat/val so partially-shaded line still detected
    if (r.s_max < 40) r.s_max = 40;
    if (r.v_min > 200) r.v_min = 200;
    r.v_max = 255;
  } else {
    // Chromatic targets — need real saturation
    r.h_min = percentile(g_cal.h_hist, total, CAL_PERCENTILE_LOW);
    r.h_max = percentile(g_cal.h_hist, total, CAL_PERCENTILE_HIGH);

    // Sanity check: low saturation on a chromatic target means the user
    // probably calibrated the wrong thing (e.g., camera was looking at the
    // floor instead of the goal). Warn but still apply.
    if (r.s_min < 50) status = CAL_STATUS_UNSATURATED;

    // Widen H range slightly for robustness (HSV hue is sensitive)
    if (r.h_min >= 3) r.h_min -= 3; else r.h_min = 0;
    if (r.h_max <= 252) r.h_max += 3; else r.h_max = 255;

    // Don't constrain S/V max too tight — lighting variation needs headroom
    r.s_max = 255;
    r.v_max = 255;
  }

  if (r.v_max > 250 && r.v_min > 240) status = CAL_STATUS_OVEREXPOSED;

  g_ranges[target] = r;

  Serial.printf("[CAL] DONE target=%u  H[%u..%u] S[%u..%u] V[%u..%u]  px=%u  status=%u\n",
                target, r.h_min, r.h_max, r.s_min, r.s_max, r.v_min, r.v_max, total, status);

  sendCalReport(target, status, r, total);
}

static void sendCalReport(uint8_t target, uint8_t status, const ColorRange& r, uint32_t samples) {
  CalReport rep = {};
  rep.start         = CAL_REPORT_START;
  rep.cam_id        = g_cam_id;
  rep.target        = target;
  rep.status        = status;
  rep.h_min         = r.h_min; rep.h_max = r.h_max;
  rep.s_min         = r.s_min; rep.s_max = r.s_max;
  rep.v_min         = r.v_min; rep.v_max = r.v_max;
  rep.sample_count  = samples > 65535 ? 65535 : (uint16_t)samples;

  // XOR over cam_id..sample_count (skip start and checksum slot)
  uint8_t* raw = (uint8_t*)&rep;
  rep.checksum = xorChecksum(raw + 1, sizeof(CalReport) - 2);

  TEENSY_UART.write(raw, sizeof(CalReport));
}

// ============================================================================
//   UART TX to Teensy
// ============================================================================
static uint8_t xorChecksum(const uint8_t* p, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) c ^= p[i];
  return c;
}

static void sendUartFrame() {
  CamToTeensy pkt = {};
  pkt.start           = UART_START_BYTE;
  pkt.len             = sizeof(CamToTeensy) - 3;
  pkt.cam_id          = g_cam_id;
  pkt.mount_deg_div10 = g_mount_deg_x10 / 100;
  pkt.frame_seq       = g_frame_seq;

  Detection d;
  noInterrupts();
  memcpy(&d, (const void*)&g_det, sizeof(d));
  interrupts();

  pkt.visible_flags = 0;
  if (d.ball_seen)    pkt.visible_flags |= VIS_BALL;
  if (d.yellow_seen)  pkt.visible_flags |= VIS_YELLOW_GOAL;
  if (d.blue_seen)    pkt.visible_flags |= VIS_BLUE_GOAL;
  if (d.line_seen)    pkt.visible_flags |= VIS_WHITE_LINE;
  if (d.line_seen && d.line_dist_band >= LINE_DIST_MID) pkt.visible_flags |= VIS_LINE_CLOSE;

  pkt.ball_angle           = d.ball_seen   ? d.ball_angle_x10   : BALL_NOT_SEEN;
  pkt.ball_radius_px       = d.ball_radius_px;
  pkt.yellow_goal_angle    = d.yellow_seen ? d.yellow_angle_x10 : GOAL_NOT_SEEN;
  pkt.yellow_goal_width_px = d.yellow_width_px;
  pkt.blue_goal_angle      = d.blue_seen   ? d.blue_angle_x10   : GOAL_NOT_SEEN;
  pkt.blue_goal_width_px   = d.blue_width_px;
  pkt.line_angle           = d.line_seen   ? d.line_angle_x10   : LINE_NOT_SEEN;
  pkt.line_dist_band       = d.line_dist_band;
  pkt.line_row_y           = d.line_row_y;
  pkt.ball_confidence      = d.ball_confidence;

  uint8_t* raw = (uint8_t*)&pkt;
  pkt.checksum = xorChecksum(raw + 2, sizeof(CamToTeensy) - 3);

  TEENSY_UART.write(raw, sizeof(CamToTeensy));
}

// ============================================================================
//   UART RX — handle commands from Teensy
// ============================================================================
static void handleUartRx() {
  static enum { WAIT_START, READ_CMD, READ_LEN, READ_PAYLOAD, READ_CHECKSUM } state = WAIT_START;
  static uint8_t cmd, len, idx;
  static uint8_t buf[16];

  while (TEENSY_UART.available()) {
    uint8_t b = TEENSY_UART.read();
    switch (state) {
      case WAIT_START:
        if (b == CMD_START_BYTE) state = READ_CMD;
        break;
      case READ_CMD:
        cmd = b; state = READ_LEN; break;
      case READ_LEN:
        len = b;
        idx = 0;
        if (len > sizeof(buf)) { state = WAIT_START; break; }
        state = (len == 0) ? READ_CHECKSUM : READ_PAYLOAD;
        break;
      case READ_PAYLOAD:
        buf[idx++] = b;
        if (idx >= len) state = READ_CHECKSUM;
        break;
      case READ_CHECKSUM: {
        uint8_t expected = cmd ^ len;
        for (uint8_t i = 0; i < len; i++) expected ^= buf[i];
        if (b == expected) {
          switch (cmd) {
            case CMD_SET_TEAM_STATE:
              if (len == sizeof(TeamStatePayload) && g_is_master) {
                noInterrupts();
                memcpy((void*)&g_team_state, buf, sizeof(TeamStatePayload));
                interrupts();
              }
              break;
            case CMD_SET_TX_POWER:
              if (len == 1) {
                esp_wifi_set_max_tx_power(buf[0]);
                Serial.printf("[CMD] TX pwr → %u qdBm\n", buf[0]);
              }
              break;
            case CMD_CAL_BEGIN:
              if (len == 1) calBegin(buf[0]);
              break;
            case CMD_CAL_COMMIT:
              saveRangesToNvs();
              break;
            case CMD_CAL_RESET:
              resetRangesToFactory();
              if (g_prefs.begin(NVS_NAMESPACE, false)) {
                g_prefs.clear();
                g_prefs.end();
              }
              Serial.println("[CMD] CAL_RESET — factory defaults restored");
              break;
            case CMD_CAL_DUMP:
              for (uint8_t t = 0; t < CAL_TARGET_COUNT; t++) {
                sendCalReport(t, CAL_STATUS_OK, g_ranges[t], 0);
                delay(2);   // spread on the wire
              }
              break;
          }
        }
        state = WAIT_START;
      } break;
    }
  }
}

// ============================================================================
//   ESP-NOW (master only)
// ============================================================================
static bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(COMM_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_max_tx_power(TX_POWER_QDBM);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = COMM_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  return true;
}

static void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(RobotMsg)) return;
  const RobotMsg* m = (const RobotMsg*)data;
  if (m->magic   != COMM_MAGIC)   return;
  if (m->version != COMM_VERSION) return;
  if (m->team_id != MY_TEAM_ID)   return;

  // Forward to Teensy with 0xCC start byte (distinct from 0xAA cam frames
  // and 0xDD cal reports)
  uint8_t out[2 + sizeof(RobotMsg) + 1];
  out[0] = 0xCC;
  out[1] = sizeof(RobotMsg);
  memcpy(out + 2, m, sizeof(RobotMsg));
  out[sizeof(out) - 1] = xorChecksum(out + 2, sizeof(RobotMsg));
  TEENSY_UART.write(out, sizeof(out));
}

static void sendEspNowFrame() {
  Detection d;
  TeamStatePayload ts;
  noInterrupts();
  memcpy(&d,  (const void*)&g_det,        sizeof(d));
  memcpy(&ts, (const void*)&g_team_state, sizeof(ts));
  interrupts();

  RobotMsg msg = {};
  msg.magic       = COMM_MAGIC;
  msg.version     = COMM_VERSION;
  msg.team_id     = MY_TEAM_ID;
  msg.robot_id    = 1;            // CHANGE per-robot or read from a 3rd bridge
  msg.seq         = ++g_espnow_seq;
  msg.tx_millis   = millis();
  msg.role        = (RobotRole)ts.role;
  msg.battery_pct = ts.battery_pct;
  msg.field_x_cm  = ts.field_x_cm;
  msg.field_y_cm  = ts.field_y_cm;

  msg.flags = 0;
  if (d.ball_seen)    msg.flags |= ENMSG_BALL_VISIBLE;
  if (ts.has_ball)    msg.flags |= ENMSG_HAS_BALL;
  if (d.yellow_seen)  msg.flags |= ENMSG_YGOAL_VISIBLE;
  if (d.blue_seen)    msg.flags |= ENMSG_BGOAL_VISIBLE;
  if (d.line_seen && d.line_dist_band >= LINE_DIST_MID) msg.flags |= ENMSG_NEAR_LINE;

  msg.ball_angle        = d.ball_seen   ? d.ball_angle_x10   : BALL_NOT_SEEN;
  msg.ball_radius_px    = d.ball_radius_px;
  msg.yellow_goal_angle = d.yellow_seen ? d.yellow_angle_x10 : GOAL_NOT_SEEN;
  msg.blue_goal_angle   = d.blue_seen   ? d.blue_angle_x10   : GOAL_NOT_SEEN;

  esp_now_send(BROADCAST_MAC, (const uint8_t*)&msg, sizeof(msg));
}
