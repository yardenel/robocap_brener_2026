// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  RoboCap – XIAO ESP32-S3 Sense  │  Camera Object Detection             ║
// ║  RoboCupJunior Soccer 2026                                              ║
// ║                                                                         ║
// ║  All 4 ESPs run IDENTICAL code. Mount angle is read at boot from       ║
// ║  2 PCB strap GPIO pins (hardware-encoded on the PCB footprint).        ║
// ║                                                                         ║
// ║  Field facts (field_specification.pdf):                                 ║
// ║    Goals  : Yellow  /  Blue  (matte interior, §3.0.3)                  ║
// ║    Surface: Green carpet (§4.0.1)                                       ║
// ║    WHITE lines (20 mm): field boundary, penalty areas (§4, §7)         ║
// ║    BLACK lines (thin) : center circle 60 cm Ø, neutral spots (§5, §6)  ║
// ║    Walls  : matte BLACK, 22 cm high (§2.0.1)                           ║
// ║                                                                         ║
// ║  ── UART packet layout ──────────────────────────────────────          ║
// ║  Object detected  → 4 bytes:                                            ║
// ║    [0] ESP unique ID      (0x01–0x04, set in config below)             ║
// ║    [1] Object type        0x00=goal  0x01=white line  0x02=black line  ║
// ║    [2] Angle (int8_t)     camera-local, approx –60° … +60°             ║
// ║         Teensy must add mount offset from its ID→angle table            ║
// ║    [3] Distance proxy     0=far/small … 255=close/large (blob area)    ║
// ║                                                                         ║
// ║  Nothing detected → 2 bytes:                                            ║
// ║    [0] ESP unique ID                                                    ║
// ║    [1] 0xFF  (PACKET_NO_DETECT sentinel)                                ║
// ║                                                                         ║
// ║  Async WiFi events (may appear between normal packets):                 ║
// ║    0xB0 <partnerID>          partner robot found                        ║
// ║    0xB1 <len> <bytes…>       data relayed from partner robot            ║
// ╚══════════════════════════════════════════════════════════════════════════╝

#include "Arduino.h"
#include "WiFi.h"
#include "WiFiUdp.h"
#include "esp_camera.h"


// ══════════════════════════════════════════════════════════════════════════
//  ①  PER-UNIT CONFIGURATION
//     Only ESP_UNIQUE_ID changes between the 4 firmware images.
//     Mount angle is read automatically from GPIO strap pins (see ②).
// ══════════════════════════════════════════════════════════════════════════

// Unique ID for this unit. Must be different on each of the 4 ESPs.
// Suggested mapping: 0x01=forward · 0x02=right · 0x03=rear · 0x04=left
#define ESP_UNIQUE_ID 0x01

// ── PCB strap GPIO pins – hardware-encode the mount angle ───────────────
//
//   we use INPUT_PULLUP and invert the reading.
//   The elegant side effect: the forward ESP needs zero solder work (both pads open
//   = 0°), which is the most common position.
//  STRAP_B shorted		STRAP_A 		shortedMount angle
//		No				No				0°
//Forward — no work needed 		No				Yes
//90° Right 		Yes				No				270° Left 		Yes
//Yes				180° Rear
//
//  GPIO 1 and GPIO 2 are free on XIAO ESP32-S3 (camera uses 10-18, 38-40, 47-48).
#define GPIO_STRAP_A 1  // LSB of 2-bit mount code
#define GPIO_STRAP_B 2  // MSB of 2-bit mount code

// WiFi credentials – both robots on the field must share these
#define WIFI_SSID "RoboCap_Link"
#define WIFI_PASSWORD "rcj2026!"
#define WIFI_UDP_PORT 4210

// ══════════════════════════════════════════════════════════════════════════
//  ②  XIAO ESP32-S3 SENSE CAMERA PIN MAP  (OV2640 – do not change)
// ══════════════════════════════════════════════════════════════════════════
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

// ══════════════════════════════════════════════════════════════════════════
//  ③  PROTOCOL CONSTANTS
// ══════════════════════════════════════════════════════════════════════════
#define PACKET_NO_DETECT 0xFF
#define OBJ_GOAL 0x00        // Colored goal (yellow or blue)
#define OBJ_LINE_WHITE 0x01  // White boundary / penalty area line
#define OBJ_LINE_BLACK 0x02  // Black center circle line

// Teensy → ESP command bytes
#define CMD_QUERY_ID 0xA0     // "Who are you?" → reply ESP_UNIQUE_ID
#define CMD_WIFI_START 0xA1   // Assign WiFi bridge role to this ESP
#define CMD_WIFI_STOP 0xA2    // Revoke WiFi bridge role
#define CMD_WIFI_STATUS 0xA3  // "Partner ID?" → reply partnerID (0=none)
#define CMD_RELAY_DATA 0xC0   // Teensy → ESP: forward <len> <bytes> to partner

// ESP → Teensy async event prefixes
#define EVT_PARTNER_FOUND 0xB0  // Followed by 1 byte: partnerID
#define EVT_WIFI_DATA 0xB1      // Followed by <len> <bytes>

// ══════════════════════════════════════════════════════════════════════════
//  ④  CAMERA & FRAME SETTINGS
// ══════════════════════════════════════════════════════════════════════════
#define FRAME_W 320  // QVGA – easily processed within 100 ms
#define FRAME_H 240
#define CAM_FOV_DEG 120.0f  // Horizontal FOV of stock XIAO S3 Sense lens

// ══════════════════════════════════════════════════════════════════════════
//  ⑤  HSV COLOR THRESHOLDS
//
//  Scale: H 0–179 (degrees/2, OpenCV convention)  S 0–255  V 0–255
//
//  Source for each threshold group:
//    Goals  → field_specification.pdf §3.0.3  (yellow & blue, matte)
//    White  → field_specification.pdf §4.0.1 / §7.0.2  (20 mm painted lines)
//    Black  → field_specification.pdf §5.0.1 / §6.0.1  (thin marker lines)
//    Walls  → field_specification.pdf §2.0.1  (matte black – mitigated by ROI)
//
//  ⚠️  CALIBRATE all thresholds on your actual field under competition
//     lighting conditions before the event.
// ══════════════════════════════════════════════════════════════════════════

// ── Yellow goal (§3.0.3) ─────────────────────────────────────────────────
#define YEL_H_MIN 20  // ~40° hue
#define YEL_H_MAX 35  // ~70° hue
#define YEL_S_MIN 130
#define YEL_S_MAX 255
#define YEL_V_MIN 100
#define YEL_V_MAX 255

// ── Blue goal (§3.0.3 – "brighter shade" recommended) ────────────────────
#define BLU_H_MIN 100  // ~200° hue
#define BLU_H_MAX 130  // ~260° hue
#define BLU_S_MIN 100
#define BLU_S_MAX 255
#define BLU_V_MIN 60  // Slightly relaxed V_MIN because goal depth is dark
#define BLU_V_MAX 255

// ── White lines – field boundary + penalty areas (§4, §7) ────────────────
// Characteristic: high V (bright), very low S (achromatic / near-white)
#define WHITE_S_MAX 50
#define WHITE_V_MIN 185

// ── Black lines – center circle + neutral spots (§5, §6) ─────────────────
// Characteristic: very low V (dark), low S
// ALSO matches walls (§2) – mitigated by scan-band position (see ⑰)
#define BLACK_S_MAX 60
#define BLACK_V_MAX 55

// Minimum blob pixel counts (noise rejection)
#define MIN_GOAL_PIX 400   // ~1.5% of QVGA active scan area
#define MIN_WHITE_PIX 150  // White lines: 20 mm wide, may be at distance
#define MIN_BLACK_PIX 100  // Center circle: very thin marker line

// ══════════════════════════════════════════════════════════════════════════
//  ⑥  TIMING
// ══════════════════════════════════════════════════════════════════════════
#define TICK_MS 100        // Sensing + transmit period (ms)
#define WIFI_BCAST_MS 500  // WHO_AM_I broadcast interval (ms)

// ══════════════════════════════════════════════════════════════════════════
//  ⑦  DATA TYPES
// ══════════════════════════════════════════════════════════════════════════
struct Detection {
    bool detected;
    uint8_t objType;   // OBJ_GOAL / OBJ_LINE_WHITE / OBJ_LINE_BLACK
    int8_t angle;      // Camera-local signed degrees (Teensy adds mountOffset)
    uint8_t distance;  // 0=far … 255=close
};

typedef enum { WS_IDLE, WS_SCANNING, WS_PAIRED, WS_STOPPED } WifiState;

// ══════════════════════════════════════════════════════════════════════════
//  ⑧  GLOBALS
// ══════════════════════════════════════════════════════════════════════════
static WiFiUDP udp;
static WifiState wifiState = WS_IDLE;
static bool isWifiESP      = false;
static uint8_t partnerID   = 0x00;
static int mountAngle      = 0;  // Set in setup() from GPIO straps
static uint32_t lastTick   = 0;
static uint32_t lastBcast  = 0;

// ══════════════════════════════════════════════════════════════════════════
//  ⑨  PROTOTYPES
// ══════════════════════════════════════════════════════════════════════════
bool initCamera();
int readMountAngle();
Detection detectObjects(camera_fb_t* fb);
Detection scanGoal(camera_fb_t* fb);
Detection scanWhiteLine(camera_fb_t* fb);
Detection scanBlackLine(camera_fb_t* fb);
void rgb565ToHSV(uint16_t px, uint8_t& h, uint8_t& s, uint8_t& v);
int8_t pixelToAngle(int cx);
uint8_t blobToDistance(long pixelCount, long maxPixels);
void sendPacket(const Detection& d);
void handleTeensyCommands();
void initWiFi();
void wifiTask();

// ══════════════════════════════════════════════════════════════════════════
//  ⑩  SETUP
// ══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(300);

    // Read mount angle from PCB hardware straps (set once at assembly)
    pinMode(GPIO_STRAP_A, INPUT_PULLUP);
    pinMode(GPIO_STRAP_B, INPUT_PULLUP);
    delay(10);
    mountAngle = readMountAngle();

    // Then immediately release:
    // release the pullup resistors to save current
    pinMode(GPIO_STRAP_A, INPUT);
    pinMode(GPIO_STRAP_B, INPUT);

    // Boot report – mount angle is hardware-derived, not guessed
    Serial.printf(
        "\n[RoboCap] ID=0x%02X  mount=%d deg  (STRAP_B=%d STRAP_A=%d)\n",
        ESP_UNIQUE_ID,
        mountAngle,
        digitalRead(GPIO_STRAP_B),
        digitalRead(GPIO_STRAP_A)
    );

    if (!initCamera()) {
        Serial.println("[FATAL] Camera init failed. Halting.");
        while (true) {
            delay(1000);
        }
    }
    Serial.println("[OK] Camera ready. 100 ms loop starting.");
    lastTick = millis();
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑪  MAIN LOOP
// ══════════════════════════════════════════════════════════════════════════
void loop() {
    handleTeensyCommands();
    if (isWifiESP) wifiTask();

    if ((millis() - lastTick) < TICK_MS) return;
    lastTick = millis();

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        // Skip tick on frame grab failure; do NOT send a packet (Teensy timeout)
        return;
    }

    Detection result = detectObjects(fb);
    esp_camera_fb_return(fb);
    sendPacket(result);
}

// ══════════════════════════════════════════════════════════════════════
//  MOUNT ANGLE  –  internal pull-up version
//
//  Pads default HIGH (open). Shorting a pad to GND pulls it LOW.
//  Invert the readings so that "shorted = 1" for the bit-code.
//
//  PCB footprint per ESP position:
//    Two pads: one connected to GPIO_STRAP_A, one to GPIO_STRAP_B.
//    Each pad has a trace to GND with a solder-bridge gap.
//    Short the gap with solder to activate that bit.
//    No external resistors needed – internal pull-ups handle it.
// ══════════════════════════════════════════════════════════════════════
int readMountAngle() {
    // Invert: open pad = HIGH = 0, shorted pad = LOW = 1
    uint8_t a   = !digitalRead(GPIO_STRAP_A);  // bit 0 (LSB)
    uint8_t b   = !digitalRead(GPIO_STRAP_B);  // bit 1 (MSB)
    uint8_t Key = a | b;

    // because of PCB numbering mistake the retun valuues will be: 0, 90, 270, 180
    switch (Key) {
        case 0:
            return 0;  // 0 degrees (FWD)
        case 1:
            return 90;  // 90 degrees (Right)
        case 2:
            return 270;  // 270 degrees (Left)
        case 3:
            return 180;  // 180 degrees (Rear)
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑬  CAMERA INITIALIZATION
// ══════════════════════════════════════════════════════════════════════════
bool initCamera() {
    camera_config_t cfg;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0       = Y2_GPIO_NUM;
    cfg.pin_d1       = Y3_GPIO_NUM;
    cfg.pin_d2       = Y4_GPIO_NUM;
    cfg.pin_d3       = Y5_GPIO_NUM;
    cfg.pin_d4       = Y6_GPIO_NUM;
    cfg.pin_d5       = Y7_GPIO_NUM;
    cfg.pin_d6       = Y8_GPIO_NUM;
    cfg.pin_d7       = Y9_GPIO_NUM;
    cfg.pin_xclk     = XCLK_GPIO_NUM;
    cfg.pin_pclk     = PCLK_GPIO_NUM;
    cfg.pin_vsync    = VSYNC_GPIO_NUM;
    cfg.pin_href     = HREF_GPIO_NUM;
    cfg.pin_sscb_sda = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl = SIOC_GPIO_NUM;
    cfg.pin_pwdn     = PWDN_GPIO_NUM;
    cfg.pin_reset    = RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_RGB565;
    cfg.frame_size   = FRAMESIZE_QVGA;
    cfg.fb_count     = 2;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;

    if (esp_camera_init(&cfg) != ESP_OK) return false;

    sensor_t* s = esp_camera_sensor_get();
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
    s->set_hmirror(s, 0);  // Set to 1 if image appears mirrored on PCB
    s->set_vflip(s, 0);    // Set to 1 if image appears upside-down
    return true;
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑭  MASTER DETECTION  – priority order: goal > white line > black line
//
//  Goals are the primary navigation target.
//  White lines signal boundaries / penalty zones (high priority constraint).
//  Black lines (center circle) are lower-priority position hints.
// ══════════════════════════════════════════════════════════════════════════
Detection detectObjects(camera_fb_t* fb) {
    Detection g = scanGoal(fb);
    if (g.detected) return g;

    Detection w = scanWhiteLine(fb);
    if (w.detected) return w;

    return scanBlackLine(fb);  // detected=false if nothing found
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑮  GOAL DETECTION  (yellow or blue blob)
//
//  Scan ROI: lower 2/3 of frame.
//    Upper 1/3 skipped: contains ceiling, audience shirts (Rule 3.1.4),
//    and color noise from overhead lighting.
//
//  Single pass accumulates both yellow and blue centroids separately.
//  Larger valid blob wins. ESP does NOT distinguish which color = our goal;
//  Teensy does that using pre-match side-assignment and heading data.
// ══════════════════════════════════════════════════════════════════════════
Detection scanGoal(camera_fb_t* fb) {
    Detection res = {false, OBJ_GOAL, 0, 0};
    uint16_t* pix = (uint16_t*)fb->buf;
    const int W = fb->width, H = fb->height;

    long ySumX = 0, yCount = 0;
    long bSumX = 0, bCount = 0;

    for (int row = H / 3; row < H; row++) {
        for (int col = 0; col < W; col++) {
            // OV2640 RGB565 is big-endian; swap bytes
            uint16_t px = pix[row * W + col];
            px          = (uint16_t)((px >> 8) | (px << 8));

            uint8_t h, s, v;
            rgb565ToHSV(px, h, s, v);

            if (h
                >= YEL_H_MIN
                && h
                <= YEL_H_MAX
                && s
                >= YEL_S_MIN
                && s
                <= YEL_S_MAX
                && v
                >= YEL_V_MIN
                && v
                <= YEL_V_MAX) {
                ySumX += col;
                yCount++;
            }
            if (h
                >= BLU_H_MIN
                && h
                <= BLU_H_MAX
                && s
                >= BLU_S_MIN
                && s
                <= BLU_S_MAX
                && v
                >= BLU_V_MIN
                && v
                <= BLU_V_MAX) {
                bSumX += col;
                bCount++;
            }
        }
    }

    long bestCount = 0, bestSumX = 0;
    if (yCount >= MIN_GOAL_PIX && yCount >= bCount) {
        bestCount = yCount;
        bestSumX  = ySumX;
    } else if (bCount >= MIN_GOAL_PIX) {
        bestCount = bCount;
        bestSumX  = bSumX;
    }
    if (bestCount == 0) return res;

    res.detected = true;
    res.objType  = OBJ_GOAL;
    res.angle    = pixelToAngle((int)(bestSumX / bestCount));
    res.distance = blobToDistance(bestCount, 18000L);
    return res;
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑯  WHITE LINE DETECTION  (field boundary + penalty area, §4/§7)
//
//  Scan ROI: bottom half of frame (lines are on the ground directly ahead).
//  White = high V, very low S (achromatic). Any hue qualifies.
//  Centroid gives dominant line direction; distance byte encodes proximity.
// ══════════════════════════════════════════════════════════════════════════
Detection scanWhiteLine(camera_fb_t* fb) {
    Detection res = {false, OBJ_LINE_WHITE, 0, 0};
    uint16_t* pix = (uint16_t*)fb->buf;
    const int W = fb->width, H = fb->height;

    long sumX = 0, count = 0;

    for (int row = H / 2; row < H; row++) {
        for (int col = 0; col < W; col++) {
            uint16_t px = pix[row * W + col];
            px          = (uint16_t)((px >> 8) | (px << 8));
            uint8_t h, s, v;
            rgb565ToHSV(px, h, s, v);
            if (s <= WHITE_S_MAX && v >= WHITE_V_MIN) {
                sumX += col;
                count++;
            }
        }
    }

    if (count < MIN_WHITE_PIX) return res;
    res.detected = true;
    res.objType  = OBJ_LINE_WHITE;
    res.angle    = pixelToAngle((int)(sumX / count));
    res.distance = blobToDistance(count, 8000L);
    return res;
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑰  BLACK LINE DETECTION  (center circle §6 + neutral spots §5)
//
//  Scan ROI: middle band only (rows H/3 … 2H/3).
//    TOP of frame (rows 0..H/3): matte black walls (§2.0.1) dominate here.
//      Excluding top rows prevents wall false-positives.
//    BOTTOM of frame (rows 2H/3..H): reserved for white line detection above,
//      and very near-ground pixels that produce noise due to low grazing angle.
//    MIDDLE band: center circle appears here when robot is mid-field.
//
//  If wall false-positives still occur, raise MIN_BLACK_PIX.
//  Neutral spots (§5, 1 cm diameter) are unlikely to reliably trigger at
//  normal game distances; center circle (§6, 60 cm diameter) is the main
//  useful black feature for localization.
// ══════════════════════════════════════════════════════════════════════════
Detection scanBlackLine(camera_fb_t* fb) {
    Detection res = {false, OBJ_LINE_BLACK, 0, 0};
    uint16_t* pix = (uint16_t*)fb->buf;
    const int W = fb->width, H = fb->height;

    long sumX = 0, count = 0;

    for (int row = H / 3; row < (2 * H) / 3; row++) {
        for (int col = 0; col < W; col++) {
            uint16_t px = pix[row * W + col];
            px          = (uint16_t)((px >> 8) | (px << 8));
            uint8_t h, s, v;
            rgb565ToHSV(px, h, s, v);
            // Black: very low V, low S (distinct from colored objects and green
            // carpet)
            if (s <= BLACK_S_MAX && v <= BLACK_V_MAX) {
                sumX += col;
                count++;
            }
        }
    }

    if (count < MIN_BLACK_PIX) return res;
    res.detected = true;
    res.objType  = OBJ_LINE_BLACK;
    res.angle    = pixelToAngle((int)(sumX / count));
    res.distance = blobToDistance(count, 5000L);
    return res;
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑱  RGB565 → HSV  (integer arithmetic, no float)
//
//  H: 0–179  S: 0–255  V: 0–255  (OpenCV HSV convention)
// ══════════════════════════════════════════════════════════════════════════
void rgb565ToHSV(uint16_t px, uint8_t& h, uint8_t& s, uint8_t& v) {
    uint8_t r = ((px >> 11) & 0x1F) << 3;
    uint8_t g = ((px >> 5) & 0x3F) << 2;
    uint8_t b = ((px >> 0) & 0x1F) << 3;

    uint8_t maxC = max(r, max(g, b));
    uint8_t minC = min(r, min(g, b));
    int delta    = (int)maxC - (int)minC;

    v = maxC;
    s = (maxC == 0) ? 0 : (uint8_t)((long)delta * 255L / maxC);
    if (delta == 0) {
        h = 0;
        return;
    }

    int hue;
    if (maxC == r)
        hue = 60 * ((int)g - (int)b) / delta;
    else if (maxC == g)
        hue = 60 * ((int)b - (int)r) / delta + 120;
    else
        hue = 60 * ((int)r - (int)g) / delta + 240;
    if (hue < 0) hue += 360;
    h = (uint8_t)(hue / 2);
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑲  PIXEL CENTROID → CAMERA-LOCAL ANGLE
//
//  Output is camera-local (NOT robot-relative).
//  Teensy converts: robotAngle = cameraAngle + mountAngle[espID]
//
//  cx = 0     → –(FOV/2) = –60°
//  cx = W/2   → 0°
//  cx = W–1   → +(FOV/2) = +60°
// ══════════════════════════════════════════════════════════════════════════
int8_t pixelToAngle(int cx) {
    float norm  = (float)(cx - FRAME_W / 2) / (float)(FRAME_W / 2);
    float angle = norm * (CAM_FOV_DEG / 2.0f);
    return (int8_t)constrain((int)angle, -128, 127);
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑳  BLOB SIZE → DISTANCE PROXY  (0=far … 255=close)
//
//  maxPixels is object-specific (larger objects need higher values):
//    Goals:       18000  (large colored area when robot ~15–20 cm away)
//    White lines:  8000  (wide line, robot near boundary)
//    Black lines:  5000  (thin center-circle line)
// ══════════════════════════════════════════════════════════════════════════
uint8_t blobToDistance(long pixelCount, long maxPixels) {
    return (uint8_t)(constrain(pixelCount, 0L, maxPixels) * 255L / maxPixels);
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉑  SEND UART PACKET  (ESP → Teensy, no ACK)
// ══════════════════════════════════════════════════════════════════════════
void sendPacket(const Detection& d) {
    if (d.detected) {
        Serial.write((uint8_t)ESP_UNIQUE_ID);
        Serial.write((uint8_t)d.objType);
        Serial.write((uint8_t)d.angle);  // Cast to int8_t on Teensy side
        Serial.write((uint8_t)d.distance);
    } else {
        Serial.write((uint8_t)ESP_UNIQUE_ID);
        Serial.write((uint8_t)PACKET_NO_DETECT);
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉒  HANDLE TEENSY → ESP COMMANDS  (non-blocking, called every loop)
// ══════════════════════════════════════════════════════════════════════════
void handleTeensyCommands() {
    while (Serial.available() > 0) {
        uint8_t cmd = (uint8_t)Serial.read();
        switch (cmd) {
            case CMD_QUERY_ID:
                Serial.write((uint8_t)ESP_UNIQUE_ID);
                break;
            case CMD_WIFI_START:
                if (!isWifiESP) {
                    isWifiESP = true;
                    wifiState = WS_SCANNING;
                    partnerID = 0x00;
                    initWiFi();
                }
                break;
            case CMD_WIFI_STOP:
                wifiState = WS_STOPPED;
                isWifiESP = false;
                WiFi.disconnect(true);
                WiFi.mode(WIFI_OFF);
                break;
            case CMD_WIFI_STATUS:
                Serial.write((uint8_t)partnerID);  // 0x00 = not yet paired
                break;
            default:
                break;  // Unknown / noise byte – discard
        }
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉓  WIFI INITIALIZATION
// ══════════════════════════════════════════════════════════════════════════
void initWiFi() {
    WiFi.mode(WIFI_AP);
    // Unique AP name per ESP so two robots don't collide
    WiFi.softAP(("RCap_" + String(ESP_UNIQUE_ID, HEX)).c_str(), WIFI_PASSWORD);
    udp.begin(WIFI_UDP_PORT);
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉔  WIFI TASK  (inter-robot discovery + transparent relay)
//
//  WS_SCANNING : broadcast WHO_AM_I every WIFI_BCAST_MS ms.
//    On "WHO_AM_I:<id>" received from partner  → reply ROBOT_ID:<ownID>.
//    On "ROBOT_ID:<id>"  received from partner  → partner found, go PAIRED.
//
//  WS_PAIRED : transparent bridge.
//    UDP received                → EVT_WIFI_DATA (0xB1 <len> <bytes>) to Teensy
//    0xC0 <len> <bytes> from Teensy → forward as UDP broadcast to partner
// ══════════════════════════════════════════════════════════════════════════
void wifiTask() {
    if (wifiState == WS_IDLE || wifiState == WS_STOPPED) return;

    // Receive incoming UDP
    int pktSize = udp.parsePacket();
    if (pktSize > 0 && pktSize < 128) {
        char buf[128] = {0};
        udp.read(buf, sizeof(buf) - 1);
        String msg = String(buf);

        if (wifiState == WS_SCANNING) {
            if (msg.startsWith("ROBOT_ID:")) {
                partnerID = (uint8_t)msg.substring(9).toInt();
                wifiState = WS_PAIRED;
                Serial.write((uint8_t)EVT_PARTNER_FOUND);
                Serial.write((uint8_t)partnerID);
            } else if (msg.startsWith("WHO_AM_I:")) {
                udp.beginPacket(udp.remoteIP(), WIFI_UDP_PORT);
                udp.print("ROBOT_ID:" + String((int)ESP_UNIQUE_ID));
                udp.endPacket();
            }
        } else if (wifiState == WS_PAIRED) {
            uint8_t len = (uint8_t)min(pktSize, 60);
            Serial.write((uint8_t)EVT_WIFI_DATA);
            Serial.write(len);
            Serial.write((uint8_t*)buf, len);
        }
    }

    // Periodic WHO_AM_I broadcast while scanning
    if (wifiState == WS_SCANNING && (millis() - lastBcast) >= WIFI_BCAST_MS) {
        lastBcast  = millis();
        String msg = "WHO_AM_I:" + String((int)ESP_UNIQUE_ID);
        udp.beginPacket(IPAddress(192, 168, 4, 255), WIFI_UDP_PORT);
        udp.print(msg);
        udp.endPacket();
    }

    // Forward Teensy relay data to partner
    if (wifiState == WS_PAIRED && Serial.available() >= 2) {
        if ((uint8_t)Serial.peek() == CMD_RELAY_DATA) {
            Serial.read();
            uint8_t len = Serial.read();
            if (len > 0 && len <= 64 && Serial.available() >= len) {
                uint8_t payload[64];
                Serial.readBytes(payload, len);
                udp.beginPacket(IPAddress(192, 168, 4, 255), WIFI_UDP_PORT);
                udp.write(payload, len);
                udp.endPacket();
            }
        }
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  TEENSY 4.1 INTEGRATION REFERENCE
// ══════════════════════════════════════════════════════════════════════════
//
//  A. STARTUP SEQUENCE
//  ────────────────────
//  1. Send 0xA0 to each of the 4 Serial ports, read back 1 byte → ID.
//     This builds the runtime port→ID map.
//  2. Build the ID→mountAngle table matching the PCB strap wiring:
//       const int MOUNT[5] = {0, 0, 90, 180, 270}; // index=ID
//  3. Send 0xA1 to the ESP chosen for WiFi (e.g. forward, ID=0x01).
//     Poll 0xA3 until EVT_PARTNER_FOUND (0xB0 <partnerID>) arrives.
//  4. Use partnerID byte to determine robot game role.
//
//  B. PACKET PARSING (per Serial port, every loop)
//  ─────────────────────────────────────────────────
//  byte0 = read()  → ESP_UNIQUE_ID (known; validate if desired)
//  byte1 = read()
//    case 0xFF       → no detection, done (2 bytes consumed)
//    case 0xB0       → partner found; read 1 more byte = partnerID
//    case 0xB1       → WiFi data;    read <len> byte, then <len> bytes
//    case 0x00/01/02 → detection;    read angle + distance (2 more bytes)
//
//  C. ANGLE CONVERSION
//  ─────────────────────
//  int8_t cameraAngle = (int8_t)angleByte;
//  int    robotAngle  = (int)cameraAngle + MOUNT[espID];
//  while (robotAngle >  180) robotAngle -= 360;
//  while (robotAngle < -180) robotAngle += 360;
//  // robotAngle: 0=forward, +90=right, -90=left, ±180=rear
//
//  D. PCB STRAP NOTES FOR NEXT REVISION
//  ──────────────────────────────────────
//  Per ESP footprint, add:
//    • 10 kΩ pull-down resistors: GPIO1→GND, GPIO2→GND  (always present)
//    • Solder-bridge pads:        GPIO1→3V3, GPIO2→3V3  (install per table)
//  Mount table from section ①:
//    Forward  (0°)  : both bridges open   (B=LOW, A=LOW)
//    Right    (90°) : bridge A only       (B=LOW, A=HIGH)
//    Rear    (180°) : bridge B only       (B=HIGH, A=LOW)
//    Left    (270°) : both bridges closed (B=HIGH, A=HIGH)
//  No firmware change needed when repositioning units on the PCB.
// ══════════════════════════════════════════════════════════════════════════
