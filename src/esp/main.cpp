// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  RoboCap – XIAO ESP32-S3 Sense │ Combined Firmware  [v2.2]             ║
// ║  RoboCupJunior Soccer 2026                                              ║
// ║                                                                         ║
// ║  ── v2.2 CHANGELOG ──                                                  ║
// ║  Inter-robot WiFi (Mission 3) migrated from SoftAP+UDP to ESP-NOW.     ║
// ║  Why: SoftAP-only on both robots cannot pair (each is its own subnet). ║
// ║  ESP-NOW is symmetric, connectionless, lower latency, same range.      ║
// ║  Identical firmware on both robots, no ROBOT_ID compile flag needed.   ║
// ║  Application protocol (WHO_AM_I, ROBOT_ID, relay) preserved 1:1.       ║
// ║  UART protocol (binary + ASCII CAL:) and Teensy events UNCHANGED.      ║
// ║                                                                         ║
// ║  THREE MISSIONS (all 4 ESPs run IDENTICAL code):                       ║
// ║                                                                         ║
// ║  MISSION 1 – CAMERA OBJECT DETECTION                                   ║
// ║    OV2640 camera (120° FOV, QVGA, RGB565)                              ║
// ║    Detects: Yellow goal, Blue goal, White field lines, Black lines      ║
// ║    Sources: field_specification.pdf §2–§7                               ║
// ║      Goals  : Yellow / Blue (matte interior, §3.0.3)                   ║
// ║      Surface: Green carpet (§4.0.1)                                     ║
// ║      WHITE lines (20 mm): field boundary, penalty areas (§4, §7)       ║
// ║      BLACK lines (thin) : center circle 60 cm Ø, neutral spots (§5,§6) ║
// ║      Walls  : matte BLACK, 22 cm high (§2.0.1)                         ║
// ║                                                                         ║
// ║  MISSION 2 – UART COMMUNICATION WITH TEENSY 4.1                        ║
// ║    Dual-mode UART on same Serial port:                                  ║
// ║    ── Binary mode (bytes ≥ 0x80 or special opcodes):                   ║
// ║       Detection packet (4 bytes, object found):                         ║
// ║         [0] ESP unique ID    (0x01–0x04)                               ║
// ║         [1] Object type      0x00=goal 0x01=white line 0x02=black line ║
// ║         [2] Angle (int8_t)   camera-local –60°…+60° (Teensy adds offset)║
// ║         [3] Distance proxy   0=far … 255=close (blob area based)       ║
// ║       No-detection packet (2 bytes):                                    ║
// ║         [0] ESP unique ID                                               ║
// ║         [1] 0xFF                                                        ║
// ║       Async WiFi events (between normal packets):                       ║
// ║         0xB0 <macRank>     partner found (1=we hold higher MAC,2=lower)  ║
// ║         0xB1 <len> <data>  data from partner robot                      ║
// ║    ── ASCII mode (printable bytes, '\n' terminated):                   ║
// ║       Teensy → ESP: CAL: commands (update camera HSV thresholds)       ║
// ║       ESP → Teensy: OK: / ERR: / INFO: / WARN: responses               ║
// ║       See SECTION ⑦ for full CAL: command reference.                   ║
// ║                                                                         ║
// ║  MISSION 3 – INTER-ROBOT WIFI (forward ESP only, mountAngle == 0°)    ║
// ║    Automatically started at boot when PCB straps = 0,0 (forward).      ║
// ║    Teensy CMD_WIFI_START/STOP can override at runtime.                  ║
// ║    Protocol: ESP-NOW broadcast (symmetric, no AP/STA roles).           ║
// ║    WHO_AM_I discovery → ROBOT_ID pairing → transparent relay.          ║
// ║                                                                         ║
// ║  TEENSY → ESP BINARY COMMANDS:                                          ║
// ║    0xA0 CMD_QUERY_ID    → reply ESP_UNIQUE_ID                           ║
// ║    0xA1 CMD_WIFI_START  → start WiFi bridge (overrides auto-start)      ║
// ║    0xA2 CMD_WIFI_STOP   → stop WiFi bridge                              ║
// ║    0xA3 CMD_WIFI_STATUS → reply macRank (0=unpaired,1=higher,2=lower)   ║
// ║    0xC0 CMD_RELAY_DATA  → <len><bytes> forward to partner via UDP       ║
// ╚══════════════════════════════════════════════════════════════════════════╝

#include "esp_camera.h"
#include "Arduino.h"
#include "WiFi.h"
#include "esp_now.h"             // [v2.2] replaces WiFiUdp.h
#include "Preferences.h"         // ESP32 NVS (replaces EEPROM for threshold storage)
#include "esp_wifi.h"            // [v3] TX-power cap
#include <ESPAsyncWebServer.h>   // [v3] TEST-mode web console (needs AsyncTCP)
#include "robot_protocol.h"      // [v3] shared single source of truth (Teensy + ESP)
#include "webapp_html.h"         // [v3] embedded English Web UI (PROGMEM)

// ── [v3] Serial split ──────────────────────────────────────────────────────
//  TEENSY = UART0 on the TX/RX pads (GPIO43 TX / GPIO44 RX) -> Teensy main CPU.
//  DBG    = USB-CDC (the USB-C port) -> human-readable debug monitor ONLY.
//  REQUIRES board setting:  Tools -> "USB CDC On Boot" -> Enabled
//    (so `Serial` = USB-CDC and `Serial0` = UART0). With it Disabled, `Serial`
//    would BE UART0 and the two would collide.
#define TEENSY Serial0
#define DBG    Serial


// ══════════════════════════════════════════════════════════════════════════
//  ①  PER-UNIT CONFIGURATION
//     The ONLY value that must differ between the 4 firmware images.
//     Mount angle is read automatically from 2 PCB strap GPIO pins.
// ══════════════════════════════════════════════════════════════════════════

// Unique ID for this unit = CAMERA POSITION (not robot identity).
//   0x01=forward(0°) · 0x02=right(90°) · 0x03=rear(180°) · 0x04=left(270°)
// [FIX] No longer a compile-time constant: the single shared firmware image
//       derived it as 1 on every unit. It is now computed at boot from the
//       SAME two strap pins that produce mountAngle (see setup()). The global
//       declaration lives next to mountAngle; 0x01 there is only a fallback.
// NOTE: This is the per-robot camera ID used by Teensy CMD_QUERY_ID port
//       mapping. ROBOT identity over ESP-NOW is a separate concern → MAC-based
//       (TODO once Teensy protocol for 0xB0/0xA3 is confirmed).

// ── PCB strap GPIO pins (hardware-encode mount angle) ───────────────────
//
//  Two pads per ESP footprint on the upper PCB.  Internal pull-up is used.
//  DEFAULT: pad open   → pin reads HIGH  → bit = 0
//  BRIDGED: pad shorted to GND → pin reads LOW → bit = 1
//
//   STRAP_B  STRAP_A │ code │ Mount angle │ Direction │ WiFi auto-start
//   ─────────────────┼──────┼─────────────┼───────────┼────────────────
//     open     open  │  0   │     0°      │ Forward   │ YES (auto)
//     open     GND   │  1   │    90°      │ Right     │ no
//     GND      open  │  2   │   180°      │ Rear      │ no
//     GND      GND   │  3   │   270°      │ Left      │ no
//
//  GPIO1 = D0, GPIO2 = D1 on XIAO ESP32-S3 (safe; not used by camera).
//  No external resistors needed — internal pull-ups handle it.
//  After reading, pins are released to INPUT (no pull) to save ~100 µA.
#define GPIO_STRAP_A      1     // LSB (D0)
#define GPIO_STRAP_B      2     // MSB (D1)

// WiFi credentials – both robots must share these
// [v2.2] SSID/UDP_PORT no longer used for inter-robot (ESP-NOW now).
// WIFI_PASSWORD will be reused for Test Mode SoftAP (added in a later patch).
#define WIFI_SSID         "RoboCap_Link"
#define WIFI_PASSWORD     "rcj2026!"
#define WIFI_UDP_PORT     4210     // [v2.2] unused now – kept for future Test Mode TCP

// ══════════════════════════════════════════════════════════════════════════
//  ②  XIAO ESP32-S3 SENSE – OV2640 CAMERA PIN MAP  (do not change)
// ══════════════════════════════════════════════════════════════════════════
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

// ══════════════════════════════════════════════════════════════════════════
//  ③  BINARY UART PROTOCOL CONSTANTS
// ══════════════════════════════════════════════════════════════════════════
// [v3] detection/cmd/event constants moved to robot_protocol.h (single source of truth)

// ══════════════════════════════════════════════════════════════════════════
//  ④  CAMERA & FRAME SETTINGS
// ══════════════════════════════════════════════════════════════════════════
#define FRAME_W           320    // QVGA width  – fits in 100 ms budget
#define FRAME_H           240    // QVGA height
#define CAM_FOV_DEG       120.0f // Horizontal FOV of XIAO S3 Sense stock lens

// ══════════════════════════════════════════════════════════════════════════
//  ⑤  DEFAULT HSV COLOR THRESHOLDS  (loaded from NVS if previously saved)
//
//  H: 0–179  (degrees/2, OpenCV convention)   S: 0–255   V: 0–255
//
//  Field sources (field_specification.pdf):
//    §3.0.3 → Yellow goal (matte) / Blue goal (matte, brighter shade)
//    §4.0.1 / §7.0.2 → White painted lines, 20 mm wide
//    §5.0.1 / §6.0.1 → Black marker lines (neutral spots, center circle)
//    §2.0.1 → Matte black walls – mitigated by scan-ROI position (see ⑭–⑯)
//
//  ⚠️  These compiled defaults MUST be calibrated at the competition venue.
//     Use CAL: commands (SECTION ⑦) to update without reflashing.
// ══════════════════════════════════════════════════════════════════════════

// ── Yellow goal ──────────────────────────────────────────────────────────
#define DEF_YEL_H_MIN   20
#define DEF_YEL_H_MAX   35
#define DEF_YEL_S_MIN  130
#define DEF_YEL_S_MAX  255
#define DEF_YEL_V_MIN  100
#define DEF_YEL_V_MAX  255

// ── Blue goal ─────────────────────────────────────────────────────────────
#define DEF_BLU_H_MIN  100
#define DEF_BLU_H_MAX  130
#define DEF_BLU_S_MIN  100
#define DEF_BLU_S_MAX  255
#define DEF_BLU_V_MIN   60    // Relaxed: goal interior is recessed and darker
#define DEF_BLU_V_MAX  255

// ── White lines (field boundary + penalty area) ───────────────────────────
#define DEF_WHITE_S_MAX   50   // Near-achromatic (low saturation)
#define DEF_WHITE_V_MIN  185   // High brightness

// ── Black lines (center circle + neutral spots) ───────────────────────────
#define DEF_BLACK_S_MAX   60   // Low saturation
#define DEF_BLACK_V_MAX   55   // Very low brightness

// Minimum blob sizes for noise rejection
#define MIN_GOAL_PIX     400   // ~1.5% of QVGA scan area
#define MIN_WHITE_PIX    150   // White lines: 20 mm real width
#define MIN_BLACK_PIX    100   // Center circle: thin marker line

// ══════════════════════════════════════════════════════════════════════════
//  ⑥  TIMING
// ══════════════════════════════════════════════════════════════════════════
#define TICK_MS          100   // Camera sensing + transmit period (ms)
#define WIFI_BCAST_MS    500   // WHO_AM_I broadcast interval (ms)

// ══════════════════════════════════════════════════════════════════════════
//  ⑦  ASCII CALIBRATION PROTOCOL  (Teensy → ESP via UART, '\n' terminated)
//
//  Mirrors the calibration framework in the Teensy firmware (color_calibration
//  _and_UART_FW.ino) — but here it updates the ESP's OWN camera HSV thresholds
//  stored in NVS (Preferences), not the Teensy's TCS34725 thresholds.
//
//  Commands received from Teensy (ASCII, any UART):
//    CAL:YELLOW:hMin,hMax,sMin,sMax,vMin,vMax   → update yellow goal HSV range
//    CAL:BLUE:hMin,hMax,sMin,sMax,vMin,vMax     → update blue   goal HSV range
//    CAL:WHITE:sMax,vMin                         → update white line thresholds
//    CAL:BLACK:sMax,vMax                         → update black line thresholds
//    CAL:SAVE                                    → persist active thresholds to NVS
//    CAL:LOAD                                    → restore thresholds from NVS
//    CAL:RESET                                   → restore compiled-in defaults
//    CAL:STATUS                                  → print all active thresholds
//
//  ESP replies (ASCII, printed back on same Serial):
//    OK:<command>          e.g. OK:SAVE
//    ERR:<reason>          e.g. ERR:PARSE_YELLOW
//    INFO:<message>        e.g. INFO:CAL_LOADED_NVS
//    WARN:<message>        e.g. WARN:CAL_USING_DEFAULTS
//
//  ASCII line buffer: bytes 0x20–0x7E are buffered; '\n' triggers processing.
//  Binary command bytes (≥ 0x80) bypass the line buffer entirely.
// ══════════════════════════════════════════════════════════════════════════

// ══════════════════════════════════════════════════════════════════════════
//  ⑧  DATA STRUCTURES
// ══════════════════════════════════════════════════════════════════════════

// Active camera HSV thresholds (runtime, updated by CAL: commands)
struct CameraThresholds {
  uint8_t yelHMin, yelHMax, yelSMin, yelSMax, yelVMin, yelVMax;
  uint8_t bluHMin, bluHMax, bluSMin, bluSMax, bluVMin, bluVMax;
  uint8_t whiteSMax, whiteVMin;
  uint8_t blackSMax, blackVMax;
};

// Result of one detection scan tick
struct Detection {
  bool    detected;
  uint8_t objType;   // OBJ_GOAL / OBJ_LINE_WHITE / OBJ_LINE_BLACK
  int8_t  angle;     // Camera-local signed degrees; Teensy adds mountAngle
  uint8_t distance;  // 0=far/small … 255=close/large
};

// WiFi state machine
typedef enum { WS_IDLE, WS_SCANNING, WS_PAIRED, WS_STOPPED } WifiState;

// ASCII line buffer for CAL: command parsing (interrupt-safe single buffer)
#define ASCII_BUF_SIZE  128
struct AsciiLineBuf {
  char    buf[ASCII_BUF_SIZE];
  uint8_t head      = 0;
  bool    lineReady = false;
  char    line[ASCII_BUF_SIZE];

  // Called from handleTeensyCommands() for each printable/newline byte
  void push(char c) {
    if (c == '\n') {
      buf[head] = '\0';
      memcpy(line, buf, head + 1);
      lineReady = true;
      head = 0;
    } else if (c != '\r' && head < ASCII_BUF_SIZE - 1) {
      buf[head++] = c;
    }
    // Overflow: silently discard (line too long → not a valid CAL command)
  }
};

// ══════════════════════════════════════════════════════════════════════════
//  ⑨  GLOBALS
// ══════════════════════════════════════════════════════════════════════════

// ── Calibration ──────────────────────────────────────────────────────────
static Preferences prefs;   // ESP32 NVS – replaces EEPROM, wear-levelled

// Compiled defaults (fallback when NVS has no saved data)
const CameraThresholds DEFAULT_THRESHOLDS = {
  DEF_YEL_H_MIN, DEF_YEL_H_MAX, DEF_YEL_S_MIN, DEF_YEL_S_MAX,
  DEF_YEL_V_MIN, DEF_YEL_V_MAX,
  DEF_BLU_H_MIN, DEF_BLU_H_MAX, DEF_BLU_S_MIN, DEF_BLU_S_MAX,
  DEF_BLU_V_MIN, DEF_BLU_V_MAX,
  DEF_WHITE_S_MAX, DEF_WHITE_V_MIN,
  DEF_BLACK_S_MAX, DEF_BLACK_V_MAX
};
static CameraThresholds activeThr;   // Runtime thresholds (calibrated)
// [v5] HSV sample for the calibration UI: center-box average + yellow/blue pixel
// counts from the last scanGoal. Updated every camera tick; read by the /hsv route.
volatile uint8_t  g_smpH = 0, g_smpS = 0, g_smpV = 0;
volatile uint16_t g_smpY = 0, g_smpB = 0;

// ── WiFi / ESP-NOW ───────────────────────────────────────────────────────
// [v2.2] WiFiUDP removed; replaced by ESP-NOW for inter-robot comms.
static WifiState wifiState = WS_IDLE;
static bool      isWifiESP = false;   // True when ESP-NOW bridge is active
static uint8_t   partnerID = 0x00;    // Partner robot ESP ID (0=not found)

// [v2.2 NEW] ESP-NOW peer tracking
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t   partnerMAC[6] = {0};  // Set when partner identified
static uint8_t   g_myMAC[6]    = {0};  // [v4] this ESP's own STA MAC (for identity tie-break)
static bool      partnerPeerAdded = false;

// [v2.2 NEW] Inter-task comm: ESP-NOW recv callback → main loop
// (Serial.write from the WiFi-task callback is unsafe; we queue and process
//  in wifiTask() from the main loop.)
static volatile bool    g_pairingPending = false;
static uint8_t          g_pendingPartnerID = 0;
static uint8_t          g_pendingPartnerMAC[6] = {0};

#define RX_QUEUE_DEPTH 4
struct EspNowRxItem {
  uint8_t len;
  uint8_t data[64];
};
static volatile EspNowRxItem g_rxQueue[RX_QUEUE_DEPTH];
static volatile uint8_t      g_rxHead = 0;   // producer (callback)
static volatile uint8_t      g_rxTail = 0;   // consumer (main loop)

// ── Mount angle ───────────────────────────────────────────────────────────
static int mountAngle = 0;   // 0/90/180/270 – read from GPIO straps in setup()

// ── Camera-position ID ─────────────────────────────────────────────────────
// [FIX] Was a #define (frozen to 1 on every unit because the FW image is
//       identical). Now derived from the SAME straps as mountAngle in setup().
//       0x01 here is only a fallback before setup() runs.
static uint8_t ESP_UNIQUE_ID = 0x03;   // 1=fwd/0° 2=right/90° 3=rear/180° 4=left/270°

// ── Timing ────────────────────────────────────────────────────────────────
static uint32_t lastTick   = 0;
static uint32_t lastBcast  = 0;

// ── ASCII line buffer ─────────────────────────────────────────────────────
static AsciiLineBuf asciiBuf;

static bool camOK = false;
static int camErr = 0;

// ══════════════════════════════════════════════════════════════════════════
//  [v3]  TEST-MODE CONSOLE GLOBALS   (active on the forward ESP only)
// ══════════════════════════════════════════════════════════════════════════
static AsyncWebServer   g_http(TEST_HTTP_PORT);
static AsyncEventSource g_sse("/events");
static uint8_t  g_robotState   = ROBOT_STATE_READY;   // updated by CMD_ROBOT_STATE
static bool     g_apUp         = false;
static uint32_t g_lastActivity = 0;
static char     g_lastTlm[160] = "TLM:0,0,0,-1,0,0,0,0,-1";
static bool     g_relaySessionForA = false;           // B-side: mirror TLM back to A

#define RELAY_Q_DEPTH 6
struct RelayQItem { uint8_t type; uint8_t len; char data[RELAY_MAX_PAYLOAD + 1]; };
static volatile RelayQItem g_relayQ[RELAY_Q_DEPTH];
static volatile uint8_t    g_relayHead = 0, g_relayTail = 0;

// [v4] Robot-identity tie-break byte reported to the Teensy on EVT_PARTNER_FOUND
//      (0xB0) and CMD_WIFI_STATUS (0xA3). Identity is MAC-based (both forward
//      ESPs share ESP_UNIQUE_ID==1, so the old per-camera id could not tell the
//      two robots apart). partnerID/partnerMAC stay as-is internally for the
//      relay path; only the byte we *report* to the Teensy changes meaning:
//        0 = not paired
//        1 = THIS robot holds the higher MAC  -> attacker on an auction tie
//        2 = THIS robot holds the lower  MAC  -> goalie   on an auction tie
static uint8_t partnerRankByte() {
  if (partnerID == 0x00 || !partnerPeerAdded) return 0x00;        // not paired yet
  return (memcmp(g_myMAC, partnerMAC, 6) > 0) ? 1 : 2;            // higher MAC -> 1
}

// forward declarations
void testConsoleSetup();
void testConsoleLoop();
void testStartAP();
void testStopAP();
void testHandleCmd(AsyncWebServerRequest* req);
void testOnTelemetry(const char* line);
void emitToTeensy(const char* line);
void relaySend(uint8_t type, const char* payload);

// [v2.2 NEW] Forward declarations for ESP-NOW callbacks
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

// ══════════════════════════════════════════════════════════════════════════
//  ⑩  FUNCTION PROTOTYPES
// ══════════════════════════════════════════════════════════════════════════
bool      initCamera();
int       readMountAngle();
void      saveCalToNVS();
bool      loadCalFromNVS();
void      resetCalToDefault();
void      printCalStatus();
bool      parseHSV6(const char* s, uint8_t* out);  // parse 6 uint8 values
bool      parseUint8_2(const char* s, uint8_t* a, uint8_t* b); // parse 2 values
void      processCalCommand(const char* line);
void      handleTeensyCommands();
Detection detectObjects(camera_fb_t* fb);
Detection scanGoal(camera_fb_t* fb);
Detection scanWhiteLine(camera_fb_t* fb);
Detection scanBlackLine(camera_fb_t* fb);
void      sampleCenterHSV(camera_fb_t* fb);   // [v5] center-box HSV for calib UI
void      rgb565ToHSV(uint16_t px, uint8_t& h, uint8_t& s, uint8_t& v);
int8_t    pixelToAngle(int cx);
uint8_t   blobToDistance(long pixelCount, long maxPixels);
void      sendPacket(const Detection& d);
void      initWiFi();
void      wifiTask();
void      handleFrameBmp(AsyncWebServerRequest* req);

// ══════════════════════════════════════════════════════════════════════════
//  ⑪  SETUP
// ══════════════════════════════════════════════════════════════════════════
void setup() {
  TEENSY.begin(UART_BAUD);   // [v3] UART0 (GPIO43/44) -> Teensy
  DBG.begin(115200);         // [v3] USB-CDC -> debug monitor
  delay(300);

  // ── 1. Read mount angle from PCB strap pins ───────────────────────────
  //  Internal pull-up: open = HIGH = 0,  shorted-to-GND = LOW = 1 (inverted)
  pinMode(GPIO_STRAP_A, INPUT_PULLUP);
  pinMode(GPIO_STRAP_B, INPUT_PULLUP);
  delay(10);                             // Let pins settle
  mountAngle = readMountAngle();
  // [FIX] Derive the camera ID from the SAME straps (single source of truth):
  //   0°→1, 90°→2, 180°→3, 270°→4. Keeps 0x00 reserved as the "unset" value.
  ESP_UNIQUE_ID = (uint8_t)(mountAngle / 90) + 1;
  // Release pins immediately → no pull-up current drain for rest of runtime
  pinMode(GPIO_STRAP_A, INPUT);
  pinMode(GPIO_STRAP_B, INPUT);

  DBG.printf(
    "\n[RoboCap] ID=0x%02X  mount=%d deg\n",
    ESP_UNIQUE_ID, mountAngle);

  // ── 2. Load calibration from NVS (fallback to compiled defaults) ──────
  if (loadCalFromNVS()) {
    DBG.println("INFO:CAL_LOADED_NVS");
  } else {
    resetCalToDefault();
    DBG.println("WARN:CAL_USING_DEFAULTS");
  }

  // ── 3. Initialise camera ──────────────────────────────────────────────
  camOK = initCamera();

  if (!camOK)
  {
    DBG.println("[FATAL] Camera init failed. Vision disabled.");
  }
  else
  {
    DBG.println("[OK] Camera ready. 100 ms loop starting.");
  }

  // ── 4. Auto-start WiFi if this is the forward ESP (mountAngle == 0°) ──
  //  The forward ESP is the natural WiFi bridge: it faces the opponent goal
  //  and has clear line-of-sight to the partner robot across the field.
  //  Teensy CMD_WIFI_START/STOP can still override this at runtime.
  if (mountAngle == 0) {
    isWifiESP = true;
    wifiState = WS_SCANNING;
    partnerID = 0x00;
    initWiFi();
    DBG.println("[WiFi] Auto-started (forward ESP, mountAngle=0)");
    testConsoleSetup();   // [v3] TEST web console (forward ESP only)
  }

  lastTick = millis();
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑫  MAIN LOOP
// ══════════════════════════════════════════════════════════════════════════
void loop() {
  // Always: process Teensy commands (binary + ASCII) immediately
  handleTeensyCommands();

  // WiFi state machine – runs only on the bridge ESP
  if (isWifiESP) wifiTask();
  if (mountAngle == 0) testConsoleLoop();   // [v3]

  // ── 100 ms camera sensing tick ────────────────────────────────────────
  if ((millis() - lastTick) < TICK_MS) return;
  lastTick = millis();
  if (!camOK) return;
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    // Camera busy / PSRAM allocation failed – skip this tick silently.
    // [v5] No no-detect flood here: writing to Serial0 every tick from the
    // loop collides with command writes from the async-web task and corrupts
    // both. The Teensy now learns forwardPortIdx from the first ASCII command
    // instead, so TEST/TLM work without any vision traffic.
    return;
  }

  Detection result = detectObjects(fb);
  sampleCenterHSV(fb);        // [v5] keep center-box HSV fresh for the calibration UI
  esp_camera_fb_return(fb);   // Return buffer immediately to free PSRAM
  sendPacket(result);
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑬  MOUNT ANGLE FROM GPIO STRAPS
//
//  INPUT_PULLUP: open pad = HIGH → bit 0.  Shorted-to-GND pad = LOW → bit 1.
//  Inverted so "no solder work" = forward (0°), the most common position.
// ═════════════════════════════════════════════════════════════════════════
int readMountAngle() {
  delay(10);
  uint8_t a    = !digitalRead(GPIO_STRAP_A);   // invert: GND=1, open=0
  uint8_t b    = !digitalRead(GPIO_STRAP_B);
  uint8_t code = (b << 1) | a;                 // 0…3
  return (int)code * 90;                        // 0, 90, 180, 270
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑭  CAMERA INITIALIZATION
// ══════════════════════════════════════════════════════════════════════════
bool initCamera() {
  // [v3] PSRAM is mandatory: QVGA RGB565 x2 framebuffers (~300 KB) live in PSRAM.
  // If this fails, it is almost always the board setting, NOT the code:
  //   Arduino IDE -> Tools -> PSRAM -> "OPI PSRAM"   (XIAO ESP32-S3 Sense)
  if (!psramFound()) {
    DBG.println("[FATAL] PSRAM not found / disabled.");
    DBG.println("        Fix: Arduino IDE -> Tools -> PSRAM -> \"OPI PSRAM\", then re-flash.");
    return false;
  }

  camera_config_t cfg;
  cfg.ledc_channel   = LEDC_CHANNEL_0;
  cfg.ledc_timer     = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz  = 20000000;
  cfg.pixel_format  = PIXFORMAT_RGB565;   // Required for HSV color math
  cfg.frame_size    = FRAMESIZE_QVGA;     // 320×240
  cfg.fb_count      = 2;                  // Double-buffer for throughput
  cfg.grab_mode     = CAMERA_GRAB_LATEST; // Always use freshest frame
  cfg.fb_location   = CAMERA_FB_IN_PSRAM;// XIAO S3 Sense has 8 MB PSRAM

  esp_err_t err = esp_camera_init(&cfg);
  camErr = (int)err;

  if (err != ESP_OK)
  {
    DBG.printf("[FATAL] esp_camera_init failed: 0x%X\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s,  1);    // Slight boost for indoor competition lighting
  s->set_saturation(s,  1);    // Richer color aids goal detection
  s->set_contrast(s,    0);
  s->set_whitebal(s,    1);    // Auto white-balance ON
  s->set_awb_gain(s,    1);
  s->set_exposure_ctrl(s, 1);  // Auto exposure ON
  s->set_aec2(s,        1);    // Extended AEC
  s->set_gain_ctrl(s,   1);    // Auto gain ON
  s->set_gainceiling(s, (gainceiling_t)2);
  s->set_colorbar(s,    0);    // No test pattern
  s->set_hmirror(s,     0);    // Set 1 if image appears L/R mirrored on PCB
  s->set_vflip(s,       0);    // Set 1 if image appears upside-down on PCB
  return true;
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑮  NVS CALIBRATION STORAGE  (Preferences – ESP32 wear-levelled flash)
//
//  Mirrors the EEPROM pattern in color_calibration_and_UART_FW.ino (Teensy)
//  but uses ESP32 Preferences (key-value NVS) which is safer than raw EEPROM.
//  The namespace "rcal" is shared across all 4 ESPs (each has its own flash).
// ══════════════════════════════════════════════════════════════════════════
void saveCalToNVS() {
  prefs.begin("rcal", false);   // false = read-write
  prefs.putBytes("thr", &activeThr, sizeof(activeThr));
  prefs.end();
  TEENSY.println("OK:SAVE");
}

bool loadCalFromNVS() {
  prefs.begin("rcal", true);    // true = read-only
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

// ══════════════════════════════════════════════════════════════════════════
//  ⑯  CALIBRATION STATUS PRINT
//
//  Mirrors printCalStatus() in color_calibration_and_UART_FW.ino (Teensy).
//  Output is ASCII on Serial (Teensy receives and can display on USB console).
// ══════════════════════════════════════════════════════════════════════════
void printCalStatus() {
  TEENSY.println("=== CAM CALIBRATION STATUS ===");
  TEENSY.printf("YELLOW H:[%d-%d] S:[%d-%d] V:[%d-%d]\n",
    activeThr.yelHMin, activeThr.yelHMax,
    activeThr.yelSMin, activeThr.yelSMax,
    activeThr.yelVMin, activeThr.yelVMax);
  TEENSY.printf("BLUE   H:[%d-%d] S:[%d-%d] V:[%d-%d]\n",
    activeThr.bluHMin, activeThr.bluHMax,
    activeThr.bluSMin, activeThr.bluSMax,
    activeThr.bluVMin, activeThr.bluVMax);
  TEENSY.printf("WHITE  S_max:%d  V_min:%d\n",
    activeThr.whiteSMax, activeThr.whiteVMin);
  TEENSY.printf("BLACK  S_max:%d  V_max:%d\n",
    activeThr.blackSMax, activeThr.blackVMax);
  TEENSY.println("==============================");
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑰  CALIBRATION COMMAND PARSER
//
//  Called when a complete '\n'-terminated ASCII line arrives from Teensy.
//  Format: mirrors color_calibration_and_UART_FW.ino §CALIBRATION PROTOCOL.
//
//  Commands:
//    CAL:YELLOW:hMin,hMax,sMin,sMax,vMin,vMax
//    CAL:BLUE:hMin,hMax,sMin,sMax,vMin,vMax
//    CAL:WHITE:sMax,vMin
//    CAL:BLACK:sMax,vMax
//    CAL:SAVE
//    CAL:LOAD
//    CAL:RESET
//    CAL:STATUS
// ══════════════════════════════════════════════════════════════════════════

// Parse 6 comma-separated uint8 values. Returns true on success.
bool parseHSV6(const char* s, uint8_t* out) {
  char tmp[64];
  strncpy(tmp, s, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = '\0';
  uint8_t idx = 0;
  char* tok = strtok(tmp, ",");
  while (tok && idx < 6) {
    int v = atoi(tok);
    if (v < 0 || v > 255) return false;
    out[idx++] = (uint8_t)v;
    tok = strtok(nullptr, ",");
  }
  return (idx == 6);
}

// Parse 2 comma-separated uint8 values. Returns true on success.
bool parseUint8_2(const char* s, uint8_t* a, uint8_t* b) {
  char tmp[32];
  strncpy(tmp, s, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = '\0';
  char* tok = strtok(tmp, ",");
  if (!tok) return false;
  *a = (uint8_t)atoi(tok);
  tok = strtok(nullptr, ",");
  if (!tok) return false;
  *b = (uint8_t)atoi(tok);
  return true;
}

void processCalCommand(const char* rawLine) {
  // All calibration commands start with "CAL:"
  if (strncmp(rawLine, "CAL:", 4) != 0) return;

  const char* cmd = rawLine + 4;   // advance past "CAL:"

  // ── CAL:SAVE ──────────────────────────────────────────────────────────
  if (strcmp(cmd, "SAVE") == 0) {
    saveCalToNVS();    // prints OK:SAVE internally
    return;
  }

  // ── CAL:LOAD ──────────────────────────────────────────────────────────
  if (strcmp(cmd, "LOAD") == 0) {
    if (loadCalFromNVS()) {
      TEENSY.println("OK:LOAD");
      printCalStatus();
    } else {
      TEENSY.println("ERR:NO_NVS_DATA");
    }
    return;
  }

  // ── CAL:RESET ─────────────────────────────────────────────────────────
  if (strcmp(cmd, "RESET") == 0) {
    resetCalToDefault();   // prints OK:RESET internally
    return;
  }

  // ── CAL:STATUS ────────────────────────────────────────────────────────
  if (strcmp(cmd, "STATUS") == 0) {
    printCalStatus();
    return;
  }

  // ── CAL:YELLOW:hMin,hMax,sMin,sMax,vMin,vMax ─────────────────────────
  if (strncmp(cmd, "YELLOW:", 7) == 0) {
    uint8_t v[6];
    if (parseHSV6(cmd + 7, v)) {
      activeThr.yelHMin = v[0]; activeThr.yelHMax = v[1];
      activeThr.yelSMin = v[2]; activeThr.yelSMax = v[3];
      activeThr.yelVMin = v[4]; activeThr.yelVMax = v[5];
      TEENSY.println("OK:CAL:YELLOW");
    } else {
      TEENSY.println("ERR:PARSE_YELLOW");
    }
    return;
  }

  // ── CAL:BLUE:hMin,hMax,sMin,sMax,vMin,vMax ───────────────────────────
  if (strncmp(cmd, "BLUE:", 5) == 0) {
    uint8_t v[6];
    if (parseHSV6(cmd + 5, v)) {
      activeThr.bluHMin = v[0]; activeThr.bluHMax = v[1];
      activeThr.bluSMin = v[2]; activeThr.bluSMax = v[3];
      activeThr.bluVMin = v[4]; activeThr.bluVMax = v[5];
      TEENSY.println("OK:CAL:BLUE");
    } else {
      TEENSY.println("ERR:PARSE_BLUE");
    }
    return;
  }

  // ── CAL:WHITE:sMax,vMin ───────────────────────────────────────────────
  if (strncmp(cmd, "WHITE:", 6) == 0) {
    uint8_t a, b;
    if (parseUint8_2(cmd + 6, &a, &b)) {
      activeThr.whiteSMax = a;
      activeThr.whiteVMin = b;
      TEENSY.println("OK:CAL:WHITE");
    } else {
      TEENSY.println("ERR:PARSE_WHITE");
    }
    return;
  }

  // ── CAL:BLACK:sMax,vMax ───────────────────────────────────────────────
  if (strncmp(cmd, "BLACK:", 6) == 0) {
    uint8_t a, b;
    if (parseUint8_2(cmd + 6, &a, &b)) {
      activeThr.blackSMax = a;
      activeThr.blackVMax = b;
      TEENSY.println("OK:CAL:BLACK");
    } else {
      TEENSY.println("ERR:PARSE_BLACK");
    }
    return;
  }

  TEENSY.println("ERR:UNKNOWN_CAL_CMD");
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑱  UART COMMAND HANDLER  (called every loop iteration, non-blocking)
//
//  Dual-mode parser on the same Serial port:
//    Bytes ≥ 0x80  → binary command (processed immediately)
//    Bytes 0x20–0x7E, '\r', '\n' → ASCII character (buffered for CAL:)
//    Bytes < 0x20 (except \r \n) → discard (noise / control chars)
//
//  This clean split works because all binary command bytes were deliberately
//  assigned to the 0x80–0xFF range, and all ASCII printable characters are
//  in the 0x20–0x7E range. There is zero overlap.
// ══════════════════════════════════════════════════════════════════════════
void handleTeensyCommands() {
  while (TEENSY.available() > 0) {
    uint8_t b = (uint8_t)TEENSY.peek();

    // ── BINARY COMMAND ────────────────────────────────────────────────
    if (b >= 0x80) {
      TEENSY.read();   // consume the command byte
      switch (b) {

        case CMD_ROBOT_STATE: {   // [v3] +1 byte: ROBOT_STATE_READY/GAME/TEST
          uint32_t t0 = micros();
          while (!TEENSY.available() && (micros() - t0) < 2000) { }
          if (TEENSY.available()) g_robotState = TEENSY.read();
          break;
        }

        case CMD_QUERY_ID:
          // Teensy maps UART ports → ESP IDs at startup
          TEENSY.write((uint8_t)ESP_UNIQUE_ID);
          break;

        case CMD_WIFI_START:
          // Teensy overrides/confirms WiFi bridge role
          if (!isWifiESP) {
            isWifiESP = true;
            wifiState = WS_SCANNING;
            partnerID = 0x00;
            initWiFi();
          }
          break;

        case CMD_WIFI_STOP:
          // [v2.2] ESP-NOW: deinit instead of WiFi.disconnect
          wifiState = WS_STOPPED;
          isWifiESP = false;
          partnerID = 0x00;
          memset(partnerMAC, 0, 6);
          partnerPeerAdded = false;
          esp_now_deinit();
          // Keep WIFI_STA mode active (no .mode(WIFI_OFF)) so a later
          // CMD_WIFI_START can restart ESP-NOW quickly without re-init radio.
          DBG.println("INFO:ESPNOW_STOPPED");
          break;

        case CMD_WIFI_STATUS:
          // [v4] Teensy polls partner status; reply MAC-rank tie-break byte
          //   0 = not paired, 1 = we hold higher MAC, 2 = we hold lower MAC
          TEENSY.write(partnerRankByte());
          break;

        case CMD_RELAY_DATA: {
          uint32_t t0 = millis();

          while (!TEENSY.available() && millis() - t0 < 5) { }
          if (!TEENSY.available()) break;

          uint8_t len = TEENSY.read();

          if (len == 0 || len > 64)
            break;

          uint8_t payload[64];
          uint8_t got = 0;
          t0 = millis();

          while (got < len && millis() - t0 < 10)
          {
            if (TEENSY.available())
              payload[got++] = TEENSY.read();
          }

          if (got == len && wifiState == WS_PAIRED && partnerPeerAdded)
          {
            esp_now_send(partnerMAC, payload, len);
          }

          break;
        }

        default:
          break;
      }

    // ── ASCII CHARACTER ───────────────────────────────────────────────
    } else if (b >= 0x20 || b == '\r' || b == '\n') {
      char c = (char)TEENSY.read();
      asciiBuf.push(c);
      // Process completed line immediately in same loop iteration
      if (asciiBuf.lineReady) {
        asciiBuf.lineReady = false;
        if (strncmp(asciiBuf.line, "CAL:", 4) == 0) processCalCommand(asciiBuf.line);
        else testOnTelemetry(asciiBuf.line);   // [v3] TLM:/IR:/VIS:/CMP:/STA:/LOG:/ACK:
      }

    } else {
      TEENSY.read();   // Discard control/noise byte (< 0x20, not CR/LF)
    }
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑲  MASTER DETECTION  – priority: goal > white line > black line
//
//  Single frame is scanned in three passes (goal, white, black).
//  First successful detection wins — lower-priority scans are skipped.
//  Rationale: goals are the primary navigation target; white lines are
//  high-priority boundary warnings; black lines are position hints only.
// ══════════════════════════════════════════════════════════════════════════
Detection detectObjects(camera_fb_t* fb) {
  Detection g = scanGoal(fb);
  if (g.detected) return g;

  Detection w = scanWhiteLine(fb);
  if (w.detected) return w;

  return scanBlackLine(fb);   // detected=false returned if nothing seen
}

// ══════════════════════════════════════════════════════════════════════════
//  ⑳  GOAL DETECTION  (yellow or blue color blob)
//
//  Scan ROI: lower 2/3 of frame (rows H/3 … H).
//    Top 1/3 excluded: audience shirts, ceiling lights, overhead noise
//    (Rule 3.1.4 warns about visible colors above walls).
//
//  Both colors are accumulated in a single pixel pass.
//  Larger valid blob wins. ESP does NOT distinguish our goal vs opponent
//  goal — the Teensy resolves this using heading + pre-match side assignment.
//
//  Distance proxy: 18 000 pixels ≈ "very close" (robot ~15 cm from goal).
//  Calibrate MAX_GOAL_PIX empirically on your actual field.
// ══════════════════════════════════════════════════════════════════════════
Detection scanGoal(camera_fb_t* fb) {
  Detection res = {false, OBJ_GOAL, 0, 0};
  uint16_t* pix = (uint16_t*)fb->buf;
  const int W = fb->width, H = fb->height;

  long ySumX = 0, yCount = 0;   // Yellow blob accumulator
  long bSumX = 0, bCount = 0;   // Blue blob accumulator

  for (int row = H / 3; row < H; row++) {
    for (int col = 0; col < W; col++) {
      // OV2640 sends RGB565 big-endian; swap bytes to get standard order
      uint16_t px = pix[row * W + col];
      px = (uint16_t)((px >> 8) | (px << 8));

      uint8_t h, s, v;
      rgb565ToHSV(px, h, s, v);

      if (h >= activeThr.yelHMin && h <= activeThr.yelHMax &&
          s >= activeThr.yelSMin && s <= activeThr.yelSMax &&
          v >= activeThr.yelVMin && v <= activeThr.yelVMax) {
        ySumX += col;  yCount++;
      }
      if (h >= activeThr.bluHMin && h <= activeThr.bluHMax &&
          s >= activeThr.bluSMin && s <= activeThr.bluSMax &&
          v >= activeThr.bluVMin && v <= activeThr.bluVMax) {
        bSumX += col;  bCount++;
      }
    }
  }

  // Select larger blob; both must exceed noise threshold
  g_smpY = (uint16_t)(yCount > 65535L ? 65535L : yCount);   // [v5] expose counts to /hsv
  g_smpB = (uint16_t)(bCount > 65535L ? 65535L : bCount);
  long bestCount = 0, bestSumX = 0;
  uint8_t goalCol = OBJ_GOAL;                 // [v5] which colour won
  if (yCount >= MIN_GOAL_PIX && yCount >= bCount) {
    bestCount = yCount;  bestSumX = ySumX;  goalCol = OBJ_GOAL_YELLOW;
  } else if (bCount >= MIN_GOAL_PIX) {
    bestCount = bCount;  bestSumX = bSumX;  goalCol = OBJ_GOAL_BLUE;
  }
  if (bestCount == 0) return res;

  res.detected = true;
  res.objType  = goalCol;                     // [v5] OBJ_GOAL_YELLOW / OBJ_GOAL_BLUE
  res.angle    = pixelToAngle((int)(bestSumX / bestCount));
  res.distance = blobToDistance(bestCount, 18000L);
  return res;
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉑  WHITE LINE DETECTION  (field boundary + penalty area, §4/§7)
//
//  Scan ROI: bottom half of frame (rows H/2 … H).
//  Lines are on the ground surface directly ahead of the robot.
//  White = high V (bright), very low S (achromatic). Any hue qualifies.
//  Centroid gives dominant line direction.
// ══════════════════════════════════════════════════════════════════════════
Detection scanWhiteLine(camera_fb_t* fb) {
  Detection res = {false, OBJ_LINE_WHITE, 0, 0};
  uint16_t* pix = (uint16_t*)fb->buf;
  const int W = fb->width, H = fb->height;

  long sumX = 0, count = 0;

  for (int row = H / 2; row < H; row++) {
    for (int col = 0; col < W; col++) {
      uint16_t px = pix[row * W + col];
      px = (uint16_t)((px >> 8) | (px << 8));
      uint8_t h, s, v;
      rgb565ToHSV(px, h, s, v);
      if (s <= activeThr.whiteSMax && v >= activeThr.whiteVMin) {
        sumX += col;  count++;
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
//  ㉒  BLACK LINE DETECTION  (center circle §6 + neutral spots §5)
//
//  Scan ROI: middle band only (rows H/3 … 2H/3).
//
//  ROI rationale:
//    Top third (0 … H/3)  : dominated by matte-black walls (§2.0.1).
//      Excluding it prevents wall false-positives.
//    Bottom third (2H/3…H): reserved for white-line scan above; also
//      near-ground grazing pixels are noisy.
//    Middle band            : center circle (60 cm Ø) appears here when
//      the robot is anywhere in the mid-field zone.
//
//  Neutral spots (§5): 1 cm diameter — too small for reliable detection
//  at typical game distances. Center circle is the primary useful feature.
//
//  If wall false-positives persist, raise MIN_BLACK_PIX.
// ══════════════════════════════════════════════════════════════════════════
Detection scanBlackLine(camera_fb_t* fb) {
  Detection res = {false, OBJ_LINE_BLACK, 0, 0};
  uint16_t* pix = (uint16_t*)fb->buf;
  const int W = fb->width, H = fb->height;

  long sumX = 0, count = 0;

  for (int row = H / 3; row < (2 * H) / 3; row++) {
    for (int col = 0; col < W; col++) {
      uint16_t px = pix[row * W + col];
      px = (uint16_t)((px >> 8) | (px << 8));
      uint8_t h, s, v;
      rgb565ToHSV(px, h, s, v);
      if (s <= activeThr.blackSMax && v <= activeThr.blackVMax) {
        sumX += col;  count++;
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
//  ㉓  RGB565 → HSV  (integer arithmetic, no floating point)
//
//  H: 0–179  (degrees/2, OpenCV convention)
//  S: 0–255
//  V: 0–255
// ══════════════════════════════════════════════════════════════════════════
void rgb565ToHSV(uint16_t px, uint8_t& h, uint8_t& s, uint8_t& v) {
  uint8_t r = ((px >> 11) & 0x1F) << 3;   // 5-bit → 8-bit
  uint8_t g = ((px >>  5) & 0x3F) << 2;   // 6-bit → 8-bit
  uint8_t b = ((px >>  0) & 0x1F) << 3;   // 5-bit → 8-bit

  uint8_t maxC  = max(r, max(g, b));
  uint8_t minC  = min(r, min(g, b));
  int     delta = (int)maxC - (int)minC;

  v = maxC;
  s = (maxC == 0) ? 0 : (uint8_t)((long)delta * 255L / maxC);
  if (delta == 0) { h = 0; return; }

  int hue;
  if      (maxC == r) hue = 60 * ((int)g - (int)b) / delta;
  else if (maxC == g) hue = 60 * ((int)b - (int)r) / delta + 120;
  else                hue = 60 * ((int)r - (int)g) / delta + 240;
  if (hue < 0) hue += 360;
  h = (uint8_t)(hue / 2);   // Compress 0–360 → 0–179
}

// [v5] Average H,S,V over the central 1/3 box of the frame -> calibration UI.
//  The user centers the goal in view, samples, and reads the real camera HSV.
void sampleCenterHSV(camera_fb_t* fb) {
  uint16_t* pix = (uint16_t*)fb->buf;
  const int W = fb->width, H = fb->height;
  long sh=0, ss=0, sv=0, cnt=0;
  for (int row = H/3; row < 2*H/3; row++)
    for (int col = W/3; col < 2*W/3; col++) {
      uint16_t px = pix[row*W+col];
      px = (uint16_t)((px>>8)|(px<<8));               // RGB565 byte-swap (as in scanGoal)
      uint8_t h,s,v; rgb565ToHSV(px,h,s,v);
      sh+=h; ss+=s; sv+=v; cnt++;
    }
  if (cnt) { g_smpH=(uint8_t)(sh/cnt); g_smpS=(uint8_t)(ss/cnt); g_smpV=(uint8_t)(sv/cnt); }
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉔  PIXEL CENTROID → CAMERA-LOCAL ANGLE  (signed degrees, int8_t)
//
//  cx = 0       → –(FOV/2) = –60°  (left edge of frame)
//  cx = W/2     →         0°       (camera optical axis)
//  cx = W–1     → +(FOV/2) = +60°  (right edge of frame)
//
//  Output is camera-local, NOT robot-relative.
//  Teensy converts:  robotAngle = cameraAngle + MOUNT[espID]
//  (wrap result to –180…+180 on Teensy side)
// ══════════════════════════════════════════════════════════════════════════
int8_t pixelToAngle(int cx) {
  float norm  = (float)(cx - FRAME_W / 2) / (float)(FRAME_W / 2);
  float angle = norm * (CAM_FOV_DEG / 2.0f);   // –60 … +60
  return (int8_t)constrain((int)angle, -128, 127);
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉕  BLOB SIZE → DISTANCE PROXY  (0 = far/small … 255 = close/large)
//
//  maxPixels (object-specific, tune empirically):
//    Goals       : 18 000  (large colored area, robot ~15–20 cm from goal)
//    White lines :  8 000  (wide line, robot near boundary)
//    Black lines :  5 000  (thin center-circle marker)
// ══════════════════════════════════════════════════════════════════════════
uint8_t blobToDistance(long pixelCount, long maxPixels) {
  return (uint8_t)(constrain(pixelCount, 0L, maxPixels) * 255L / maxPixels);
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉖  SEND UART PACKET  (ESP → Teensy, fire-and-forget, no ACK)
//
//  Detection (4 bytes):
//    [0] ESP_UNIQUE_ID
//    [1] objType    0x00=goal  0x01=white line  0x02=black line  0x03=yellow goal  0x04=blue goal
//    [2] angle      camera-local int8 (Teensy: cast to int8_t; add MOUNT[id])
//    [3] distance   0=far … 255=close
//
//  No detection (2 bytes):
//    [0] ESP_UNIQUE_ID
//    [1] 0xFF
//
//  Teensy parse rule:
//    read byte[0] → ID (validate if desired)
//    read byte[1]:
//      0xFF        → no detection (done, 2 bytes)
//      0xB0        → partner found; read 1 more byte = partnerID
//      0xB1        → WiFi data;    read <len> byte, then <len> bytes
//      0x00/01/02  → detection;    read 2 more bytes (angle, distance)
// ══════════════════════════════════════════════════════════════════════════
void sendPacket(const Detection& d) {
  // TODO(MUTEX, before competition): this runs in loop(), while emitToTeensy()
  // runs in the AsyncTCP task. When the camera works during TEST, vision packets
  // here and command bytes there both write TEENSY (Serial0) and can interleave,
  // corrupting both. Guard all TEENSY writes (here, emitToTeensy, and the ESP-NOW
  // event writes) with a single FreeRTOS mutex so each frame/command is atomic.
  if (d.detected) {
    TEENSY.write((uint8_t)ESP_UNIQUE_ID);
    TEENSY.write((uint8_t)d.objType);
    TEENSY.write((uint8_t)d.angle);      // Teensy: int8_t a = (int8_t)byte;
    TEENSY.write((uint8_t)d.distance);
  } else {
    TEENSY.write((uint8_t)ESP_UNIQUE_ID);
    TEENSY.write((uint8_t)PACKET_NO_DETECT);
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉗  WIFI / ESP-NOW INITIALIZATION  [v2.2 REWRITTEN]
//
//  ESP-NOW: Espressif-proprietary peer-to-peer protocol on the WiFi radio.
//  - Symmetric (no AP/STA); both robots run identical code.
//  - Connectionless; no session to lose, automatic re-pair on partner reboot.
//  - Lower latency than UDP-over-WiFi (~2-5 ms vs 10-30 ms).
//  - Coexists with SoftAP (used later for Test Mode smartphone UI).
//
//  Requires WIFI_STA mode (the radio must be on, but no actual connection
//  is made to any access point).
// ══════════════════════════════════════════════════════════════════════════
void initWiFi() {
  WiFi.mode(WIFI_AP_STA);   // [v3] AP_STA: TEST SoftAP coexists with ESP-NOW
  WiFi.disconnect();                  // make sure we don't try to associate
  delay(50);                          // brief settle for radio init

  if (esp_now_init() != ESP_OK) {
    DBG.println("ERR:ESPNOW_INIT_FAILED");
    isWifiESP = false;
    wifiState = WS_IDLE;
    return;
  }

  esp_now_register_recv_cb(onESPNowRecv);
  esp_now_register_send_cb(onESPNowSend);

  // Add broadcast peer (for WHO_AM_I discovery).
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = 0;                   // current channel
  peer.encrypt = false;
  esp_now_add_peer(&peer);            // idempotent; ignore "already exists"

  partnerPeerAdded = false;
  memset(partnerMAC, 0, 6);
  partnerID = 0x00;

  uint8_t myMac[6];
  WiFi.macAddress(myMac);
  memcpy(g_myMAC, myMac, 6);          // [v4] cache for the identity tie-break
  DBG.printf("INFO:ESPNOW_READY:%02X%02X%02X%02X%02X%02X\n",
                myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉘  ESP-NOW RECEIVE CALLBACK  [v2.2 NEW]
//
//  This runs in the WiFi-task context, NOT in the main loop. Two rules:
//    1. Keep it short and non-blocking.
//    2. Do NOT call Serial.write here (race with main loop's Serial.write).
//  Instead: queue events and let wifiTask() (main loop) drain the queue.
//
//  Note: signature differs between Arduino-ESP32 v2.x and v3.x.
//  This code uses the v3.x signature (esp_now_recv_info_t).
//  For v2.x, use:  void onESPNowRecv(const uint8_t *mac, const uint8_t *data, int len)
//  and replace `info->src_addr` with `mac`.
// ══════════════════════════════════════════════════════════════════════════
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  void onESPNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    const uint8_t *srcMac = info->src_addr;
#else
  void onESPNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
    const uint8_t *srcMac = mac;
#endif
  if (wifiState == WS_IDLE || wifiState == WS_STOPPED) return;
  if (len < 2 || len > 64) return;


  // Filter: ignore our own broadcasts (shouldn't happen, but safe).
  uint8_t myMac[6];
  WiFi.macAddress(myMac);
  if (memcmp(srcMac, myMac, 6) == 0) return;

  // [v3] TEST relay frames (magic 0x70 0x54) -> queue for main loop, not game data
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

  // ── DISCOVERY phase ────────────────────────────────────────────────────
  if (wifiState == WS_SCANNING) {
    // "WHO_AM_I:<id>" → reply "ROBOT_ID:<ourID>" to that MAC
    if (len >= 10 && memcmp(data, "WHO_AM_I:", 9) == 0) {
      // Add the sender as a unicast peer so we can reply to them
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, srcMac, 6);
      peer.channel = 0;
      peer.encrypt = false;
      esp_now_add_peer(&peer);        // ignore "already exists" error

      char reply[16];
      int rlen = snprintf(reply, sizeof(reply), "ROBOT_ID:%d", (int)ESP_UNIQUE_ID);
      esp_now_send(srcMac, (const uint8_t*)reply, rlen);
      return;
    }
    // "ROBOT_ID:<id>" → pairing complete
    if (len >= 10 && memcmp(data, "ROBOT_ID:", 9) == 0) {
      // Copy ID (we parse later in main loop where it's safer)
      char idStr[8] = {0};
      uint8_t copyLen = (len - 9 < 7) ? (len - 9) : 7;
      memcpy(idStr, &data[9], copyLen);
      g_pendingPartnerID = (uint8_t)atoi(idStr);
      memcpy(g_pendingPartnerMAC, srcMac, 6);
      g_pairingPending = true;        // main loop will finalize
      return;
    }
    // Unknown message during scanning – ignore.
    return;
  }

  // ── PAIRED phase: queue inbound data for main loop forwarding ─────────
  if (wifiState == WS_PAIRED) {
    // Accept only from our paired partner (security + sanity)
    if (memcmp(srcMac, partnerMAC, 6) != 0) return;

    uint8_t nextHead = (g_rxHead + 1) % RX_QUEUE_DEPTH;
    if (nextHead == g_rxTail) {
      // Queue full – drop packet (main loop is too slow). Cheap to recover
      // because Teensy expects unreliable transport at the app layer.
      return;
    }
    g_rxQueue[g_rxHead].len = (uint8_t)len;
    memcpy((void*)g_rxQueue[g_rxHead].data, data, len);
    g_rxHead = nextHead;
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉙  ESP-NOW SEND CALLBACK  [v2.2 NEW]  (diagnostics only; non-critical)
// ══════════════════════════════════════════════════════════════════════════
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 1, 0)
void onESPNowSend(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  (void)tx_info; (void)status;
#else
void onESPNowSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr; (void)status;
#endif
  // Intentionally minimal. We don't retry at this layer — application
  // protocol (Teensy ↔ Teensy at 20 Hz) is tolerant to occasional drops.
  // Hook for future debug:
  //   if (status != ESP_NOW_SEND_SUCCESS) { /* increment a counter */ }
}

// ══════════════════════════════════════════════════════════════════════════
//  ㉚  WIFI TASK  (inter-robot discovery + transparent relay)  [v2.2 REWRITTEN]
//
//  Called every loop() iteration when isWifiESP == true.
//
//  Responsibilities (everything time-driven / main-loop driven):
//    1. Periodic WHO_AM_I broadcast while WS_SCANNING.
//    2. Finalize pairing when callback flagged g_pairingPending.
//    3. Drain ESP-NOW RX queue → forward to Teensy as EVT_WIFI_DATA.
//    4. Forward CMD_RELAY_DATA (0xC0) from Teensy → partner via esp_now_send.
//
//  Teensy owns the inter-robot application protocol; the ESP is a
//  transparent ESP-NOW ↔ UART bridge once paired.
// ══════════════════════════════════════════════════════════════════════════
void wifiTask() {
  if (wifiState == WS_IDLE || wifiState == WS_STOPPED) return;

  // ── 1. Periodic WHO_AM_I broadcast while scanning ─────────────────────
  if (wifiState == WS_SCANNING && (millis() - lastBcast) >= WIFI_BCAST_MS) {
    lastBcast = millis();
    char msg[16];
    int mlen = snprintf(msg, sizeof(msg), "WHO_AM_I:%d", (int)ESP_UNIQUE_ID);
    esp_now_send(BROADCAST_MAC, (const uint8_t*)msg, mlen);
  }

  // ── 2. Finalize pairing (callback set g_pairingPending) ───────────────
  if (wifiState == WS_SCANNING && g_pairingPending) {
    g_pairingPending = false;
    partnerID = g_pendingPartnerID;
    memcpy(partnerMAC, (const void*)g_pendingPartnerMAC, 6);

    // Add partner as a permanent unicast peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, partnerMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);          // idempotent
    partnerPeerAdded = true;

    wifiState = WS_PAIRED;

    // Notify Teensy (async binary event) – safe to call Serial from main loop
    TEENSY.write((uint8_t)EVT_PARTNER_FOUND);
    TEENSY.write(partnerRankByte());      // [v4] 1=we hold higher MAC, 2=lower (tie-break)
    DBG.printf("INFO:PAIRED_WITH:0x%02X (MAC %02X%02X%02X%02X%02X%02X)\n",
                  partnerID,
                  partnerMAC[0], partnerMAC[1], partnerMAC[2],
                  partnerMAC[3], partnerMAC[4], partnerMAC[5]);
  }

  // ── 3. Drain RX queue → Teensy UART ───────────────────────────────────
  while (g_rxTail != g_rxHead) {
    uint8_t len = g_rxQueue[g_rxTail].len;
    TEENSY.write((uint8_t)EVT_WIFI_DATA);
    TEENSY.write(len);
    TEENSY.write((const uint8_t*)g_rxQueue[g_rxTail].data, len);
    g_rxTail = (g_rxTail + 1) % RX_QUEUE_DEPTH;
  }

  // ── 4. Forward Teensy relay data → partner robot via ESP-NOW ──────────
  // Teensy sends: 0xC0 <len> <bytes…> to relay to the other robot.
  // handleTeensyCommands() left CMD_RELAY_DATA on the wire intact via peek().
}

// ══════════════════════════════════════════════════════════════════════════
//  TEENSY 4.1 INTEGRATION REFERENCE
// ══════════════════════════════════════════════════════════════════════════
//
//  A. STARTUP SEQUENCE
//  ────────────────────
//  1. Send 0xA0 to each of the 4 Serial ports; read back 1 byte → ID.
//     Builds the runtime port→ID map.
//  2. Build ID→mountAngle lookup (matches PCB strap wiring):
//       const int MOUNT[5] = {0, 0, 90, 180, 270};  // index = ESP ID
//  3. Forward ESP (ID whose MOUNT=0) starts ESP-NOW automatically at boot.
//     Teensy may still send 0xA1/0xA2 to override if needed.
//  4. Poll 0xA3 (or wait for 0xB0 event) to get partner robot ID.
//  5. Encode role (goalkeeper / attacker) from partner ID and heading.
//
//  B. BINARY PACKET PARSING (per Serial port, every loop)
//  ─────────────────────────────────────────────────────────
//  byte0 = read()  → ESP_UNIQUE_ID
//  byte1 = read()
//    0xFF        → no detection (2 bytes consumed, done)
//    0xB0        → partner found; read 1 more byte = partnerID
//    0xB1        → ESP-NOW data; read <len>; read <len> bytes [v2.2: was UDP]
//    0x00/01/02  → detection;    read angle byte + distance byte
//
//  C. ANGLE CONVERSION ON TEENSY
//  ───────────────────────────────
//  int8_t cameraAngle = (int8_t)angleByte;
//  int    robotAngle  = (int)cameraAngle + MOUNT[espID];
//  while (robotAngle >  180) robotAngle -= 360;
//  while (robotAngle < -180) robotAngle += 360;
//  // robotAngle: 0=forward, +90=right, –90=left, ±180=rear
//
//  D. ASCII CALIBRATION WORKFLOW (via Teensy USB console or script)
//  ──────────────────────────────────────────────────────────────────
//  Teensy forwards ASCII lines from USB serial to ESP32 UART:
//    Serial1.println("CAL:STATUS");          → ESP prints current thresholds
//    Serial1.println("CAL:YELLOW:20,35,130,255,100,255"); → update yellow
//    Serial1.println("CAL:BLUE:100,130,100,255,60,255");  → update blue
//    Serial1.println("CAL:WHITE:50,185");    → update white thresholds
//    Serial1.println("CAL:BLACK:60,55");     → update black thresholds
//    Serial1.println("CAL:SAVE");            → persist to ESP NVS
//    Serial1.println("CAL:RESET");           → restore compiled defaults
//  ESP replies ("OK:...", "ERR:...", etc.) appear on Teensy's USB console.
//
//  E. PCB STRAP WIRING SUMMARY
//  ─────────────────────────────
//  Per ESP footprint: 2 solder-bridge gaps (GPIO1→GND, GPIO2→GND).
//  Open gap = pull-up HIGH = bit 0. Shorted gap = LOW = bit 1.
//    Forward  (0°)  : both gaps OPEN   (no solder work)
//    Right    (90°) : gap A shorted
//    Rear    (180°) : gap B shorted
//    Left    (270°) : both gaps shorted
//  Pins released to INPUT (no pull) after reading → zero current drain.
// ══════════════════════════════════════════════════════════════════════════


// ══════════════════════════════════════════════════════════════════════════
//  [v3]  TEST-MODE CONSOLE IMPLEMENTATION   (forward ESP only)
//
//  Phone (browser) --WiFi AP--> this ESP --UART(ASCII)--> Teensy (TEST state)
//  Robot B is reached via ESP-NOW relay (magic 0x70 0x54). The Teensy is the
//  authority: it executes motion/kick only while its own state == TEST.
// ══════════════════════════════════════════════════════════════════════════

static void buildAPSSID(char* out, size_t n) {
  uint8_t m[6]; WiFi.macAddress(m);
  snprintf(out, n, "%s%02X%02X", TEST_AP_PREFIX, m[4], m[5]);   // e.g. RCap_9F3A
}

void testStartAP() {
  if (g_apUp) return;
  char ssid[20]; buildAPSSID(ssid, sizeof(ssid));
  // AP on the ESP-NOW channel -> single radio channel, ESP-NOW keeps working.
  bool ok = WiFi.softAP(ssid, TEST_AP_PASSWORD, TEST_AP_CHANNEL);
  esp_wifi_set_max_tx_power(60);   // <=15 dBm conducted -> EIRP <100 mW (Rule 1.3.1.1)
  g_apUp = true;
  DBG.printf("INFO:TEST_AP_UP:%s ok=%d ip=%s ch=%d\n",
                ssid, ok ? 1 : 0, WiFi.softAPIP().toString().c_str(), TEST_AP_CHANNEL);
}

void testStopAP() {
  if (!g_apUp) return;
  WiFi.softAPdisconnect(true);     // drop AP, keep STA / ESP-NOW alive
  g_apUp = false;
  DBG.println("INFO:TEST_AP_DOWN");
}

// One ASCII line to the local Teensy over the shared UART.
void emitToTeensy(const char* line) {
#if 0  // [v5] echo OFF: runs in the async-web task; USB-CDC blocking here stalls /cmd
  DBG.print("[TX->Teensy] "); DBG.println(line);
#endif
  TEENSY.print(line);
  TEENSY.print('\n');
}

// Send a relay frame to the paired partner (robot B).
void relaySend(uint8_t type, const char* payload) {
  if (partnerID == 0x00 || !partnerPeerAdded) return;
  uint8_t buf[3 + RELAY_MAX_PAYLOAD];
  buf[0] = RELAY_MAGIC0; buf[1] = RELAY_MAGIC1; buf[2] = type;
  size_t pl = 0;
  while (pl < RELAY_MAX_PAYLOAD && payload[pl]) { buf[3 + pl] = (uint8_t)payload[pl]; pl++; }
  esp_now_send(partnerMAC, buf, (size_t)(3 + pl));
}

// Translate a /cmd request into the Teensy ASCII command. false if unknown op.
static bool buildAsciiCmd(AsyncWebServerRequest* req, char* out, size_t n) {
  if (!req->hasParam("op")) return false;
  String op = req->getParam("op")->value();
  auto P = [&](const char* k, const char* def) -> String {
    return req->hasParam(k) ? req->getParam(k)->value() : String(def);
  };
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
  else if (op == "goal_lock")         snprintf(out, n, "GOAL_LOCK:%s", P("color","yellow").c_str());
  else if (op == "ir_raw")            snprintf(out, n, "IR:RAW");
  else if (op == "color_raw")         snprintf(out, n, "COLOUR:RAW");
  else if (op == "color_dbg")         snprintf(out, n, "COLOUR:DBG");
  else if (op == "compass_read")      snprintf(out, n, "COMPASS:READ");
  else if (op == "vision_read")       snprintf(out, n, "VISION:READ");
  else if (op == "compass_cal_start") snprintf(out, n, "COMPASS:CAL_START");
  else if (op == "compass_cal_stop")  snprintf(out, n, "COMPASS:CAL_STOP");
  else if (op == "query_status")      snprintf(out, n, "QUERY:STATUS");
  else if (op == "ping_partner")      snprintf(out, n, "QUERY:STATUS");
  else if (op == "pass_test")         snprintf(out, n, "PASS_TEST");
  else if (op == "cal")               snprintf(out, n, "CALCAM:%s:%s", P("cam","1").c_str(), P("raw","STATUS").c_str());
  else return false;
  return true;
}

void testHandleCmd(AsyncWebServerRequest* req) {
  g_lastActivity = millis();
  char line[96];
  if (!buildAsciiCmd(req, line, sizeof(line))) { req->send(400, "text/plain", "bad op"); return; }
  String tgt = req->hasParam("target") ? req->getParam("target")->value() : String("A");
  if (tgt == "B") relaySend(RELAY_TYPE_CMD, line);   // -> partner robot
  else            emitToTeensy(line);                // -> local Teensy
  req->send(200, "text/plain", "ok");
}

// A telemetry line arrived from the local Teensy (ASCII, non-CAL).
// All ASCII lines are forwarded to the browser SSE stream, including COL: and COLDBG:.
void testOnTelemetry(const char* line) {
#if 0  // [v5] echo OFF: fired on every TLM (5 Hz) + ERR floods -> USB-CDC overrun/stall
  DBG.print("[RX<-Teensy] "); DBG.println(line);
#endif
  if (strncmp(line, "TLM:", 4) == 0) {
    strncpy(g_lastTlm, line, sizeof(g_lastTlm) - 1);
    g_lastTlm[sizeof(g_lastTlm) - 1] = 0;
  }
  g_sse.send(line, "tlm", millis());            // to the phone on THIS robot
  if (g_relaySessionForA) relaySend(RELAY_TYPE_RESP, line);  // B -> A mirror
}

void handleFrameBmp(AsyncWebServerRequest* req)
{
  if (!camOK)
  {
    char msg[160];
    snprintf(
      msg,
      sizeof(msg),
      "camera offline\npsramFound=%d\nfreeHeap=%u\nfreePsram=%u\ncamErr=0x%X",
      psramFound() ? 1 : 0,
      ESP.getFreeHeap(),
      ESP.getFreePsram(),
      camErr
    );

    req->send(503, "text/plain", msg);
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb)
  {
    req->send(503, "text/plain", "no frame");
    return;
  }

  const int OUT_W = 160;
  const int OUT_H = 120;
  const int SRC_W = fb->width;   // expected 320
  const int SRC_H = fb->height;  // expected 240
  const int ROW_SIZE = (OUT_W * 3 + 3) & ~3;
  const int PIX_DATA_SIZE = ROW_SIZE * OUT_H;
  const int FILE_SIZE = 54 + PIX_DATA_SIZE;

  AsyncResponseStream* response = req->beginResponseStream("image/bmp");
  response->addHeader("Cache-Control", "no-store");

  uint8_t header[54] = {0};
  header[0] = 'B';
  header[1] = 'M';

  auto put32 = [&](int pos, uint32_t v) {
    header[pos + 0] = (uint8_t)(v);
    header[pos + 1] = (uint8_t)(v >> 8);
    header[pos + 2] = (uint8_t)(v >> 16);
    header[pos + 3] = (uint8_t)(v >> 24);
  };

  auto put16 = [&](int pos, uint16_t v) {
    header[pos + 0] = (uint8_t)(v);
    header[pos + 1] = (uint8_t)(v >> 8);
  };

  put32(2, FILE_SIZE);
  put32(10, 54);
  put32(14, 40);
  put32(18, OUT_W);
  put32(22, OUT_H);
  put16(26, 1);
  put16(28, 24);
  put32(34, PIX_DATA_SIZE);

  response->write(header, sizeof(header));

  uint8_t row[ROW_SIZE];

  for (int y = OUT_H - 1; y >= 0; y--)
  {
    memset(row, 0, sizeof(row));

    int srcY = map(y, 0, OUT_H - 1, 0, SRC_H - 1);

    for (int x = 0; x < OUT_W; x++)
    {
      int srcX = map(x, 0, OUT_W - 1, 0, SRC_W - 1);
      int srcIndex = (srcY * SRC_W + srcX) * 2;

      uint8_t hi = fb->buf[srcIndex];
      uint8_t lo = fb->buf[srcIndex + 1];
      uint16_t px = ((uint16_t)hi << 8) | lo;  // RGB565

      uint8_t r = ((px >> 11) & 0x1F) << 3;
      uint8_t g = ((px >> 5)  & 0x3F) << 2;
      uint8_t b = (px & 0x1F) << 3;

      int dst = x * 3;
      row[dst + 0] = b;
      row[dst + 1] = g;
      row[dst + 2] = r;
    }

    response->write(row, ROW_SIZE);
  }

  esp_camera_fb_return(fb);
  req->send(response);
}

void testConsoleSetup() {
  if (mountAngle != 0) return;                  // forward ESP only

  // [v3] AP event logger -> lets us SEE on Serial what the phone does:
  //   AP_CLIENT_CONNECTED then ...GOT_IP  -> assoc OK, issue is the browser
  //   CONNECTED then DISCONNECTED (loops) -> assoc unstable (channel/power)
  //   nothing at all                      -> phone not associating (pwd/channel)
  WiFi.onEvent([](arduino_event_id_t e, arduino_event_info_t){
    if      (e == ARDUINO_EVENT_WIFI_AP_STACONNECTED)    DBG.println("INFO:AP_CLIENT_CONNECTED");
    else if (e == ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED)   DBG.println("INFO:AP_CLIENT_GOT_IP");
    else if (e == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) DBG.println("INFO:AP_CLIENT_DISCONNECTED");
  });

  g_http.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send_P(200, "text/html", INDEX_HTML);
  });
  g_http.on("/cmd", HTTP_GET, testHandleCmd);
  g_http.on("/frame.bmp", HTTP_GET, handleFrameBmp);
  g_http.on("/cam", HTTP_GET, [](AsyncWebServerRequest* r) {
    const char* page = R"HTML(
  <!doctype html>
  <html>
  <head>
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>RoboCap Camera</title>
    <style>
      body { font-family: Arial; background:#111; color:white; text-align:center; }
      img { width:95vw; max-width:640px; border:2px solid #555; }
      button { font-size:20px; padding:10px 18px; margin:8px; }
    </style>
  </head>
  <body>
    <h2>RoboCap Camera Preview</h2>
    <img id="cam" src="/frame.bmp">
    <br>
    <button onclick="refresh()">Refresh</button>
    <button onclick="toggle()">Auto</button>
    <p id="status">manual</p>

  <script>
  let auto = false;
  let timer = null;

  function refresh() {
    document.getElementById('cam').src = '/frame.bmp?t=' + Date.now();
  }

  function toggle() {
    auto = !auto;
    document.getElementById('status').innerText = auto ? 'auto refresh' : 'manual';
    if (auto) {
      timer = setInterval(refresh, 400);
    } else {
      clearInterval(timer);
    }
  }

  </script>
  </body>
  </html>
  )HTML";

    r->send(200, "text/html", page);
  });\
  
  g_http.on("/hsv", HTTP_GET, [](AsyncWebServerRequest* r){   // [v5] front-cam center HSV + counts
    char b[48];
    snprintf(b,sizeof(b),"%u,%u,%u,%u,%u",g_smpH,g_smpS,g_smpV,g_smpY,g_smpB);
    r->send(200,"text/plain",b);
  });
  g_sse.onConnect([](AsyncEventSourceClient* c){ c->send(g_lastTlm, "tlm", millis()); });
  g_http.addHandler(&g_sse);
  g_http.begin();
  g_lastActivity = millis();
  // SoftAP itself is raised by testConsoleLoop() per robot state (READY at boot).
}

void testConsoleLoop() {
  if (mountAngle != 0) return;                  // forward ESP only

  // 1) Gate SoftAP by state: up in READY/TEST, down in GAME (ch.14.6)
  bool wantAP = (g_robotState != ROBOT_STATE_GAME);
  if (wantAP && !g_apUp) testStartAP();
  if (!wantAP && g_apUp) testStopAP();

  // 2) Drain ESP-NOW relay queue
  while (g_relayTail != g_relayHead) {
    uint8_t type = g_relayQ[g_relayTail].type;
    char buf[RELAY_MAX_PAYLOAD + 1];
    memcpy(buf, (const void*)g_relayQ[g_relayTail].data, RELAY_MAX_PAYLOAD + 1);
    g_relayTail = (uint8_t)((g_relayTail + 1) % RELAY_Q_DEPTH);
    if (type == RELAY_TYPE_CMD) {       // we are robot B: run it + mirror TLM to A
      g_relaySessionForA = true;
      emitToTeensy(buf);
    } else {                             // RELAY_TYPE_RESP: we are A: show B's data
      g_sse.send(buf, "tlmB", millis());
    }
  }

  // 3) Idle timeout -> ask the Teensy to leave TEST (ch.14.5)
  if (g_robotState == ROBOT_STATE_TEST &&
      (millis() - g_lastActivity) > TEST_IDLE_TIMEOUT_MS) {
    emitToTeensy("TEST:OFF");
    g_lastActivity = millis();
  }
}
