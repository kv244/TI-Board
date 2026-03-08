// ============================================================================
// FILE:   lidar_display.ino
// TITLE:  2D Polar LiDAR Map — Arduino UNO R4 WiFi + 0.96" SSD1306 OLED
//
// DESCRIPTION:
//   Receives [angle, distance] packets from TMS320F28069M over UART.
//   Renders a live 2D polar map on the 0.96" SSD1306 OLED display.
//   The display shows a top-down radar-style sweep view.
//
// WIRING - OLED to Arduino UNO R4 WiFi:
//   OLED VCC → 3.3V
//   OLED GND → GND
//   OLED SDA → A4
//   OLED SCL → A5
//
// WIRING - F28069M to Arduino:
//   F28069M GPIO28 (TX) → Arduino Pin 0 (RX)
//   F28069M GND         → Arduino GND
//
// LIBRARIES REQUIRED:
//   - Adafruit SSD1306
//   - Adafruit GFX Library
//
// PACKET FORMAT (5 bytes):
//   [0xFF][angle 0-180][dist_hi][dist_lo][checksum]
//   checksum = angle XOR dist_hi XOR dist_lo
//   distance in cm (max ~4000cm = 40m, LIDAR-Lite v3 max range)
//
// DISPLAY LAYOUT:
//   The OLED is 128x64. The polar map origin sits at bottom-centre.
//   Max display radius = 30 pixels = configurable MAX_DISPLAY_CM range.
//   Each dot = one LiDAR return plotted at its polar coordinate.
//   Sweep line shows current servo angle.
//   Top-right shows live distance in cm.
// ============================================================================

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>

// --- OLED ------------------------------------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Polar Map Config -------------------------------------------------------
// Origin = bottom centre of screen
#define ORIGIN_X 64   // Centre horizontally
#define ORIGIN_Y 63   // Bottom of screen
#define MAP_RADIUS 30 // Pixels = MAX_DISPLAY_CM range
#define MAX_DISPLAY_CM                                                         \
  200 // 200cm = 2 metres full scale
      // Increase for longer range display

// --- Scan Buffer -----------------------------------------------------------
// Store last reading at each angle for persistent map
#define NUM_ANGLES 181           // 0 to 180 inclusive
uint16_t distBuffer[NUM_ANGLES]; // Distance in cm at each angle
bool hasData[NUM_ANGLES];        // Whether we have a reading yet

// --- Current state ---------------------------------------------------------
uint8_t currentAngle = 0;
uint16_t currentDistance = 0;
uint32_t packetsReceived = 0;

// --- Protocol --------------------------------------------------------------
#define SYNC_BYTE 0xFF

typedef enum {
  WAIT_SYNC,
  READ_ANGLE,
  READ_DIST_HI,
  READ_DIST_LO,
  READ_CHECKSUM
} RxState;

RxState rxState = WAIT_SYNC;
uint8_t rxAngle = 0;
uint8_t rxDistHi = 0;
uint8_t rxDistLo = 0;
bool frameReady = false;

// ============================================================================
// setup()
// ============================================================================
void setup() {
  Serial1.begin(115200); // UART from F28069M
  Serial.begin(115200);  // Debug to PC

  // Clear scan buffers
  for (int i = 0; i < NUM_ANGLES; i++) {
    distBuffer[i] = 0;
    hasData[i] = false;
  }

  // Init OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED not found!");
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  // Splash screen
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(10, 10);
  display.println("LiDAR Scanner");
  display.setCursor(10, 25);
  display.println("TMS320F28069M");
  display.setCursor(10, 40);
  display.println("Waiting for data...");
  display.display();

  Serial.println("LiDAR Polar Display Ready");
}

// ============================================================================
// loop()
// ============================================================================
void loop() {
  // --- Receive and parse packets ---
  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    switch (rxState) {
    case WAIT_SYNC:
      if (b == SYNC_BYTE)
        rxState = READ_ANGLE;
      break;

    case READ_ANGLE:
      if (b == SYNC_BYTE) {
        rxState = READ_ANGLE;
        break;
      } // re-sync
      rxAngle = b;
      rxState = READ_DIST_HI;
      break;

    case READ_DIST_HI:
      if (b == SYNC_BYTE) {
        rxState = READ_ANGLE;
        break;
      }
      rxDistHi = b;
      rxState = READ_DIST_LO;
      break;

    case READ_DIST_LO:
      if (b == SYNC_BYTE) {
        rxState = READ_ANGLE;
        break;
      }
      rxDistLo = b;
      rxState = READ_CHECKSUM;
      break;

    case READ_CHECKSUM: {
      uint8_t expected = rxAngle ^ rxDistHi ^ rxDistLo;
      if (b == expected && rxAngle <= 180) {
        // Valid packet
        currentAngle = rxAngle;
        currentDistance = ((uint16_t)rxDistHi << 8) | rxDistLo;

        // Store in buffer
        distBuffer[currentAngle] = currentDistance;
        hasData[currentAngle] = true;
        packetsReceived++;
        frameReady = true;
      }
      rxState = WAIT_SYNC;
      break;
    }
    }
  }

  // --- Render on new data ---
  if (frameReady) {
    frameReady = false;
    renderPolarMap();
  }
}

// ============================================================================
// polarToXY()
// Converts polar (angle degrees, distance cm) to OLED pixel (x, y)
// Origin at bottom-centre, 0°=left, 90°=up, 180°=right
// ============================================================================
void polarToXY(uint8_t angleDeg, uint16_t distCm, int *px, int *py) {
  // Clamp distance to display range
  float d = (float)distCm;
  if (d > MAX_DISPLAY_CM)
    d = MAX_DISPLAY_CM;

  // Scale to pixel radius
  float r = (d / (float)MAX_DISPLAY_CM) * (float)MAP_RADIUS;

  // Convert angle: 0°=left(180° in screen coords), 180°=right(0° screen)
  // Standard math: 0° is right, 90° is up
  // Our servo: 0°=left, 90°=centre-up, 180°=right
  float rad = ((float)angleDeg * M_PI) / 180.0f; // 0 to PI

  // X: origin + r*cos(angle), but mirrored so 0°=left
  *px = ORIGIN_X + (int)(r * cosf(M_PI - rad));
  // Y: origin - r*sin(angle) (screen Y inverted)
  *py = ORIGIN_Y - (int)(r * sinf(rad));
}

// ============================================================================
// renderPolarMap()
// Draws the full polar scan map on the OLED
// ============================================================================
void renderPolarMap() {
  display.clearDisplay();

  // --- Draw polar grid ---
  // Outer arc (max range)
  drawArc(ORIGIN_X, ORIGIN_Y, MAP_RADIUS);
  // Mid range arc
  drawArc(ORIGIN_X, ORIGIN_Y, MAP_RADIUS / 2);

  // Radial lines at 30° intervals
  for (int a = 0; a <= 180; a += 30) {
    int ex, ey;
    polarToXY(a, MAX_DISPLAY_CM, &ex, &ey);
    display.drawLine(ORIGIN_X, ORIGIN_Y, ex, ey, SSD1306_WHITE);
  }

  // --- Draw all stored scan points ---
  for (int a = 0; a < NUM_ANGLES; a++) {
    if (!hasData[a])
      continue;
    if (distBuffer[a] == 0 || distBuffer[a] == 0xFFFF)
      continue;
    if (distBuffer[a] > MAX_DISPLAY_CM)
      continue;

    int px, py;
    polarToXY(a, distBuffer[a], &px, &py);

    // Draw a small cross at each return point
    display.drawPixel(px, py, SSD1306_WHITE);
    display.drawPixel(px + 1, py, SSD1306_WHITE);
    display.drawPixel(px - 1, py, SSD1306_WHITE);
    display.drawPixel(px, py + 1, SSD1306_WHITE);
    display.drawPixel(px, py - 1, SSD1306_WHITE);
  }

  // --- Draw current sweep line ---
  {
    int sx, sy;
    polarToXY(currentAngle, MAX_DISPLAY_CM, &sx, &sy);
    display.drawLine(ORIGIN_X, ORIGIN_Y, sx, sy, SSD1306_WHITE);
  }

  // --- Current reading overlay (top right) ---
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Distance
  display.setCursor(72, 0);
  if (currentDistance == 0xFFFF)
    display.print("ERR");
  else {
    display.print(currentDistance);
    display.print("cm");
  }

  // Angle
  display.setCursor(72, 10);
  display.print(currentAngle);
  display.print((char)247); // Degree symbol approximation
  display.print(" ");

  // Packet counter (confirms live data)
  display.setCursor(72, 20);
  display.print("P:");
  display.print(packetsReceived % 10000);

  // Scale label
  display.setCursor(0, 0);
  display.print("2m");

  display.display();

  // Debug to PC
  Serial.print("A=");
  Serial.print(currentAngle);
  Serial.print(" D=");
  Serial.print(currentDistance);
  Serial.println("cm");
}

// ============================================================================
// drawArc()
// Draws a semicircular arc (180°) centred at (cx, cy) with given radius
// ============================================================================
void drawArc(int cx, int cy, int r) {
  int prevX = -1, prevY = -1;
  for (int a = 0; a <= 180; a += 3) {
    float rad = (float)a * M_PI / 180.0f;
    int x = cx + (int)(r * cosf(M_PI - rad));
    int y = cy - (int)(r * sinf(rad));

    if (prevX >= 0)
      display.drawLine(prevX, prevY, x, y, SSD1306_WHITE);

    prevX = x;
    prevY = y;
  }
}
