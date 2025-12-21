#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPUpdate.h>  // For HTTP-based OTA updates
#include "FS.h"
#include "SD_MMC.h"
#include "time.h"
#include "config.h"  // ⚠️ Create from config.h.example with your credentials

// OTA update state
bool otaPending = false;
String otaUrl = "";
String otaVersion = "";

#define DETECT_LED 12
#define BUZZER_PIN 13  // Buzzer on GPIO 13 - HIGH = beep, LOW = silent 

// NTP settings for timestamp (from config.h)
const char* ntpServer = NTP_SERVER;
const long gmtOffset_sec = GMT_OFFSET_SEC;
const int daylightOffset_sec = DAYLIGHT_OFFSET;
bool timeConfigured = false;

// WiFi networks list (from config.h)
struct WiFiNetwork {
  const char* ssid;
  const char* password;
};

// Default WiFi from config.h
const char* DEFAULT_WIFI_SSID = WIFI_SSID;
const char* DEFAULT_WIFI_PASS = WIFI_PASSWORD;

const unsigned long DEFAULT_WIFI_TIMEOUT = WIFI_TIMEOUT_MS;

// Fallback WiFi networks from config.h
const WiFiNetwork wifiNetworks[] = {
  {WIFI_FALLBACK_1_SSID, WIFI_FALLBACK_1_PASSWORD},
  {WIFI_FALLBACK_2_SSID, WIFI_FALLBACK_2_PASSWORD},
};
const int wifiNetworkCount = sizeof(wifiNetworks) / sizeof(wifiNetworks[0]);
int currentWifiIndex = 0;

// MQTT servers list from config.h
struct MQTTServer {
  const char* host;
  int port;
};

const MQTTServer mqttServers[] = {
  {MQTT_SERVER_BACKUP, MQTT_PORT_BACKUP},  // Backup (empty = skip)
  {MQTT_SERVER, MQTT_PORT},                 // Primary server
};
const int mqttServerCount = sizeof(mqttServers) / sizeof(mqttServers[0]);
int currentMqttIndex = 0;
const char* mqtt_topic = MQTT_TOPIC_DETECTION;

// fixed settings
const int LEFT_ZONE = 140;   // wider left tolerance
const int RIGHT_ZONE = 180;  // wider right tolerance
const int MOTION_THRESHOLD = 100;
const unsigned long COOLDOWN_MS = 2000;  // 2s between counts

// WiFi connection settings (non-blocking)
bool wifiConnected = false;
unsigned long lastWifiAttempt = 0;
const unsigned long WIFI_RETRY_INTERVAL = 60000;  // 1 minute
const unsigned long WIFI_CONNECT_TIMEOUT = 4000;  // 4s per network (non-blocking)
unsigned long wifiConnectStart = 0;
bool wifiConnecting = false;

// SD card settings
bool sdCardAvailable = false;
unsigned long imageCounter = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
int passengerCount = 0;
int state = 0;  // 0=none, 1=left, 2=right
unsigned long lastCountTime = 0;
uint8_t roiPrev[19200] = {0};


// AI-Thinker pins
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Helper function to draw a horizontal line on the frame buffer
void drawHLine(uint8_t* buf, int width, int x1, int x2, int y, uint8_t color) {
  if (y < 0 || y >= 240) return;
  for (int x = max(0, x1); x <= min(width-1, x2); x++) {
    buf[y * width + x] = color;
  }
}

// Helper function to draw a vertical line on the frame buffer
void drawVLine(uint8_t* buf, int width, int x, int y1, int y2, uint8_t color) {
  if (x < 0 || x >= width) return;
  for (int y = max(0, y1); y <= min(239, y2); y++) {
    buf[y * width + x] = color;
  }
}

// Draw rectangle on frame buffer
void drawRect(uint8_t* buf, int width, int x1, int y1, int x2, int y2, uint8_t color) {
  drawHLine(buf, width, x1, x2, y1, color);  // Top
  drawHLine(buf, width, x1, x2, y2, color);  // Bottom
  drawVLine(buf, width, x1, y1, y2, color);  // Left
  drawVLine(buf, width, x2, y1, y2, color);  // Right
}

// Draw arrow pattern for direction indication
// ENTER (L→R): draws >>> pattern
// EXIT (R→L): draws <<< pattern
void drawDirectionArrow(uint8_t* buf, int width, bool isEnter) {
  int startX = isEnter ? 10 : 280;  // Position based on direction
  int y = 15;
  uint8_t color = 255;  // White
  
  for (int i = 0; i < 3; i++) {  // 3 arrows
    int baseX = isEnter ? (startX + i * 20) : (startX - i * 20);
    if (isEnter) {
      // Draw > arrow
      for (int j = 0; j < 8; j++) {
        int px = baseX + j;
        int py1 = y - j;
        int py2 = y + j;
        if (px >= 0 && px < width) {
          if (py1 >= 0 && py1 < 240) buf[py1 * width + px] = color;
          if (py2 >= 0 && py2 < 240) buf[py2 * width + px] = color;
        }
      }
    } else {
      // Draw < arrow
      for (int j = 0; j < 8; j++) {
        int px = baseX - j;
        int py1 = y - j;
        int py2 = y + j;
        if (px >= 0 && px < width) {
          if (py1 >= 0 && py1 < 240) buf[py1 * width + px] = color;
          if (py2 >= 0 && py2 < 240) buf[py2 * width + px] = color;
        }
      }
    }
  }
}

// Flip image 180 degrees (upside down correction)
void flipImage(uint8_t* buf, int width, int height) {
  for (int y = 0; y < height / 2; y++) {
    for (int x = 0; x < width; x++) {
      int topIdx = y * width + x;
      int botIdx = (height - 1 - y) * width + (width - 1 - x);
      uint8_t temp = buf[topIdx];
      buf[topIdx] = buf[botIdx];
      buf[botIdx] = temp;
    }
  }
}

// LED control functions
void ledOn() {
  digitalWrite(DETECT_LED, HIGH);
}

void ledOff() {
  digitalWrite(DETECT_LED, LOW);
}

// Rapid blink LED for 3 seconds (WiFi success indication)
void rapidBlinkLED(int durationMs) {
  unsigned long endTime = millis() + durationMs;
  while (millis() < endTime) {
    digitalWrite(DETECT_LED, HIGH);
    delay(50);
    digitalWrite(DETECT_LED, LOW);
    delay(50);
}
}

// Ring bell function - beep pattern for passenger notification
// ACTIVE HIGH: HIGH = beep, LOW = silent
void ringBell() {
  Serial.println("🔔 RING BELL TRIGGERED!");
  
  // 0.5 second long beep
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  
  delay(150);  // Short pause
  
  // 0.25 second short beep
  digitalWrite(BUZZER_PIN, HIGH);
  delay(250);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println("🔔 Ring complete");
}

// MQTT callback for receiving commands
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.printf("📨 MQTT [%s]: %s\n", topic, message.c_str());
  
  // Check for ring command
  if (String(topic).indexOf("ring") >= 0) {
    ringBell();
  }
  
  // Check for OTA update command
  #ifdef OTA_ENABLED
  if (String(topic).indexOf("ota") >= 0) {
    // Parse JSON to get URL and version
    // Simple parsing without ArduinoJson to save memory
    int urlStart = message.indexOf("\"url\":\"");
    int versionStart = message.indexOf("\"version\":\"");
    
    if (urlStart >= 0 && versionStart >= 0) {
      urlStart += 7;  // Skip "url":"
      int urlEnd = message.indexOf("\"", urlStart);
      
      versionStart += 11;  // Skip "version":"
      int versionEnd = message.indexOf("\"", versionStart);
      
      if (urlEnd > urlStart && versionEnd > versionStart) {
        otaUrl = message.substring(urlStart, urlEnd);
        otaVersion = message.substring(versionStart, versionEnd);
        
        Serial.printf("📥 OTA Update requested: v%s\n", otaVersion.c_str());
        Serial.printf("📥 Firmware URL: %s\n", otaUrl.c_str());
        
        // Check if we need to update (skip if same version unless forced)
        bool forceUpdate = message.indexOf("\"force\":true") >= 0;
        if (otaVersion == FIRMWARE_VERSION && !forceUpdate) {
          Serial.println("ℹ️ Already on this version, skipping update");
          return;
        }
        
        // Set flag to perform OTA in main loop (not in callback!)
        otaPending = true;
      }
    }
  }
  #endif
}

// Function to save image to SD card - accepts frame buffer and bounding box
// Saves as BMP format with rectangle overlay and direction arrow
void saveImageToSD(const char* direction, camera_fb_t* fb, int blobMinX, int blobMaxX, int blobMinY, int blobMaxY) {
  if (!sdCardAvailable) {
    Serial.println("⚠️ SD card not available, skipping save");
    return;
  }
  
  if (!fb || !fb->buf) {
    Serial.println("❌ Invalid frame buffer for SD save");
    return;
  }
  
  // Flip image 180 degrees first (correct upside-down camera)
  flipImage(fb->buf, 320, 240);
  
  // Flip bounding box coordinates too
  int newMinX = 320 - 1 - blobMaxX;
  int newMaxX = 320 - 1 - blobMinX;
  int newMinY = 240 - 1 - blobMaxY;
  int newMaxY = 240 - 1 - blobMinY;
  
  // Draw rectangle around detected blob (white = 255)
  drawRect(fb->buf, 320, newMinX, newMinY, newMaxX, newMaxY, 255);
  // Draw thicker rectangle (2px)
  drawRect(fb->buf, 320, newMinX-1, newMinY-1, newMaxX+1, newMaxY+1, 255);
  
  // Draw direction arrow at top of image (flip direction since image is flipped)
  bool isEnter = (strcmp(direction, "enter") == 0);
  drawDirectionArrow(fb->buf, 320, isEnter);
  
  // Draw the zone lines (LEFT_ZONE and RIGHT_ZONE) as dashed vertical lines
  int flippedLeftZone = 320 - 1 - RIGHT_ZONE;
  int flippedRightZone = 320 - 1 - LEFT_ZONE;
  for (int y = 0; y < 240; y += 4) {
    if (y % 8 < 4) {  // Dashed pattern
      fb->buf[y * 320 + flippedLeftZone] = 180;
      fb->buf[y * 320 + flippedRightZone] = 180;
    }
  }
  
  // Create filename with timestamp or counter
  char filename[64];
  if (timeConfigured) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char timeStr[32];
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H-%M-%S", &timeinfo);
      snprintf(filename, sizeof(filename), "/person_detect/%s_%s_%lu.bmp", 
               timeStr, direction, imageCounter++);
    } else {
      snprintf(filename, sizeof(filename), "/person_detect/%s_%lu_%lu.bmp", 
               direction, millis() / 1000, imageCounter++);
    }
  } else {
    snprintf(filename, sizeof(filename), "/person_detect/%s_%lu_%lu.bmp", 
             direction, millis() / 1000, imageCounter++);
  }
  
  // BMP file parameters for 320x240 8-bit grayscale
  const int width = 320;
  const int height = 240;
  const int colorTableSize = 256 * 4;  // 256 colors * 4 bytes each
  const int headerSize = 54;  // BMP header + DIB header
  const int imageSize = width * height;
  const int fileSize = headerSize + colorTableSize + imageSize;
  
  File file = SD_MMC.open(filename, FILE_WRITE);
  if (!file) {
    Serial.printf("❌ Failed to open file: %s\n", filename);
    return;
  }
  
  // BMP Header (14 bytes)
  uint8_t bmpHeader[14] = {
    'B', 'M',                           // Signature
    fileSize & 0xFF, (fileSize >> 8) & 0xFF, (fileSize >> 16) & 0xFF, (fileSize >> 24) & 0xFF,  // File size
    0, 0, 0, 0,                         // Reserved
    (headerSize + colorTableSize) & 0xFF, ((headerSize + colorTableSize) >> 8) & 0xFF, 0, 0  // Pixel data offset
  };
  file.write(bmpHeader, 14);
  
  // DIB Header (40 bytes) - BITMAPINFOHEADER
  uint8_t dibHeader[40] = {
    40, 0, 0, 0,                        // DIB header size
    width & 0xFF, (width >> 8) & 0xFF, 0, 0,    // Width
    (uint8_t)((-height) & 0xFF), (uint8_t)(((-height) >> 8) & 0xFF), 0xFF, 0xFF,  // Height (negative = top-down)
    1, 0,                               // Color planes
    8, 0,                               // Bits per pixel (8-bit grayscale)
    0, 0, 0, 0,                         // Compression (none)
    imageSize & 0xFF, (imageSize >> 8) & 0xFF, (imageSize >> 16) & 0xFF, 0,  // Image size
    0x13, 0x0B, 0, 0,                   // Horizontal resolution (2835 ppm)
    0x13, 0x0B, 0, 0,                   // Vertical resolution (2835 ppm)
    0, 1, 0, 0,                         // Colors in palette (256)
    0, 0, 0, 0                          // Important colors (all)
  };
  file.write(dibHeader, 40);
  
  // Color table (256 grayscale entries, 4 bytes each: B, G, R, 0)
  uint8_t colorEntry[4];
  for (int i = 0; i < 256; i++) {
    colorEntry[0] = i;  // Blue
    colorEntry[1] = i;  // Green
    colorEntry[2] = i;  // Red
    colorEntry[3] = 0;  // Reserved
    file.write(colorEntry, 4);
  }
  
  // Pixel data (write directly - already 320x240 grayscale with overlays)
  file.write(fb->buf, fb->len);
  file.close();
  
  Serial.printf("📸 Saved: %s (%d bytes)\n", filename, fileSize);
}


void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("🚌 STABLE Bus Counter v1.2 (with SD Card + Timestamp)");
  
  // Initialize LED for detection indicator
  pinMode(DETECT_LED, OUTPUT);
  digitalWrite(DETECT_LED, LOW);  // Start with LED off
  
  // Initialize Buzzer pin - ACTIVE HIGH (HIGH = beep, LOW = silent)
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Start with buzzer OFF

  // IMPORTANT: Initialize camera FIRST before SD card to avoid I2C conflicts
  // camera config
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 16000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // Use grayscale for detection
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("❌ Camera");
    while(1);
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_contrast(s, 2);
  Serial.println("✅ Camera");

  // Initialize SD card AFTER camera to avoid pin conflicts
  Serial.println("📂 Initializing SD card...");
  if (!SD_MMC.begin("/sdcard", true)) {  // 1-bit mode for compatibility
    Serial.println("⚠️ SD Card mount failed - continuing without SD");
    sdCardAvailable = false;
  } else {
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("⚠️ No SD card inserted");
      sdCardAvailable = false;
    } else {
      sdCardAvailable = true;
      Serial.printf("✅ SD Card: %lluMB\n", SD_MMC.cardSize() / (1024 * 1024));
      
      // Create directory for person detection images
      if (!SD_MMC.exists("/person_detect")) {
        SD_MMC.mkdir("/person_detect");
        Serial.println("📁 Created /person_detect folder");
      }
    }
  }

  // Try default WiFi first (4 second timeout)
  Serial.printf("📡 Trying default WiFi: %s\n", DEFAULT_WIFI_SSID);
  WiFi.begin(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASS);
  
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < DEFAULT_WIFI_TIMEOUT) {
    delay(100);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\n✅ Default WiFi Connected: " + WiFi.localIP().toString());
    
    // Keep LED on for 3 seconds to indicate success
    ledOn();
    delay(3000);
    ledOff();
    
    // Configure NTP time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("🕐 NTP time configured");
    timeConfigured = true;
  } else {
    Serial.println("\n⚠️ Default WiFi failed - will try fallback networks");
    Serial.println("📱 Detection will work offline, retrying WiFi every 1 minute");
    wifiConnecting = false;
    lastWifiAttempt = millis();
  }

  Serial.println("🛡️ STABLE MODE - 2s cooldown active");
  if (sdCardAvailable) {
    Serial.println("📸 SD Card saving ENABLED");
  }
}

// Try to connect to next WiFi network in the list
void tryConnectWiFi() {
  if (wifiConnecting) return;
  
  WiFi.disconnect(true);
  delay(100);
  
  Serial.printf("📶 Trying WiFi %d/%d: %s\n", 
                currentWifiIndex + 1, wifiNetworkCount, 
                wifiNetworks[currentWifiIndex].ssid);
  
  WiFi.begin(wifiNetworks[currentWifiIndex].ssid, 
             wifiNetworks[currentWifiIndex].password);
  
  wifiConnecting = true;
  wifiConnectStart = millis();
  lastWifiAttempt = millis();
}

// Handle non-blocking WiFi connection (call in loop)
void handleWiFiConnection() {
  // Check if connected
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiConnected) {
      wifiConnected = true;
      wifiConnecting = false;
      Serial.println("\n✅ WiFi Connected: " + WiFi.localIP().toString());
      
      // Configure NTP time
      if (!timeConfigured) {
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        Serial.println("🕐 NTP time configured");
        timeConfigured = true;
      }
    }
    return;
  }
  
  // WiFi disconnected
  if (wifiConnected) {
    wifiConnected = false;
    Serial.println("⚠️ WiFi Disconnected");
  }
  
  // If currently trying to connect, check timeout
  if (wifiConnecting) {
    if (millis() - wifiConnectStart > WIFI_CONNECT_TIMEOUT) {
      Serial.printf("⏱️ Timeout on %s\n", wifiNetworks[currentWifiIndex].ssid);
      wifiConnecting = false;
      // Move to next network
      currentWifiIndex = (currentWifiIndex + 1) % wifiNetworkCount;
      // Try next network immediately
      tryConnectWiFi();
    }
    return;
  }
  
  // If not connecting and interval passed, try again
  if (millis() - lastWifiAttempt > WIFI_RETRY_INTERVAL) {
    tryConnectWiFi();
  }
}

void loop() {
  // Handle WiFi connection (non-blocking)
  handleWiFiConnection();
  
  // Only handle MQTT if WiFi is connected
  if (wifiConnected) {
    if (!mqttClient.connected()) reconnectMQTT();
    mqttClient.loop();
    
    // Check if OTA update is pending
    #ifdef OTA_ENABLED
    if (otaPending) {
      performOTA();
    }
    #endif
  }

  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return;

  // ROI scan (door area) - track bounding box of dark pixels
  int motion = 0, dark = 0, blobX = 0;
  int blobMinX = 320, blobMaxX = 0, blobMinY = 240, blobMaxY = 0;
  int roiIdx = 0;
  
  for (int y = 80; y < 140; y++) {
    for (int x = 0; x < 320; x++) {
      int idx = y * 320 + x;
      uint8_t p = fb->buf[idx];
      
      int diff = abs((int)p - (int)roiPrev[roiIdx]);
      if (diff > 25) motion++;
      
      if (p < 90) {
        dark++;
        blobX += x;
        // Track bounding box
        if (x < blobMinX) blobMinX = x;
        if (x > blobMaxX) blobMaxX = x;
        if (y < blobMinY) blobMinY = y;
        if (y > blobMaxY) blobMaxY = y;
      }
      roiPrev[roiIdx++] = p;
    }
  }
  
  blobX /= max(1, dark);
  
  // Ensure valid bounding box (add padding)
  if (blobMinX > blobMaxX) { blobMinX = blobX - 30; blobMaxX = blobX + 30; }
  if (blobMinY > blobMaxY) { blobMinY = 80; blobMaxY = 140; }
  blobMinX = max(0, blobMinX - 5);
  blobMaxX = min(319, blobMaxX + 5);
  blobMinY = max(0, blobMinY - 5);
  blobMaxY = min(239, blobMaxY + 5);

  // Debug output every 2 seconds to monitor detection
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 2000) {
    Serial.printf("📊 motion:%d dark:%d blobX:%d state:%d\n", motion, dark, blobX, state);
    lastDebugTime = millis();
  }

  // stable logic with cooldown
  if (motion > MOTION_THRESHOLD && dark > 300 && 
      millis() - lastCountTime > COOLDOWN_MS) {
    
    int newState = (blobX < LEFT_ZONE) ? 1 : 
                   (blobX > RIGHT_ZONE) ? 2 : state;
    
    // only count on state change   
    if (newState != state) {
      if (state == 1 && newState == 2) {  // LEFT to RIGHT = ENTER
        passengerCount++;
        sendMQTT("enter");
        Serial.printf("✅ ENTER  C:%d (x:%d→%d)\n", passengerCount, LEFT_ZONE, RIGHT_ZONE);
        ledOn();  // Turn LED ON to indicate detection
        saveImageToSD("enter", fb, blobMinX, blobMaxX, blobMinY, blobMaxY);
        delay(200);  // Keep LED on for at least 200ms to be visible
        ledOff();  // Turn off LED after detection complete
      } else if (state == 2 && newState == 1) {  // RIGHT to LEFT = EXIT
        if (passengerCount > 0) passengerCount--;
        sendMQTT("exit");
        Serial.printf("✅ EXIT   C:%d (x:%d→%d)\n", passengerCount, RIGHT_ZONE, LEFT_ZONE);
        ledOn();  // Turn LED ON to indicate detection
        saveImageToSD("exit", fb, blobMinX, blobMaxX, blobMinY, blobMaxY);
        delay(200);  // Keep LED on for at least 200ms to be visible
        ledOff();  // Turn off LED after detection complete
      }
      state = newState;
      lastCountTime = millis();
    }
  }
  
  esp_camera_fb_return(fb);

  // --- MERGED: RSSI STATUS HEARTBEAT ---
  // Send status every 5 seconds if connected
  if (wifiConnected) {
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000) {
      publishStatus();
      lastStatusTime = millis();
    }
  }

  delay(30);
}

void sendMQTT(String dir) {
  if (!wifiConnected) return;  // Skip MQTT if no WiFi
  char buf[80];
  snprintf(buf, 80, "{\"dir\":\"%s\",\"count\":%d,\"t\":%ld}", 
           dir.c_str(), passengerCount, millis()/1000);
  mqttClient.publish(mqtt_topic, buf);
}

// --- MERGED: PUBLISH STATUS FUNCTION ---
void publishStatus() {
  if (!mqttClient.connected()) return;
  
  long rssi = WiFi.RSSI();
  char buf[100];
  // Using fixed ID "ESP32-CAM-01" to match App expectation (App listens to sut/bus/+/status)
  snprintf(buf, 100, "{\"rssi\":%ld, \"uptime\":%lu}", rssi, millis()/1000);
  
  // Topic: sut/bus/ESP32-CAM-01/status (from config.h)
  mqttClient.publish(MQTT_TOPIC_STATUS, buf);
}

void reconnectMQTT() {
  static unsigned long lastMqttAttempt = 0;
  if (millis() - lastMqttAttempt < 10000) return;
  lastMqttAttempt = millis();
  
  // Skip empty hosts
  while (strlen(mqttServers[currentMqttIndex].host) == 0) {
    currentMqttIndex = (currentMqttIndex + 1) % mqttServerCount;
  }
  
  Serial.printf("🔌 MQTT trying %d/%d: %s:%d\n", 
                currentMqttIndex + 1, mqttServerCount,
                mqttServers[currentMqttIndex].host, 
                mqttServers[currentMqttIndex].port);
  
  mqttClient.setServer(mqttServers[currentMqttIndex].host, 
                       mqttServers[currentMqttIndex].port);
  mqttClient.setCallback(mqttCallback);  // Set callback for incoming messages
  
  if (mqttClient.connect("BusCamStable")) {
    Serial.println("✅ MQTT Connected");
    // Subscribe to ring command topic
    mqttClient.subscribe(MQTT_TOPIC_RING);
    mqttClient.subscribe("sut/bus/+/ring");  // Also listen for bus-specific ring
    Serial.println("🔔 Subscribed to ring topics");
    
    // Subscribe to OTA topic
    #ifdef OTA_ENABLED
    mqttClient.subscribe(MQTT_TOPIC_OTA);
    Serial.printf("📥 Subscribed to OTA topic: %s\n", MQTT_TOPIC_OTA);
    Serial.printf("📌 Current firmware version: %s\n", FIRMWARE_VERSION);
    #endif
  } else {
    Serial.println("❌ MQTT Failed, trying next server");
    currentMqttIndex = (currentMqttIndex + 1) % mqttServerCount;
  }
}

// =============================================================================
// OTA (Over-The-Air) Update Functions
// =============================================================================

#ifdef OTA_ENABLED
void performOTA() {
  otaPending = false;  // Reset flag
  
  if (otaUrl.length() == 0) {
    Serial.println("❌ OTA URL is empty");
    return;
  }
  
  Serial.println("🔄 Starting OTA update...");
  Serial.printf("📥 Downloading: %s\n", otaUrl.c_str());
  
  // Blink LED rapidly to indicate OTA in progress
  for (int i = 0; i < 5; i++) {
    digitalWrite(DETECT_LED, HIGH);
    delay(100);
    digitalWrite(DETECT_LED, LOW);
    delay(100);
  }
  
  // Publish OTA status to MQTT
  char statusBuf[150];
  snprintf(statusBuf, sizeof(statusBuf), 
           "{\"status\":\"downloading\",\"version\":\"%s\",\"current\":\"%s\"}", 
           otaVersion.c_str(), FIRMWARE_VERSION);
  mqttClient.publish(MQTT_TOPIC_STATUS, statusBuf);
  
  // Keep LED on during download
  digitalWrite(DETECT_LED, HIGH);
  
  // Perform HTTP OTA update
  WiFiClient updateClient;
  httpUpdate.setLedPin(DETECT_LED, LOW);  // LED on during update
  httpUpdate.rebootOnUpdate(false);  // Don't auto-reboot, we'll do it manually
  
  t_httpUpdate_return ret = httpUpdate.update(updateClient, otaUrl);
  
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("❌ OTA Failed! Error (%d): %s\n", 
                    httpUpdate.getLastError(), 
                    httpUpdate.getLastErrorString().c_str());
      
      // Publish failure status
      snprintf(statusBuf, sizeof(statusBuf), 
               "{\"status\":\"failed\",\"error\":\"%s\",\"code\":%d}", 
               httpUpdate.getLastErrorString().c_str(),
               httpUpdate.getLastError());
      mqttClient.publish(MQTT_TOPIC_STATUS, statusBuf);
      
      // Blink LED to indicate failure
      for (int i = 0; i < 10; i++) {
        digitalWrite(DETECT_LED, HIGH);
        delay(50);
        digitalWrite(DETECT_LED, LOW);
        delay(50);
      }
      break;
      
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("ℹ️ No updates available");
      snprintf(statusBuf, sizeof(statusBuf), 
               "{\"status\":\"no_update\",\"version\":\"%s\"}", 
               FIRMWARE_VERSION);
      mqttClient.publish(MQTT_TOPIC_STATUS, statusBuf);
      digitalWrite(DETECT_LED, LOW);
      break;
      
    case HTTP_UPDATE_OK:
      Serial.println("✅ OTA Update successful! Rebooting...");
      
      // Publish success status
      snprintf(statusBuf, sizeof(statusBuf), 
               "{\"status\":\"success\",\"version\":\"%s\",\"rebooting\":true}", 
               otaVersion.c_str());
      mqttClient.publish(MQTT_TOPIC_STATUS, statusBuf);
      mqttClient.loop();  // Ensure message is sent
      
      // Keep LED on for 2 seconds before reboot
      digitalWrite(DETECT_LED, HIGH);
      delay(2000);
      
      // Reboot to apply new firmware
      ESP.restart();
      break;
  }
  
  // Clear OTA variables
  otaUrl = "";
  otaVersion = "";
}
#endif
