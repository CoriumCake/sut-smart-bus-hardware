#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPUpdate.h>  // For HTTP-based OTA updates
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <RTClib.h>
#include "DHT.h"
#include "config.h"  // Include configuration file

// OTA update state
bool otaPending = false;
String otaUrl = "";
String otaVersion = "";

// ================= CONFIGURATION =================
// All settings are now in config.h
// Copy config.h.example to config.h and edit values
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

const char* mqtt_server = MQTT_SERVER;
const int   mqtt_port   = MQTT_PORT;
const char* mqtt_topic  = MQTT_TOPIC;
const char* mqtt_topic_fast = MQTT_TOPIC_FAST;
const char* bus_name    = BUS_NAME;
const char* web_name    = WEB_NAME;

const char* filename = "/gps.csv";

// ================= DHT =================
DHT dht(DHTPIN, DHTTYPE);
 
// ================= OBJECTS =================
WiFiClient espClient;
PubSubClient client(espClient);
StaticJsonDocument<512> doc;
WebServer server(80);
RTC_DS3231 rtc;
 
char bus_mac[18];
 
// ================= GPS =================
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
 
bool gpsValid = false;
double lastLat = 0;
double lastLon = 0;
 
// ================= PMS =================
HardwareSerial pmsSerial(2);
 
struct PMS_Data {
  uint16_t pm2_5_std;
  uint16_t pm10_0_std;
};
PMS_Data pmsData;
bool pmsDataValid = false;
 
// ================= DHT DATA =================
float tempC = 0;
float humid = 0;
 
// ================= TIMER CONFIG =================
unsigned long last5s = 0;
 
unsigned long last30s = 0;
 
unsigned long lastGpsPublish = 0;
 
// ================= FUNCTION DECLARE =================
void setup_wifi();
void maintainMqtt();
void processGPS();
void processPMS();
void processDHT();
bool readPMSFrame();
void publishData();
void publishGPS();
void saveToSD();
void handleRoot();
void handleDownload();
void handleDelete();
 
// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);
 
  setup_wifi();
  MDNS.begin(web_name);
  WiFi.macAddress().toCharArray(bus_mac, sizeof(bus_mac));
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);  // Set callback for OTA commands
 
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  dht.begin();
  Wire.begin(21, 22);
  rtc.begin();
 
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card Mount Failed");
  } else {
    if (!SD.exists(filename)) {
      File f = SD.open(filename, FILE_WRITE);
      f.println("Date,Time,Lat,Lon,PM2.5,PM10,Temp,Hum,Source");
      f.close();
    }
  }
 
  server.on("/", handleRoot);
  server.on("/download", handleDownload);
  server.on("/delete", handleDelete);
  server.begin();
 
  Serial.println("System Ready - [5s: MQTT/Serial] [30s: SD Card]");
}
 
// ================= LOOP =================
void loop() {
  processGPS(); 
  processPMS(); 
  server.handleClient();
  maintainMqtt();
  client.loop();
  
  // Check if OTA update is pending
  #ifdef OTA_ENABLED
  if (otaPending && WiFi.status() == WL_CONNECTED) {
    performOTA();
  }
  #endif
 
  // 1. งานทุก 5 วินาที: อัปเดต Serial และส่งข้อมูลไป Server (MQTT)
  if (millis() - last5s >= INTERVAL_5S) {
    last5s = millis();
 
    processDHT(); // อ่านค่า Temp/Humid ล่าสุด
 
    Serial.println("\n--- [ 5 SECONDS UPDATE ] ---");
    if (pmsDataValid) {
      Serial.printf("PMS: PM2.5=%d, PM10=%d | ", pmsData.pm2_5_std, pmsData.pm10_0_std);
    }
    Serial.printf("DHT: T=%.1f, H=%.0f\n", tempC, humid);
 
    publishData(); // ส่งไป MQTT
  }
 
  // 2. งานทุก 30 วินาที: บันทึกลง SD Card
  if (millis() - last30s >= INTERVAL_30S) {
    last30s = millis();
 
    Serial.println(">>> [ 30 SECONDS UPDATE: SAVING TO SD CARD ] <<<");
    saveToSD();
  }
 
  // (Optional) Fast GPS publish ทุก 500ms ถ้ายังจำเป็นต้องใช้
  if (millis() - lastGpsPublish >= GPS_INTERVAL) {
    lastGpsPublish = millis();
    publishGPS();
  }
}
 
// ================= FUNCTIONS =================
 
void processGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid()) {
    gpsValid = true;
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
  }
}
 
void processPMS() {
  static unsigned long lastPMSDebug = 0;
  if (millis() - lastPMSDebug >= 5000) {
    lastPMSDebug = millis();
    Serial.printf("[PMS DEBUG] Available bytes: %d, Valid: %s\n", 
                  pmsSerial.available(), 
                  pmsDataValid ? "YES" : "NO");
  }
  if (readPMSFrame()) {
    pmsDataValid = true;
  }
}
 
bool readPMSFrame() {
  if (pmsSerial.available() < 32) return false;
  if (pmsSerial.peek() != 0x42) {
    pmsSerial.read();
    return false;
  }
  uint8_t buf[32];
  pmsSerial.readBytes(buf, 32);
  if (buf[1] != 0x4D) return false;
 
  // แก้ไขจุดที่อาจเป็น Error (Bit shift)
  pmsData.pm2_5_std = (buf[6] << 8) | buf[7];
  pmsData.pm10_0_std = (buf[8] << 8) | buf[9];
  Serial.printf("[PMS] Read OK: PM2.5=%d, PM10=%d\n", pmsData.pm2_5_std, pmsData.pm10_0_std);
  return true;
}
 
void processDHT() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  Serial.printf("[DHT DEBUG] Raw: T=%.1f, H=%.1f, Valid: %s\n", 
                t, h, 
                (!isnan(t) && !isnan(h)) ? "YES" : "NO");
  if (!isnan(t) && !isnan(h)) {
    tempC = t;
    humid = h;
  }
}
 
void publishData() {
  if (!client.connected()) return;
  doc.clear();
  doc["bus_mac"] = bus_mac;
  doc["bus_name"] = bus_name;
  if (gpsValid) {
    doc["lat"] = lastLat;
    doc["lon"] = lastLon;
  }
  if (pmsDataValid) {
    doc["pm2_5"] = pmsData.pm2_5_std;
    doc["pm10"]  = pmsData.pm10_0_std;
  }
  doc["temp"] = tempC;
  doc["hum"]  = humid;
  char payload[256];
  serializeJson(doc, payload);
  client.publish(mqtt_topic, payload);
  Serial.print("MQTT Sent: "); Serial.println(payload);
}
 
void publishGPS() {
  if (!client.connected() || !gpsValid) return;
  StaticJsonDocument<128> gpsDoc;
  gpsDoc["bus_mac"] = bus_mac;
  gpsDoc["lat"] = lastLat;
  gpsDoc["lon"] = lastLon;
  char payload[128];
  serializeJson(gpsDoc, payload);
  client.publish(mqtt_topic_fast, payload);
}
 
void saveToSD() {
  File file = SD.open(filename, FILE_APPEND);
  if (!file) {
    Serial.println("SD Error: Open Fail");
    return;
  }
  DateTime now = rtc.now();
  char dateStr[11], timeStr[9];
  sprintf(dateStr, "%04d/%02d/%02d", now.year(), now.month(), now.day());
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  file.print(dateStr); file.print(",");
  file.print(timeStr); file.print(",");
  file.print(gpsValid ? String(lastLat, 6) : ""); file.print(",");
  file.print(gpsValid ? String(lastLon, 6) : ""); file.print(",");
  file.print(pmsDataValid ? String(pmsData.pm2_5_std) : ""); file.print(",");
  file.print(pmsDataValid ? String(pmsData.pm10_0_std) : ""); file.print(",");
  file.print(tempC, 1); file.print(",");
  file.print(humid, 0); file.println(",RTC");
  file.close();
  Serial.println("SD Card: Data Logged Success");
}
 
void setup_wifi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}
 
void maintainMqtt() {
  if (!client.connected()) {
    Serial.printf("[MQTT] Attempting connection to %s:%d...\n", mqtt_server, mqtt_port);
    Serial.printf("[MQTT] WiFi Status: %s, IP: %s\n", 
                  WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                  WiFi.localIP().toString().c_str());
 
    if (client.connect(bus_mac)) {
      Serial.println("[MQTT] Connected successfully!");
      
      // Subscribe to OTA topic
      #ifdef OTA_ENABLED
      client.subscribe(MQTT_TOPIC_OTA);
      Serial.printf("📥 Subscribed to OTA topic: %s\n", MQTT_TOPIC_OTA);
      Serial.printf("📌 Current firmware version: %s\n", FIRMWARE_VERSION);
      #endif
    } else {
      int state = client.state();
      Serial.printf("[MQTT] Connection FAILED! Error code: %d\n", state);
      Serial.print("[MQTT] Error meaning: ");
      switch(state) {
        case -4: Serial.println("MQTT_CONNECTION_TIMEOUT"); break;
        case -3: Serial.println("MQTT_CONNECTION_LOST"); break;
        case -2: Serial.println("MQTT_CONNECT_FAILED"); break;
        case -1: Serial.println("MQTT_DISCONNECTED"); break;
        case 1:  Serial.println("MQTT_CONNECT_BAD_PROTOCOL"); break;
        case 2:  Serial.println("MQTT_CONNECT_BAD_CLIENT_ID"); break;
        case 3:  Serial.println("MQTT_CONNECT_UNAVAILABLE"); break;
        case 4:  Serial.println("MQTT_CONNECT_BAD_CREDENTIALS"); break;
        case 5:  Serial.println("MQTT_CONNECT_UNAUTHORIZED"); break;
        default: Serial.println("UNKNOWN"); break;
      }
    }
  }
}
 
void handleRoot() {
  server.send(200, "text/plain", "SUT BUS LOGGER - Running");
}
 
void handleDownload() {
  File file = SD.open(filename);
  if (file) {
    server.streamFile(file, "text/csv");
    file.close();
  } else {
    server.send(404, "text/plain", "File Not Found");
  }
}
 
void handleDelete() {
  SD.remove(filename);
  server.send(200, "text/plain", "Log File Deleted");
}

// =============================================================================
// MQTT Callback for OTA Commands
// =============================================================================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.printf("📨 MQTT [%s]: %s\n", topic, message.c_str());
  
  // Check for OTA update command
  #ifdef OTA_ENABLED
  if (String(topic).indexOf("ota") >= 0) {
    // Parse JSON using ArduinoJson (already included in this sketch)
    StaticJsonDocument<256> otaDoc;
    DeserializationError error = deserializeJson(otaDoc, message);
    
    if (!error && otaDoc.containsKey("url") && otaDoc.containsKey("version")) {
      otaUrl = otaDoc["url"].as<String>();
      otaVersion = otaDoc["version"].as<String>();
      
      Serial.printf("📥 OTA Update requested: v%s\n", otaVersion.c_str());
      Serial.printf("📥 Firmware URL: %s\n", otaUrl.c_str());
      
      // Check if we need to update
      bool forceUpdate = otaDoc["force"] | false;
      if (otaVersion == FIRMWARE_VERSION && !forceUpdate) {
        Serial.println("ℹ️ Already on this version, skipping update");
        return;
      }
      
      otaPending = true;
    }
  }
  #endif
}

// =============================================================================
// OTA (Over-The-Air) Update Functions
// =============================================================================

#ifdef OTA_ENABLED
void performOTA() {
  otaPending = false;
  
  if (otaUrl.length() == 0) {
    Serial.println("❌ OTA URL is empty");
    return;
  }
  
  Serial.println("🔄 Starting OTA update...");
  Serial.printf("📥 Downloading: %s\n", otaUrl.c_str());
  
  // Publish OTA status to MQTT
  StaticJsonDocument<128> statusDoc;
  statusDoc["status"] = "downloading";
  statusDoc["version"] = otaVersion;
  statusDoc["current"] = FIRMWARE_VERSION;
  char statusBuf[128];
  serializeJson(statusDoc, statusBuf);
  client.publish(mqtt_topic, statusBuf);
  
  // Perform HTTP OTA update
  WiFiClient updateClient;
  httpUpdate.rebootOnUpdate(false);
  
  t_httpUpdate_return ret = httpUpdate.update(updateClient, otaUrl);
  
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("❌ OTA Failed! Error (%d): %s\n", 
                    httpUpdate.getLastError(), 
                    httpUpdate.getLastErrorString().c_str());
      
      statusDoc.clear();
      statusDoc["status"] = "failed";
      statusDoc["error"] = httpUpdate.getLastErrorString();
      serializeJson(statusDoc, statusBuf);
      client.publish(mqtt_topic, statusBuf);
      break;
      
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("ℹ️ No updates available");
      break;
      
    case HTTP_UPDATE_OK:
      Serial.println("✅ OTA Update successful! Rebooting...");
      
      statusDoc.clear();
      statusDoc["status"] = "success";
      statusDoc["version"] = otaVersion;
      statusDoc["rebooting"] = true;
      serializeJson(statusDoc, statusBuf);
      client.publish(mqtt_topic, statusBuf);
      client.loop();
      
      delay(2000);
      ESP.restart();
      break;
  }
  
  otaUrl = "";
  otaVersion = "";
}
#endif