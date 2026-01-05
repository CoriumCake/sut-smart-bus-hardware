# SUT Smart Bus - Hardware (ESP32)

ESP32 firmware for the SUT Smart Bus tracking system.

## Components

### 1. ESP32-CAM (`esp32_cam/`)
Person detection and counting module mounted at the bus door.

**Features:**
- Motion-based person detection (enter/exit counting)
- MQTT publishing to server
- SD card image logging with timestamps
- Ring bell (buzzer) support
- OTA firmware updates
- WiFi fallback networks

**MQTT Topics:**
| Topic | Direction | Description |
|-------|-----------|-------------|
| `bus/door/count` | Publish | `{"dir":"enter/exit","count":N}` |
| `sut/bus/ESP32-CAM-01/status` | Publish | Device status/RSSI |
| `sut/bus/ring` | Subscribe | Ring bell command |
| `sut/ota/esp32_cam` | Subscribe | OTA update commands |

---

### 2. PM Sensor (`pm/`)
GPS tracking + Air quality monitoring - the main bus tracking unit.

**Features:**
- GPS location tracking (NEO-6M module)
- PM2.5/PM10 air quality (PMS5003/7003)
- Temperature & humidity (DHT11)
- SD card data logging (CSV format)
- RTC timestamping (DS3231)
- OTA firmware updates
- Web interface for data download

**MQTT Topics:**
| Topic | Direction | Description |
|-------|-----------|-------------|
| `sut/bus/gps` | Publish | Full data (GPS, PM, temp, humidity) |
| `sut/bus/gps/fast` | Publish | GPS only (every 500ms) |
| `sut/ota/pm` | Subscribe | OTA update commands |

---

### 3. MAC Address Utility (`get_mac_address/`)
Simple sketch to print the ESP32's MAC address for device registration.

---

## Getting Started

### Prerequisites

- Arduino IDE 2.x
- ESP32 Board Support Package
- Required Libraries:
  - `PubSubClient` (MQTT)
  - `TinyGPS++` (GPS parsing)
  - `ArduinoJson` (JSON handling)
  - `DHT` (Temperature/humidity)
  - `RTClib` (Real-time clock)

### Setup Instructions

#### ESP32-CAM

1. **Copy config template:**
   ```bash
   cd esp32_cam
   cp config.h.example config.h
   ```

2. **Edit `config.h`:**
   ```cpp
   #define WIFI_SSID          "your_wifi"
   #define WIFI_PASSWORD      "your_password"
   #define MQTT_SERVER        "your_server_ip"  // or Docker host IP
   #define MQTT_PORT          1883
   ```

3. **Upload:**
   - Board: `AI-Thinker ESP32-CAM`
   - Connect via USB-TTL adapter (GPIO0 → GND for flash mode)
   - Upload, then disconnect GPIO0 and reset

#### PM Sensor

1. **Copy config template:**
   ```bash
   cd pm
   cp config.h.example config.h
   ```

2. **Edit `config.h`:**
   ```cpp
   #define WIFI_SSID     "your_wifi"
   #define WIFI_PASSWORD "your_password"
   #define MQTT_SERVER   "your_server_ip"
   #define BUS_NAME      "SUT-BUS-01"
   ```

3. **Upload:**
   - Board: `ESP32 Dev Module`
   - Upload normally

---

## Hardware Pinout

### ESP32-CAM

| GPIO | Function |
|------|----------|
| 12 | Detection LED |
| 13 | Buzzer (ring bell) |
| 2-27 | Camera module (AI-Thinker) |

### PM Sensor Module

| GPIO | Function |
|------|----------|
| 32, 33 | GPS (RX, TX) |
| 16, 17 | PMS sensor (RX, TX) |
| 14 | DHT11 data |
| 5 | SD card CS |
| 21, 22 | I2C (SDA, SCL) for RTC |

---

## OTA Updates

Both devices support Over-The-Air firmware updates via MQTT:

1. Compile firmware in Arduino IDE (Sketch → Export Compiled Binary)
2. Upload `.bin` to server via `/api/firmware/upload`
3. Trigger update via `/api/ota/trigger`
4. Device downloads and installs automatically

---

## Troubleshooting

### Camera not initializing
- Check power supply (5V 2A recommended)
- Verify camera ribbon cable connection
- Ensure GPIO0 is disconnected after flashing

### WiFi not connecting
- Verify SSID/password in `config.h`
- Check 2.4GHz network (ESP32 doesn't support 5GHz)
- Check serial monitor for connection status

### MQTT not connecting
- Verify server IP in `config.h`
- Ensure MQTT broker is running (`docker-compose ps`)
- Check port 1883 is open in firewall

### GPS not getting fix
- Move to outdoor location with clear sky view
- Wait 1-2 minutes for cold start
- Check serial monitor for NMEA sentences

---

## Related Repositories

- [sut-smart-bus-server](../sut-smart-bus-server) - Backend API (FastAPI + Docker)
- [sut-smart-bus-app](../sut-smart-bus-app) - Mobile app (React Native/Expo)

## License

MIT License
