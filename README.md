# SUT Smart Bus - Hardware (ESP32)

ESP32 firmware for the SUT Smart Bus tracking system.

## Components

### 1. ESP32-CAM (`esp32_cam/`)
Person detection and counting module mounted at the bus door.

**Features:**
- Motion-based person detection
- Enter/exit counting
- MQTT publishing
- SD card image logging
- Ring bell (buzzer) support

### 2. GPS Module (`get_mac_address/`)
MAC address utility for device registration.

### 3. PM Sensor (`pm/`)
Air quality sensor integration (optional).

## Getting Started

### Prerequisites

- Arduino IDE 2.x
- ESP32 Board Support Package
- Libraries:
  - `PubSubClient` (MQTT)
  - `WiFi`

### ESP32-CAM Setup

1. **Configure credentials:**
   ```bash
   cd esp32_cam
   cp config.h.example config.h
   nano config.h
   ```

2. **Edit `config.h`:**
   ```cpp
   #define WIFI_SSID          "your_wifi"
   #define WIFI_PASSWORD      "your_password"
   #define MQTT_SERVER        "your_server_ip"
   #define MQTT_PORT          1883
   ```

3. **Upload to ESP32-CAM:**
   - Select board: `AI-Thinker ESP32-CAM`
   - Connect via USB-TTL adapter
   - Press BOOT button while uploading
   - Click Upload in Arduino IDE

### Hardware Connections

#### ESP32-CAM Pinout

| GPIO | Function |
|------|----------|
| GPIO 12 | Detection LED |
| GPIO 13 | Buzzer (ring bell) |
| GPIO 2-27 | Camera module |

## Configuration Options

```cpp
// Detection zones (adjust for your door placement)
#define LEFT_ZONE  140
#define RIGHT_ZONE 180

// Detection sensitivity
#define MOTION_THRESHOLD 100
#define COOLDOWN_MS 2000
```

## MQTT Topics

| Topic | Direction | Description |
|-------|-----------|-------------|
| `sut/person-detection` | Publish | Person count updates |
| `sut/bus/ESP32-CAM-01/status` | Publish | Device status/RSSI |
| `sut/bus/ring` | Subscribe | Ring bell command |

## Troubleshooting

### Camera not initializing
- Check power supply (5V 2A recommended)
- Verify camera ribbon cable connection
- Try lower XCLK frequency

### WiFi not connecting
- Verify SSID/password in `config.h`
- Check 2.4GHz network availability
- Move closer to router

### MQTT not connecting
- Verify server IP in `config.h`
- Check if MQTT broker is running
- Verify port 1883 is open

## Related Repositories

- [sut-smart-bus-app](https://github.com/YOUR_USERNAME/sut-smart-bus-app) - Mobile app
- [sut-smart-bus-server](https://github.com/YOUR_USERNAME/sut-smart-bus-server) - Backend API

## License

MIT License
