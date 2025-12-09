# Aqual Pro – ESP32 Aqual Level Monitor
![Dashboard Screenshot](/images/mainscreen.gif)

A smart, battery-aware water level monitoring system for residential or industrial tanks—built on ESP32 with deep sleep, MQTT, OTA updates, and a responsive web dashboard.

## ✨ Features

- Real-time water **level** and **volume** calculation
- Built-in presets for common tank sizes (500L–2000L) or **custom dimensions**
- **Battery monitoring** via AXP192/AXP202 (optimized for TTGO T-Call boards)
- **Smart deep sleep**: 15-minute active window + 20-minute sleep (configurable)
- **MQTT publishing** to `test.mosquitto.org` (or your own broker)
- **Responsive web UI** with live gauge, tank visualization, and system stats
- **ESP-NOW support** for remote sensor queries
- **OTA updates**, WiFi setup, calibration, and benchmarking tools
- **Serial CLI** for diagnostics: `read`, `benchmark`, `battery`, `reset`, etc.

## 📦 Hardware

- ESP32 (e.g., **TTGO T-Call** with AXP192 power management)
- **UART-based ultrasonic sensor** (e.g., JSN-SR04T, 9600 baud, `0x55` protocol)
- Li-ion battery (recommended for sleep mode)

### Pinout

| Function       | ESP32 Pin |
|----------------|-----------|
| Sensor TX      | GPIO 18   |
| Sensor RX      | GPIO 19   |
| Force AP Mode  | GPIO 27   |
| Blue LED       | GPIO 12   |
| Red LED        | GPIO 14   |
| Green LED      | GPIO 15   |
| I²C SDA (AXP)  | GPIO 21   |
| I²C SCL (AXP)  | GPIO 22   |

> **Note**: The TTGO T-Call board includes built-in power management (AXP192) and LEDs—no external components needed.

## ⚙️ Setup

1. Flash the firmware using Arduino IDE (ESP32 board support required)
2. On first boot:
   - If no WiFi is saved, it creates an AP: **`AquaLevelPro_XXXX`**  
     **Password**: `12345678`
   - Visit `http://192.168.4.1` to configure WiFi, tank size, sleep, and MQTT
3. After setup, access the dashboard via your local IP
4. **OTA password**: `12345678`

## 🔋 Battery & Sleep Logic

- Deep sleep conserves power between active windows
- Auto-enters **1-hour deep sleep** if battery drops below **3.0V**
- Battery voltage and charge % displayed on dashboard
- Sleep schedule fully configurable via web UI

## 🌐 MQTT Output

**Topic**: `{device_name}/status`  
**Example payload**:
```json
{
  "timestamp": "05-12-2025T14:30",
  "distance_cm": 85.2,
  "level_percent": 47.3,
  "volume_liters": 473.0,
  "battery_voltage": 3.85,
  "battery_percentage": 82,
  "next_online": "05-12-2025T14:55"
}
