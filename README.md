# AquaLevel Pro – ESP32 Aqual Monitoring System

![Dashboard Screenshot](/images/screenshot1.png)

A smart, battery-aware water level monitor for residential or industrial tanks—powered by ESP32 with deep sleep, MQTT, OTA, and a responsive web dashboard.

## ✨ Features

- Real-time water level & volume calculation
- Built-in support for common tank sizes (500L–2000L) or custom dimensions
- Battery monitoring via AXP192/AXP202 (for TTGO T-Call/SIM800L boards)
- **15-minute active window** + **20-minute deep sleep** (configurable)
- MQTT publishing to `test.mosquitto.org` (or your own broker)
- Web UI with live gauge, tank visualization, and system stats
- ESP-NOW peer support for remote queries
- OTA updates, WiFi setup, calibration, and benchmarking tools
- Serial CLI for diagnostics (`read`, `benchmark`, `battery`, etc.)

## 📦 Hardware

- ESP32 (e.g., TTGO T-Call with AXP192)
- UART-based ultrasonic sensor (9600 baud, 0x55 trigger protocol)
- Li-ion battery (optional but recommended for sleep mode)

## ⚙️ Setup

1. Upload the code via Arduino IDE (ensure ESP32 board support is installed)
2. On first boot:
   - If no WiFi is saved, it creates an AP: **`AquaLevelPro_XXXX`** / password: `12345678`
   - Visit `http://192.168.4.1` to configure WiFi, tank size, sleep, and MQTT
3. Once connected, access the dashboard via your local IP
4. Default OTA password: `12345678`

## 🔋 Battery & Sleep

- Deep sleep conserves power between active windows
- Auto-enters extended sleep if battery < 3.0V
- Real-time battery voltage and % shown on dashboard

## 🌐 MQTT Output

**Topic:** `{device_name}/status`  
**Payload example:**
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
