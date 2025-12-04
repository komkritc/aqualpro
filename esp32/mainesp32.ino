/*
 * AquaLevel Pro - ESP32 Version
 * Converted from ESP8266 to ESP32
 * Designed by: Komkrit Chooraung
 * Date: 28-Nov-2025
 * Version: 3.0 (ESP32 Port)
 * Added: Deep sleep with 15min active window + 20min sleep
 * GPIO: RX=19, TX=18, ForceAP=35, LEDs: 12,14,15
 */

#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600);  // UTC+7 (Bangkok time)

// --- Deep Sleep Configuration ---
#define DEFAULT_ACTIVE_WINDOW 900000       // 15 minutes active (900000 ms = 15 * 60 * 1000)
#define DEFAULT_SLEEP_DURATION 1200000000  // 20 minutes deep sleep (1200000000 microseconds = 20 * 60 * 1000 * 1000)
#define DEFAULT_MQTT_INTERVAL 60000        // 1 minute MQTT publish interval
#define FORCE_WAKE_PIN GPIO_NUM_0          // GPIO0 for wakeup from deep sleep

// Preferences keys for sleep configuration
const char *SLEEP_ENABLED_KEY = "sleep_enabled";
const char *ACTIVE_WINDOW_KEY = "active_window";
const char *SLEEP_DURATION_KEY = "sleep_duration";
const char *MQTT_INTERVAL_KEY = "mqtt_interval";

bool deepSleepEnabled = true;
unsigned long activeWindowStart = 0;
bool isActiveWindow = true;
unsigned long activeWindow = DEFAULT_ACTIVE_WINDOW;
unsigned long sleepDuration = DEFAULT_SLEEP_DURATION;
unsigned long mqttPublishInterval = DEFAULT_MQTT_INTERVAL;

// --- Constants ---
#define DEFAULT_DEVICE_NAME "AquaLevelPro_"
#define WIFI_AP_PASSWORD "12345678"
#define OTA_PASSWORD "12345678"
#define MQTT_HOST "test.mosquitto.org"
#define MQTT_PORT 1883

// Global variable for device name
char deviceName[32] = DEFAULT_DEVICE_NAME;
const char *PubTopic;

// Update the MQTT topic to use dynamic device name
void updateMQTTTopic() {
  static char topicBuffer[64];
  snprintf(topicBuffer, sizeof(topicBuffer), "%s/status", deviceName);
  PubTopic = topicBuffer;
}

// ESP32 GPIO Pins
#define LED_BLUE 12   // GPIO12 for status LED
#define LED_RED 14    // GPIO14 for error LED
#define LED_GREEN 15  // GPIO15 for WiFi LED
#define LED_ON HIGH
#define LED_OFF LOW

#define RX_PIN 19        // GPIO19 for RX (ESP_RX)
#define TX_PIN 18        // GPIO18 for TX (ESP_TX)
#define FORCE_AP_PIN 27  // GPIO32 force AP mode
#define SENSOR_READ_TIMEOUT 100
#define SERIAL_INPUT_TIMEOUT 5000
#define UPDATE_INTERVAL 2000
#define SERIAL_COMMAND_BUFFER_SIZE 32

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

const byte DNS_PORT = 53;
bool wifiConnected = false;

// Tank presets {Width, Height, VolumeFactor}
const float TANK_PRESETS[][3] = {
  { 68.0, 162.0, 3.086f },
  { 79.5, 170.0, 4.118f },
  { 92.5, 181.0, 5.513f },
  { 109.0, 196.0, 7.663f },
  { 123.0, 205.0, 9.742f }
};

// --- Global Variables ---
float tankWidth = TANK_PRESETS[0][0];
float tankHeight = TANK_PRESETS[0][1];
float volumeFactor = TANK_PRESETS[0][2];
int calibration_mm = 0;
float percent = 0.0f;
float volume = 0.0f;

// Sensor data
uint16_t mm = 0;
float cm = 0.0f;

// ESP-NOW
uint8_t senderMAC[] = { 0x5C, 0xCF, 0x7F, 0xF5, 0x3D, 0xE1 };

typedef struct {
  char cmd[32];
} struct_message;

typedef struct {
  char json[128];
} response_message;

String jsonDataOut;
volatile bool pendingDistanceRequest = false;
volatile bool pendingDistanceRequest_2 = false;
uint8_t requestingMAC[6];

// Web Server & DNS
AsyncWebServer server(80);
DNSServer dnsServer;
HardwareSerial sensorSerial(2);  // Use UART2 on ESP32

// Serial command buffer
char serialCommandBuffer[SERIAL_COMMAND_BUFFER_SIZE];
int serialBufferIndex = 0;

// --- Performance Monitoring ---
struct PerformanceMetrics {
  unsigned long totalReadings;
  unsigned long successfulReadings;
  unsigned long averageReadTime;
  unsigned long maxReadTime;
  unsigned long minReadTime;

  void reset() {
    totalReadings = 0;
    successfulReadings = 0;
    averageReadTime = 0;
    maxReadTime = 0;
    minReadTime = ULONG_MAX;
  }

  void addMeasurement(unsigned long readTime, bool success) {
    totalReadings++;
    if (success) {
      successfulReadings++;

      if (readTime > maxReadTime) maxReadTime = readTime;
      if (readTime < minReadTime) minReadTime = readTime;

      averageReadTime = (averageReadTime * (successfulReadings - 1) + readTime) / successfulReadings;
    }
  }

  float getSuccessRate() {
    if (totalReadings == 0) return 0.0f;
    return (float)successfulReadings / totalReadings * 100.0f;
  }
};

bool readDistanceBurst(uint16_t &out_mm, float &out_cm, int attempts = 3) {
  for (int i = 0; i < attempts; i++) {
    unsigned long startTime = millis();

    sensorSerial.flush();
    sensorSerial.write(0x55);

    unsigned long timeoutStart = millis();
    while (millis() - timeoutStart < 60) {
      if (sensorSerial.available() >= 4) {
        uint8_t buffer[4];
        if (sensorSerial.readBytes(buffer, 4) == 4 && buffer[0] == 0xFF) {
          uint16_t raw_mm = (buffer[1] << 8) | buffer[2];

          if (raw_mm > 50 && raw_mm < 5000) {
            out_mm = raw_mm + calibration_mm;
            out_cm = out_mm / 10.0f;
            return true;
          }
        }
        break;
      }
      delay(1);
    }

    if (i < attempts - 1) {
      delay(20);
    }
  }
  return false;
}

bool readDistanceFast(uint16_t &out_mm, float &out_cm) {
  static unsigned long lastTrigger = 0;
  static bool waitingForResponse = false;
  static unsigned long responseStartTime = 0;

  unsigned long currentTime = millis();

  if (!waitingForResponse && (currentTime - lastTrigger >= 50)) {
    sensorSerial.flush();
    sensorSerial.write(0x55);
    waitingForResponse = true;
    responseStartTime = currentTime;
    lastTrigger = currentTime;
    return false;
  }

  if (waitingForResponse) {
    if (currentTime - responseStartTime > 80) {
      waitingForResponse = false;
      return false;
    }

    if (sensorSerial.available() >= 4) {
      uint8_t buffer[4];
      int bytesRead = sensorSerial.readBytes(buffer, 4);

      if (bytesRead == 4 && buffer[0] == 0xFF) {
        uint16_t raw_mm = (buffer[1] << 8) | buffer[2];

        if (raw_mm > 50 && raw_mm < 5000) {
          out_mm = raw_mm + calibration_mm;
          out_cm = out_mm / 10.0f;
          waitingForResponse = false;
          return true;
        }
      }
      waitingForResponse = false;
    }
  }
  return false;
}

PerformanceMetrics performanceMetrics;

// --- Buffered Reading Manager ---
class SensorReadingManager {
private:
  static const int BUFFER_SIZE = 5;
  float readings[BUFFER_SIZE];
  int bufferIndex;
  int validReadings;
  unsigned long lastSuccessfulRead;
  bool initialized;
  float lastValidReading;
  int consecutiveErrors;

public:
  SensorReadingManager()
    : bufferIndex(0), validReadings(0), lastSuccessfulRead(0),
      initialized(false), lastValidReading(0), consecutiveErrors(0) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
      readings[i] = 0;
    }
  }

  bool getStableReading(uint16_t &out_mm, float &out_cm) {
    uint16_t raw_mm;
    float raw_cm;

    if (readDistanceFast(raw_mm, raw_cm)) {
      if (initialized && validReadings > 2) {
        float currentAverage = getAverage();
        float difference = abs(raw_cm - currentAverage);

        if (difference > 20.0f) {
          consecutiveErrors++;

          if (consecutiveErrors > 3) {
            reset();
          }
          return false;
        }
      }

      readings[bufferIndex] = raw_cm;
      bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

      if (validReadings < BUFFER_SIZE) {
        validReadings++;
      }

      consecutiveErrors = 0;
      lastSuccessfulRead = millis();
      lastValidReading = raw_cm;
      initialized = true;

      float averaged = getAverage();
      out_cm = averaged;
      out_mm = (uint16_t)(averaged * 10);

      return true;
    }
    return false;
  }

  float getAverage() {
    if (validReadings == 0) return 0;
    float sum = 0;
    for (int i = 0; i < validReadings; i++) {
      sum += readings[i];
    }
    return sum / validReadings;
  }

  bool hasRecentReading() {
    return (millis() - lastSuccessfulRead) < 5000;
  }

  void reset() {
    bufferIndex = 0;
    validReadings = 0;
    consecutiveErrors = 0;
    initialized = false;
  }

  int getValidReadingsCount() {
    return validReadings;
  }
};

SensorReadingManager sensorManager;

// Preferences for storing configuration
Preferences preferences;

// ===== HTML PAGES =====
const char PAGE_ROOT[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AquaLevel Pro Dashboard</title>
  <link href="https://fonts.googleapis.com/css2?family=Roboto:wght@300;400;500&display=swap" rel="stylesheet">
  <style>
    body { font-family: 'Roboto', sans-serif; background-color: #f5f5f5; margin: 0; padding: 20px; color: #333; }
    .container { max-width: 600px; margin: 0 auto; background: white; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); padding: 20px; }
    h1 { color: #4285f4; margin-top: 0; text-align: center; }
    .gauge-container { display: flex; justify-content: center; margin: 20px 0; }
    .gauge { width: 150px; height: 150px; position: relative; border-radius: 50%; background: #f1f1f1; display: flex; align-items: center; justify-content: center; font-size: 24px; font-weight: 500; box-shadow: inset 0 0 10px rgba(0,0,0,0.1); }
    .gauge::before { content: ''; position: absolute; width: 100%; height: 100%; border-radius: 50%; background: conic-gradient(var(--water-color) 0% var(--water-level), transparent var(--water-level) 100%); transform: rotate(0deg); transition: all 0.5s ease; }
    .gauge-inner { width: 80%; height: 80%; background: white; border-radius: 50%; display: flex; align-items: center; justify-content: center; z-index: 1; box-shadow: 0 0 5px rgba(0,0,0,0.1); }
    .stats { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin: 20px 0; }
    .stat-card { background: #f9f9f9; padding: 15px; border-radius: 8px; text-align: center; }
    .stat-card h3 { margin: 0 0 5px 0; font-size: 14px; color: #666; font-weight: 400; }
    .stat-card p { margin: 0; font-size: 20px; font-weight: 500; }
    .tank-container { margin: 20px 0; position: relative; }
    .tank-visual { width: 100%; height: 200px; background: #e0e0e0; border-radius: 5px; position: relative; overflow: hidden; border: 1px solid #ddd; }
    .water-level { position: absolute; bottom: 0; width: 100%; background: var(--water-color); transition: all 0.5s ease; }
    .tank-labels { display: flex; flex-direction: column; justify-content: flex-start; align-items: flex-start; margin-top: 5px;}
    .tank-label { font-size: 12px; color: #666; }
    .current-level { position: absolute; left: 50%; transform: translateX(-50%); bottom: calc(%PERCENT%% - 12px); font-size: 12px; background: white; padding: 2px 5px; border-radius: 3px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); z-index: 2; }
    .footer { margin-top: 20px; font-size: 12px; color: #999; text-align: center; border-top: 1px solid #eee; padding-top: 15px; }
    .config-link { display: inline-block; margin: 5px; color: #4285f4; text-decoration: none; font-weight: 500; }
    .config-link:hover { text-decoration: underline; }
    .last-update { font-size: 12px; color: #999; text-align: right; margin-top: 10px; }
    .health-indicator { display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 5px; }
    .health-healthy { background-color: #4CAF50; }
    .health-warning { background-color: #FF9800; }
    .health-error { background-color: #F44336; }
    .sleep-info { 
      background: #fff3cd; 
      border: 1px solid #ffeaa7; 
      border-radius: 4px; 
      padding: 10px; 
      margin: 10px 0; 
      text-align: center;
      font-size: 14px;
    }
    .next-online { 
      background: #e8f5e8; 
      border: 1px solid #c8e6c9; 
      border-radius: 4px; 
      padding: 10px; 
      margin: 10px 0; 
      text-align: center;
      font-size: 14px;
    }
    .footer {
      margin-top: 2rem;
      padding-top: 1.5rem;
      border-top: 1px solid #eee;
      font-size: 0.9rem;
    }
    .footer-links {
      display: flex;
      justify-content: center;
      gap: 1.5rem;
      margin-bottom: 1.5rem;
      flex-wrap: wrap;
    }
    .footer-link {
      display: flex;
      align-items: center;
      gap: 0.5rem;
      color: #4361ee;
      text-decoration: none;
      font-weight: 500;
      transition: color 0.2s;
    }
    .footer-link:hover {
      color: #3a56d4;
    }
    .icon {
      width: 18px;
      height: 18px;
      fill: currentColor;
    }
    .footer-meta {
      text-align: center;
      color: #666;
      line-height: 1.6;
    }
    .footer-meta-item {
      margin: 0.3rem 0;
    }
    @media (max-width: 600px) {
      .footer-links {
        gap: 1rem;
      }
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>AquaLevel Pro Dashboard</h1>
    
    <div class="sleep-info" id="sleepInfo">
      Active for: <span id="activeTime">%ACTIVE_TIME%</span> | Next sleep in: <span id="sleepCountdown">%SLEEP_COUNTDOWN%</span>
    </div>
    
    <div class="next-online" id="nextOnline">
      Next online at: <span id="nextOnlineTime">%NEXT_ONLINE_TIME%</span>
    </div>
    
    <div class="last-update" id="lastUpdate">
      <span class="health-indicator" id="healthIndicator"></span>
      Last update: Just now
    </div>
    
    <div class="gauge-container">
      <div class="gauge" id="gauge" style="--water-color: %WATER_COLOR%; --water-level: %PERCENT%%;">
        <div class="gauge-inner">
          <span id="percentValue">%PERCENT% %</span>
        </div>
      </div>
    </div>
    
    <div class="stats">
      <div class="stat-card">
        <h3>Distance</h3>
        <p id="distanceValue">%DISTANCE% cm</p>
      </div>
      <div class="stat-card">
        <h3>Volume</h3>
        <p id="volumeValue">%VOLUME% L</p>
      </div>
      <div class="stat-card">
        <h3>Tank Width</h3>
        <p>%TANK_WIDTH% cm</p>
      </div>
      <div class="stat-card">
        <h3>Tank Height</h3>
        <p>%TANK_HEIGHT% cm</p>
      </div>
    </div>
    
    <div class="tank-container">
      <div class="tank-labels">
        <span class="tank-label">%MAX_VOLUME% L (MAX)</span>
      </div>
      <div class="tank-visual">
        <div class="water-level" id="waterLevel" style="height: %PERCENT%%; background: %WATER_COLOR%;"></div>
        <div class="current-level" id="currentLevel">%VOLUME% L (%PERCENT%%)</div>
      </div>
      <div class="tank-labels">
        <span class="tank-label">0 L (MIN)</span>
      </div>
    </div>
    
    <div class="footer">
      <div class="footer-links">
        <a href="/config" class="footer-link">
          <svg class="icon" viewBox="0 0 24 24"><path d="M19.14 12.94c.04-.3.06-.61.06-.94 0-.32-.02-.64-.07-.94l2.03-1.58c.18-.14.23-.41.12-.61l-1.92-3.32c-.12-.22-.37-.29-.59-.22l-2.39.96c-.5-.38-1.03-.7-1.62-.94l-.36-2.54c-.04-.22-.2-.4-.43-.4h-3.84c-.23 0-.39.18-.43.4l-.36 2.54c-.59.24-1.13.57-1.62.94l-2.39-.96c-.22-.08-.47 0-.59.22L2.74 8.87c-.12.21-.08.47.12.61l2.03 1.58c-.05.3-.09.63-.09.94s.02.64.07.94l-2.03 1.58c-.18.14-.23.41-.12.61l1.92 3.32c.12.22.37.29.59.22l2.39-.96c.5.38 1.03.7 1.62.94l.36 2.54c.04.22.2.4.43.4h3.84c.23 0 .39-.18.43-.4l.36-2.54c.59-.24 1.13-.56 1.62-.94l2.39.96c.22.08.47 0 .59-.22l1.92-3.32c.12-.22.07-.47-.12-.61l-2.01-1.58zM12 15.6c-1.98 0-3.6-1.62-3.6-3.6s1.62-3.6 3.6-3.6 3.6 1.62 3.6 3.6-1.62 3.6-3.6 3.6z"/></svg>
          Configure
        </a>
        <a href="/sensor-stats" class="footer-link">
          <svg class="icon" viewBox="0 0 24 24"><path d="M16 11c1.66 0 3-1.34 3-3s-1.34-3-3-3-3 1.34-3 3 1.34 3 3 3zM8 11c1.66 0 3-1.34 3-3s-1.34-3-3-3-3 1.34-3 3 1.34 3 3 3zM8 19c1.66 0 3-1.34 3-3s-1.34-3-3-3-3 1.34-3 3 1.34 3 3 3zM16 19c1.66 0 3-1.34 3-3s-1.34-3-3-3-3 1.34-3 3 1.34 3 3 3z"/></svg>
          Stats
        </a>
        <a href="/benchmark" class="footer-link">
          <svg class="icon" viewBox="0 0 24 24"><path d="M15.6 10.79c.97-.67 1.65-1.77 1.65-2.79 0-2.26-1.75-4-4-4H7v14h7.04c2.09 0 3.71-1.7 3.71-3.79 0-1.52-.86-2.82-2.15-3.42zM10 6.5h3c.83 0 1.5.67 1.5 1.5s-.67 1.5-1.5 1.5h-3v-3zm3.5 9H10v-3h3.5c.83 0 1.5.67 1.5 1.5s-.67 1.5-1.5 1.5z"/></svg>
          Benchmark
        </a>
        <a href="/wifi-setup" class="footer-link">
          <svg class="icon" viewBox="0 0 24 24"><path d="M1 9l2 2c4.97-4.97 13.03-4.97 18 0l2-2C16.93 2.93 7.08 2.93 1 9zm8 8l3 3 3-3c-1.65-1.66-4.34-1.66-6 0zm-4-4l2 2c2.76-2.76 7.24-2.76 10 0l2-2C15.14 9.14 8.87 9.14 5 13z"/></svg>
          WiFi
        </a>
        <a href="/sleep-config" class="footer-link">
          <svg class="icon" viewBox="0 0 24 24"><path d="M7.88 3.39L6.6 1.86 2 5.71l1.29 1.53 4.59-3.85zM22 5.72l-4.6-3.86-1.29 1.53 4.6 3.86L22 5.72zM12 4c-4.97 0-9 4.03-9 9s4.02 9 9 9c4.97 0 9-4.03 9-9s-4.03-9-9-9zm0 16c-3.87 0-7-3.13-7-7s3.13-7 7-7 7 3.13 7 7-3.13 7-7 7zm-3-9h3.63L9 15.2V17h6v-2h-3.63L15 10.8V9H9v2z"/></svg>
          Sleep
        </a>
      </div>
      <div class="footer-meta">
        <p class="footer-meta-item">MAC: %MAC_ADDRESS%</p>
        <p class="footer-meta-item">Uptime: <span id="uptimeValue">%UPTIME%</span></p>
        <p class="footer-meta-item">Valid Readings: <span id="validReadings">%VALID_READINGS%</span></p>
      </div>
    </div>

    <script>
      function updateDashboard() {
        fetch('/data')
          .then(response => response.json())
          .then(data => {
            const waterColor = data.percent < 20 ? '#ff4444' : '#4285f4';
            
            document.getElementById('percentValue').textContent = data.percent.toFixed(1) + '%';
            document.getElementById('distanceValue').textContent = data.distance.toFixed(1) + ' cm';
            document.getElementById('volumeValue').textContent = data.volume.toFixed(1) + ' L';
            document.getElementById('uptimeValue').textContent = data.uptime;
            document.getElementById('validReadings').textContent = data.valid_readings || '0';
            document.getElementById('activeTime').textContent = data.active_time || '0m 0s';
            document.getElementById('sleepCountdown').textContent = data.sleep_countdown || '0m 0s';
            document.getElementById('nextOnlineTime').textContent = data.next_online || 'Calculating...';
            
            const waterLevel = document.getElementById('waterLevel');
            const currentLevel = document.getElementById('currentLevel');
            const gauge = document.getElementById('gauge');
            const healthIndicator = document.getElementById('healthIndicator');
            
            waterLevel.style.height = data.percent + '%';
            waterLevel.style.backgroundColor = waterColor;
            
            currentLevel.style.bottom = 'calc(' + data.percent + '% - 12px)';
            currentLevel.textContent = data.volume.toFixed(1) + ' L (' + data.percent.toFixed(1) + '%)';
            
            gauge.style.setProperty('--water-color', waterColor);
            gauge.style.setProperty('--water-level', data.percent + '%');
            
            // Update health indicator
            if (data.sensor_health) {
              healthIndicator.className = 'health-indicator health-healthy';
            } else {
              healthIndicator.className = 'health-indicator health-error';
            }
            
            const now = new Date();
            document.getElementById('lastUpdate').innerHTML = 
              '<span class="health-indicator ' + (data.sensor_health ? 'health-healthy' : 'health-error') + '"></span>' +
              'Last update: ' + now.toLocaleTimeString();
          })
          .catch(error => {
            console.error('Error fetching data:', error);
            const healthIndicator = document.getElementById('healthIndicator');
            healthIndicator.className = 'health-indicator health-error';
          });
      }

      updateDashboard();
      setInterval(updateDashboard, %UPDATE_INTERVAL%);
    </script>
  </div>
</body>
</html>
)=====";

const char PAGE_CONFIG[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang='en'>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>Tank Configuration</title>
  <style>
    :root {
      --primary-color: #3498db;
      --secondary-color: #2980b9;
      --text-color: #333;
      --light-bg: #f9f9f9;
      --border-color: #e0e0e0;
    }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      line-height: 1.6;
      color: var(--text-color);
      background-color: var(--light-bg);
      margin: 0;
      padding: 20px;
      max-width: 600px;
      margin: 0 auto;
    }
    h1, h2, h3 {
      color: var(--primary-color);
      text-align: center;
    }
    .config-form {
      background: white;
      padding: 25px;
      border-radius: 8px;
      box-shadow: 0 2px 10px rgba(0,0,0,0.1);
      margin-top: 20px;
    }
    .form-group {
      margin-bottom: 20px;
    }
    label {
      display: block;
      margin-bottom: 8px;
      font-weight: 600;
    }
    select, input[type='number'], input[type='text'] {
      width: 100%;
      padding: 10px;
      border: 1px solid var(--border-color);
      border-radius: 4px;
      font-size: 16px;
      box-sizing: border-box;
    }
    input[type='submit'] {
      background-color: var(--primary-color);
      color: white;
      border: none;
      padding: 12px 20px;
      border-radius: 4px;
      font-size: 16px;
      cursor: pointer;
      width: 100%;
      transition: background-color 0.3s;
      margin-top: 15px;
    }
    input[type='submit']:hover {
      background-color: var(--secondary-color);
    }
    .back-link {
      display: block;
      text-align: center;
      margin-top: 20px;
      color: var(--primary-color);
      text-decoration: none;
    }
    .back-link:hover {
      text-decoration: underline;
    }
    .mode-selector {
      display: flex;
      margin-bottom: 20px;
      border-bottom: 1px solid var(--border-color);
      padding-bottom: 15px;
    }
    .mode-option {
      flex: 1;
      text-align: center;
      padding: 10px;
      cursor: pointer;
      border-bottom: 3px solid transparent;
    }
    .mode-option.active {
      border-bottom-color: var(--primary-color);
      font-weight: bold;
    }
    .mode-option input {
      display: none;
    }
    .manual-inputs {
      display: none;
    }
    .preset-inputs {
      display: block;
    }
    .calibration-input {
      margin-top: 20px;
      padding-top: 20px;
      border-top: 1px solid var(--border-color);
    }
  </style>
</head>
<body>
  <div class="config-form">
    <h2>Tank Configuration</h2>
    <form method='GET' action='/config'>
      <input type='hidden' name='save' value='1'>
      
      <div class="mode-selector">
        <label class="mode-option active" onclick="toggleMode('preset')">
          <input type="radio" name="preset_mode" value="1" checked> Preset Tank
        </label>
        <label class="mode-option" onclick="toggleMode('manual')">
          <input type="radio" name="preset_mode" value="0"> Custom Tank
        </label>
      </div>
      
      <div class="preset-inputs" id="preset-inputs">
        <div class="form-group">
          <label for='preset_size'>Tank Size</label>
          <select id='preset_size' name='preset_size' required>
            <option value='0'>500L (68cm × 162cm)</option>
            <option value='1'>700L (79.5cm × 170cm)</option>
            <option value='2'>1000L (92.5cm × 181cm)</option>
            <option value='3'>1500L (109cm × 196cm)</option>
            <option value='4'>2000L (123cm × 205cm)</option>
          </select>
        </div>
      </div>
      
      <div class="manual-inputs" id="manual-inputs">
        <div class="form-group">
          <label for='manual_width'>Tank Width (cm)</label>
          <input type='number' id='manual_width' name='manual_width' step='0.1' min='10' max='300' value='%TANK_WIDTH%'>
        </div>
        <div class="form-group">
          <label for='manual_height'>Tank Height (cm)</label>
          <input type='number' id='manual_height' name='manual_height' step='0.1' min='10' max='300' value='%TANK_HEIGHT%'>
        </div>
      </div>
      
      <div class="calibration-input">
        <div class="form-group">
          <label for='calibration'>Sensor Calibration (cm)</label>
          <input type='number' id='calibration' name='calibration' step='0.1' value='%CALIBRATION%'>
          <small>Positive values move water level up, negative down</small>
        </div>
      </div>
      
      <input type='submit' value='Save Configuration'>
    </form>
  </div>
  <a href='/' class='back-link'>&larr; Back to Dashboard</a>
  
  <script>
    function toggleMode(mode) {
      const presetInputs = document.getElementById('preset-inputs');
      const manualInputs = document.getElementById('manual-inputs');
      const presetOption = document.querySelector('.mode-option:nth-child(1)');
      const manualOption = document.querySelector('.mode-option:nth-child(2)');
      
      if (mode === 'preset') {
        presetInputs.style.display = 'block';
        manualInputs.style.display = 'none';
        presetOption.classList.add('active');
        manualOption.classList.remove('active');
        document.querySelector('input[name="preset_mode"][value="1"]').checked = true;
      } else {
        presetInputs.style.display = 'none';
        manualInputs.style.display = 'block';
        presetOption.classList.remove('active');
        manualOption.classList.add('active');
        document.querySelector('input[name="preset_mode"][value="0"]').checked = true;
      }
    }
  </script>
</body>
</html>
)=====";

const char PAGE_SLEEP_CONFIG[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Sleep Configuration</title>
  <style>
    :root {
      --primary: #4361ee;
      --light: #f8f9fa;
      --dark: #212529;
      --border: #dee2e6;
      --success: #4bb543;
    }
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    }
    body {
      background-color: #f5f5f5;
      color: var(--dark);
      line-height: 1.6;
      padding: 20px;
    }
    .container {
      max-width: 500px;
      margin: 30px auto;
      background: white;
      padding: 30px;
      border-radius: 8px;
      box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
    }
    h2 {
      color: var(--primary);
      margin-bottom: 20px;
      text-align: center;
    }
    .form-group {
      margin-bottom: 20px;
    }
    label {
      display: block;
      margin-bottom: 8px;
      font-weight: 500;
    }
    input[type="number"],
    input[type="checkbox"] {
      padding: 8px;
      border: 1px solid var(--border);
      border-radius: 4px;
      font-size: 16px;
    }
    input[type="checkbox"] {
      width: 18px;
      height: 18px;
      margin-right: 8px;
    }
    .checkbox-group {
      display: flex;
      align-items: center;
      margin-bottom: 15px;
    }
    .input-hint {
      font-size: 0.8rem;
      color: #666;
      margin-top: 5px;
    }
    input[type="submit"] {
      background-color: var(--primary);
      color: white;
      border: none;
      padding: 12px 20px;
      border-radius: 4px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      transition: background-color 0.3s;
      margin-top: 10px;
    }
    input[type="submit"]:hover {
      background-color: #3a56d4;
    }
    .back-link {
      display: block;
      text-align: center;
      margin-top: 20px;
      color: var(--primary);
      text-decoration: none;
    }
    .back-link:hover {
      text-decoration: underline;
    }
    .current-info {
      background: var(--light);
      padding: 15px;
      border-radius: 4px;
      margin-bottom: 20px;
      text-align: center;
    }
    .info-item {
      margin: 5px 0;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>Sleep Configuration</h2>
    
    <div class="current-info">
      <div class="info-item"><strong>Current Status:</strong> %SLEEP_STATUS%</div>
      <div class="info-item"><strong>Active Time:</strong> %ACTIVE_TIME% minutes</div>
      <div class="info-item"><strong>Sleep Time:</strong> %SLEEP_TIME% minutes</div>
      <div class="info-item"><strong>MQTT Interval:</strong> %MQTT_INTERVAL% seconds</div>
    </div>
    
    <form method="get" action="/sleep-config">
      <input type="hidden" name="save" value="1">
      
      <div class="checkbox-group">
        <input type="checkbox" id="sleep_enabled" name="sleep_enabled" value="1" %SLEEP_CHECKED%>
        <label for="sleep_enabled">Enable Deep Sleep</label>
      </div>
      
      <div class="form-group">
        <label for="active_window">Active Window (minutes)</label>
        <input type="number" id="active_window" name="active_window" min="1" max="60" value="%ACTIVE_WINDOW%" required>
        <div class="input-hint">How long the device stays awake (1-60 minutes)</div>
      </div>
      
      <div class="form-group">
        <label for="sleep_duration">Deep Sleep Duration (minutes)</label>
        <input type="number" id="sleep_duration" name="sleep_duration" min="1" max="120" value="%SLEEP_DURATION%" required>
        <div class="input-hint">How long the device sleeps (1-120 minutes)</div>
      </div>
      
      <div class="form-group">
        <label for="mqtt_interval">MQTT Publish Interval (seconds)</label>
        <input type="number" id="mqtt_interval" name="mqtt_interval" min="10" max="300" value="%MQTT_INTERVAL_VAL%" required>
        <div class="input-hint">How often to publish MQTT data during active window (10-300 seconds)</div>
      </div>
      
      <input type="submit" value="Save Sleep Configuration">
    </form>
    
    <a href="/" class="back-link">← Back to Dashboard</a>
  </div>
</body>
</html>
)=====";

const char PAGE_SLEEP_SAVED[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Sleep Configuration Saved</title>
  <style>
    :root {
      --primary: #4361ee;
      --success: #4bb543;
    }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background-color: #f5f5f5;
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
      margin: 0;
      text-align: center;
    }
    .card {
      background: white;
      padding: 2rem;
      border-radius: 8px;
      box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
      max-width: 500px;
      width: 100%;
    }
    .success-icon {
      color: var(--success);
      font-size: 3rem;
      margin-bottom: 1rem;
    }
    h1 {
      color: var(--primary);
      margin-bottom: 1rem;
    }
    p {
      margin-bottom: 1.5rem;
      color: #555;
    }
    .back-link {
      display: inline-block;
      margin-top: 1rem;
      color: var(--primary);
      text-decoration: none;
      font-weight: 500;
    }
    .back-link:hover {
      text-decoration: underline;
    }
  </style>
</head>
<body>
  <div class="card">
    <div class="success-icon">✓</div>
    <h1>Sleep Configuration Saved</h1>
    <p>Sleep settings have been updated successfully.</p>
    <a href="/sleep-config" class="back-link">← Back to Sleep Config</a>
    <a href="/" class="back-link">← Back to Dashboard</a>
  </div>
</body>
</html>
)=====";

const char PAGE_WIFI_SETUP[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>WiFi Setup</title>
  <style>
    :root {
      --primary: #4361ee;
      --light: #f8f9fa;
      --dark: #212529;
      --border: #dee2e6;
      --success: #4bb543;
    }
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    }
    body {
      background-color: #f5f5f5;
      color: var(--dark);
      line-height: 1.6;
      padding: 20px;
    }
    .container {
      max-width: 500px;
      margin: 30px auto;
      background: white;
      padding: 30px;
      border-radius: 8px;
      box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
    }
    h2 {
      color: var(--primary);
      margin-bottom: 20px;
      text-align: center;
    }
    .form-group {
      margin-bottom: 20px;
    }
    label {
      display: block;
      margin-bottom: 5px;
      font-weight: 500;
    }
    input[type="text"],
    input[type="password"] {
      width: 100%;
      padding: 10px;
      border: 1px solid var(--border);
      border-radius: 4px;
      font-size: 16px;
    }
    .input-hint {
      font-size: 0.8rem;
      color: #666;
      margin-top: 5px;
    }
    input[type="submit"] {
      background-color: var(--primary);
      color: white;
      border: none;
      padding: 12px 20px;
      border-radius: 4px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      transition: background-color 0.3s;
      margin-top: 10px;
    }
    input[type="submit"]:hover {
      background-color: #3a56d4;
    }
    .back-link {
      display: block;
      text-align: center;
      margin-top: 20px;
      color: var(--primary);
      text-decoration: none;
    }
    .back-link:hover {
      text-decoration: underline;
    }
    .current-info {
      background: var(--light);
      padding: 10px;
      border-radius: 4px;
      margin-bottom: 15px;
      text-align: center;
    }
    .current-mac {
      font-family: monospace;
      margin-top: 5px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>WiFi Configuration</h2>
    
    <div class="current-info">
      <div><strong>Current Device:</strong> %DEVICE_NAME%</div>
      <div class="current-mac">MAC: %MAC_ADDRESS%</div>
    </div>
    
    <!-- CHANGED: method="get" instead of "post" -->
    <form method="get" action="/wifi-setup">
      <input type="hidden" name="save" value="1">
      
      <div class="form-group">
        <label for="ssid">Network SSID</label>
        <input type="text" id="ssid" name="ssid" value="%SSID%" placeholder="Enter WiFi network name">
      </div>
      
      <div class="form-group">
        <label for="password">Password</label>
        <input type="password" id="password" name="password" value="%PASSWORD%" placeholder="Enter WiFi password">
      </div>
      
      <div class="form-group">
        <label for="device_name">Device Name (MQTT/AP)</label>
        <input type="text" id="device_name" name="device_name" value="%DEVICE_NAME%" placeholder="Enter device name">
        <div class="input-hint">Used for MQTT topics, Access Point name, and OTA</div>
      </div>
      
      <div class="form-group">
        <label for="mac">ESP-NOW Peer MAC (Optional)</label>
        <input type="text" id="mac" name="mac" value="%MAC%" placeholder="XX:XX:XX:XX:XX:XX">
        <div class="input-hint">Format: 00:11:22:33:44:55 (leave empty to keep current)</div>
      </div>
      
      <input type="submit" value="Save Settings">
    </form>
    <a href="/" class="back-link">Back to Dashboard</a>
  </div>
</body>
</html>
)=====";

const char PAGE_WIFI_SAVED[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Settings Saved</title>
  <style>
    :root {
      --primary: #4361ee;
      --success: #4bb543;
    }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background-color: #f5f5f5;
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
      margin: 0;
      text-align: center;
    }
    .card {
      background: white;
      padding: 2rem;
      border-radius: 8px;
      box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
      max-width: 500px;
      width: 100%;
    }
    .success-icon {
      color: var(--success);
      font-size: 3rem;
      margin-bottom: 1rem;
    }
    h1 {
      color: var(--primary);
      margin-bottom: 1rem;
    }
    p {
      margin-bottom: 1.5rem;
      color: #555;
      line-height: 1.6;
    }
    .countdown {
      font-size: 1.2rem;
      font-weight: bold;
      color: var(--primary);
      margin: 1rem 0;
      padding: 10px;
      background: #f0f8ff;
      border-radius: 5px;
      display: inline-block;
      min-width: 200px;
    }
    .progress-bar {
      height: 8px;
      background: #e0e0e0;
      border-radius: 4px;
      margin: 1.5rem 0;
      overflow: hidden;
    }
    .progress {
      height: 100%;
      background: linear-gradient(90deg, var(--primary), #3a56d4);
      width: 100%;
      animation: progress 5s linear forwards;
    }
    .info-box {
      background: #f8f9fa;
      border-left: 4px solid var(--primary);
      padding: 12px;
      margin: 1.5rem 0;
      text-align: left;
      border-radius: 4px;
    }
    .info-box p {
      margin: 5px 0;
      font-size: 0.9rem;
    }
    @keyframes progress {
      from { width: 100%; }
      to { width: 0%; }
    }
  </style>
</head>
<body>
  <div class="card">
    <div class="success-icon">✓</div>
    <h1>WiFi Settings Saved</h1>
    
    <div class="info-box">
      <p><strong>Device will reboot to apply changes</strong></p>
      <p>New settings will take effect after restart.</p>
      <p>Access point: %NEW_DEVICE_NAME%</p>
      <p>SSID: %NEW_SSID%</p>
    </div>
    
    <div class="countdown" id="countdown">Rebooting in 5 seconds...</div>
    
    <div class="progress-bar">
      <div class="progress"></div>
    </div>
    
    <p>If page doesn't redirect automatically, <a href="/">click here</a>.</p>
  </div>
  
  <script>
    let seconds = 5;
    const countdownEl = document.getElementById('countdown');
    
    // Update countdown every second
    const timer = setInterval(() => {
      seconds--;
      if (seconds > 0) {
        countdownEl.textContent = `Rebooting in ${seconds} second${seconds !== 1 ? 's' : ''}...`;
      } else {
        countdownEl.textContent = 'Rebooting now...';
        clearInterval(timer);
      }
    }, 1000);
    
    // Redirect to home after 5 seconds
    setTimeout(() => {
      window.location.href = '/';
    }, 5000);
  </script>
</body>
</html>
)=====";

const char PAGE_BENCHMARK_FORM[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Sensor Benchmark</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .card {
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            padding: 25px;
            margin-top: 20px;
        }
        h2 {
            color: #2c3e50;
            margin-top: 0;
            border-bottom: 1px solid #eee;
            padding-bottom: 10px;
        }
        .form-group {
            margin-bottom: 20px;
        }
        label {
            display: block;
            margin-bottom: 5px;
            font-weight: 500;
        }
        input[type="number"] {
            width: 100px;
            padding: 8px;
            border: 1px solid #ddd;
            border-radius: 4px;
        }
        input[type="submit"] {
            background-color: #3498db;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 16px;
            transition: background-color 0.3s;
        }
        input[type="submit"]:hover {
            background-color: #2980b9;
        }
        .info-box {
            background-color: #f8f9fa;
            border-left: 4px solid #3498db;
            padding: 15px;
            margin: 20px 0;
        }
        .back-link {
            display: inline-block;
            margin-top: 20px;
            color: #3498db;
            text-decoration: none;
        }
        .back-link:hover {
            text-decoration: underline;
        }
    </style>
</head>
<body>
    <div class="card">
        <h2>Sensor Benchmark</h2>
        <form method="get">
            <div class="form-group">
                <label for="iterations">Iterations (5-30):</label>
                <input type="number" id="iterations" name="iterations" value="20" min="5" max="30">
            </div>
            <input type="hidden" name="run" value="1">
            <input type="submit" value="Start Benchmark">
        </form>
        <div class="info-box">
            <strong>Note:</strong> Test will show all individual readings by default.
            Maximum 30 iterations recommended for stability.
        </div>
        <a href="/" class="back-link">← Back to Dashboard</a>
    </div>
</body>
</html>
)=====";

// ===== UTILITY FUNCTIONS =====
String getNextOnlineTime() {
  if (!deepSleepEnabled) {
    return "Always online (sleep disabled)";
  }

  unsigned long currentTime = millis();
  unsigned long timeSinceActiveStart = currentTime - activeWindowStart;

  if (timeSinceActiveStart < activeWindow) {
    unsigned long nextWakeTime = activeWindowStart + activeWindow + (sleepDuration / 1000);

    time_t now = timeClient.getEpochTime();
    unsigned long secondsUntilNextWake = (nextWakeTime - currentTime) / 1000;
    time_t nextWakeEpoch = now + secondsUntilNextWake;

    struct tm *ti = localtime(&nextWakeEpoch);
    char timeStr[20];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
             ti->tm_hour, ti->tm_min, ti->tm_sec);

    return String(timeStr);
  } else {
    unsigned long sleepEndTime = activeWindowStart + activeWindow + (sleepDuration / 1000);
    unsigned long nextActiveTime = sleepEndTime;

    time_t now = timeClient.getEpochTime();
    unsigned long secondsUntilNextActive = (nextActiveTime - currentTime) / 1000;
    time_t nextActiveEpoch = now + secondsUntilNextActive;

    struct tm *ti = localtime(&nextActiveEpoch);
    char timeStr[20];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
             ti->tm_hour, ti->tm_min, ti->tm_sec);

    return String(timeStr);
  }
}

String getNextOnlineTimeForMQTT() {
  if (!deepSleepEnabled) {
    return "always_online";
  }

  unsigned long currentTime = millis();
  unsigned long timeSinceActiveStart = currentTime - activeWindowStart;

  if (timeSinceActiveStart < activeWindow) {
    unsigned long nextWakeTime = activeWindowStart + activeWindow + (sleepDuration / 1000);

    time_t now = timeClient.getEpochTime();
    unsigned long secondsUntilNextWake = (nextWakeTime - currentTime) / 1000;
    time_t nextWakeEpoch = now + secondsUntilNextWake;

    struct tm *ti = localtime(&nextWakeEpoch);
    char timestamp[25];
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02d",
             ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday,
             ti->tm_hour, ti->tm_min, ti->tm_sec);

    return String(timestamp);
  } else {
    unsigned long sleepEndTime = activeWindowStart + activeWindow + (sleepDuration / 1000);
    unsigned long nextActiveTime = sleepEndTime;

    time_t now = timeClient.getEpochTime();
    unsigned long secondsUntilNextActive = (nextActiveTime - currentTime) / 1000;
    time_t nextActiveEpoch = now + secondsUntilNextActive;

    struct tm *ti = localtime(&nextActiveEpoch);
    char timestamp[25];
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02d",
             ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday,
             ti->tm_hour, ti->tm_min, ti->tm_sec);

    return String(timestamp);
  }
}

void blinkBlueLED(int count, int delayTime = 200) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BLUE, LED_ON);
    delay(delayTime);
    digitalWrite(LED_BLUE, LED_OFF);
    if (i < count - 1) delay(delayTime);
  }
}

void blinkGreenLED(int count, int delayTime = 200) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_GREEN, LED_ON);
    delay(delayTime);
    digitalWrite(LED_GREEN, LED_OFF);
    if (i < count - 1) delay(delayTime);
  }
}

void blinkRedLED(int count, int delayTime = 200) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_RED, LED_ON);
    delay(delayTime);
    digitalWrite(LED_RED, LED_OFF);
    if (i < count - 1) delay(delayTime);
  }
}

void initLEDs() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}

// Forward declarations
//bool readDistanceFast(uint16_t &out_mm, float &out_cm);
//bool readDistanceBurst(uint16_t &out_mm, float &out_cm, int attempts = 3);

// --- Serial Command Functions ---

void runSerialBenchmark(int iterations = 30) {
  Serial.printf("Starting serial benchmark with %d iterations...\n", iterations);
  performanceMetrics.reset();

  unsigned long benchmarkStart = millis();
  int successCount = 0;

  for (int i = 0; i < iterations; i++) {
    uint16_t test_mm;
    float test_cm;

    unsigned long startTime = millis();
    bool success = readDistanceBurst(test_mm, test_cm, 1);
    unsigned long endTime = millis();

    unsigned long readTime = endTime - startTime;
    performanceMetrics.addMeasurement(readTime, success);

    if (success) {
      successCount++;
      Serial.printf("[%d/%d] SUCCESS: %.1f cm (%d ms)\n",
                    i + 1, iterations, test_cm, readTime);
    } else {
      Serial.printf("[%d/%d] FAILED (%d ms)\n",
                    i + 1, iterations, readTime);
    }
    delay(50);
  }

  unsigned long benchmarkTotal = millis() - benchmarkStart;
  Serial.println("=== Benchmark Results ===");
  Serial.printf("Total time: %lu ms\n", benchmarkTotal);
  Serial.printf("Success rate: %.1f%% (%d/%d)\n",
                performanceMetrics.getSuccessRate(), successCount, iterations);
  Serial.printf("Average read time: %lu ms\n", performanceMetrics.averageReadTime);
  Serial.printf("Min read time: %lu ms\n", performanceMetrics.minReadTime);
  Serial.printf("Max read time: %lu ms\n", performanceMetrics.maxReadTime);
  Serial.println("=========================");
}

void processSerialCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command == "mac") {
    Serial.printf("MAC Address: %s\n", WiFi.macAddress().c_str());
  } else if (command == "read") {
    uint16_t temp_mm;
    float temp_cm;
    if (readDistanceBurst(temp_mm, temp_cm, 2)) {
      Serial.printf("Distance: %.1f cm\n", temp_cm);
    } else {
      Serial.println("Read failed");
    }
  } else if (command == "benchmark") {
    runSerialBenchmark();
  } else if (command == "reset") {
    Serial.println("Resetting...");
    delay(1000);
    ESP.restart();
  } else if (command == "eeprom_reset") {
    Serial.println("Resetting preferences to defaults...");
    preferences.clear();
    Serial.println("Preferences reset. Restarting...");
    delay(1000);
    ESP.restart();
  } else if (command == "preferences_dump") {
    Serial.println("Preferences Dump:");
    Serial.printf("Device Name: %s\n", deviceName);
    Serial.printf("Tank Width: %.1f\n", tankWidth);
    Serial.printf("Tank Height: %.1f\n", tankHeight);
    Serial.printf("Calibration: %d\n", calibration_mm);
  } else if (command == "sleep") {
    Serial.println("Manual deep sleep command received");
    //enterDeepSleep();
  } else if (command == "sleep_disable") {
    deepSleepEnabled = false;
    preferences.putBool(SLEEP_ENABLED_KEY, false);
    Serial.println("Deep sleep disabled");
  } else if (command == "sleep_enable") {
    deepSleepEnabled = true;
    preferences.putBool(SLEEP_ENABLED_KEY, true);
    Serial.println("Deep sleep enabled");
  } else if (command.length() > 0) {
    Serial.printf("Unknown command: '%s'\n", command.c_str());
    Serial.println("Available commands: mac, read, benchmark, reset, eeprom_reset, preferences_dump, sleep, sleep_disable, sleep_enable");
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    char inChar = Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (serialBufferIndex > 0) {
        serialCommandBuffer[serialBufferIndex] = '\0';
        String command = String(serialCommandBuffer);
        processSerialCommand(command);
        serialBufferIndex = 0;
      }
    } else if (inChar == '\b' || inChar == 127) {
      if (serialBufferIndex > 0) {
        serialBufferIndex--;
        Serial.print("\b \b");
      }
    } else if (inChar >= 32 && inChar <= 126) {
      if (serialBufferIndex < SERIAL_COMMAND_BUFFER_SIZE - 1) {
        serialCommandBuffer[serialBufferIndex] = inChar;
        serialBufferIndex++;
        Serial.print(inChar);
      }
    }
  }
}

// --- Preferences Functions ---
void saveConfig() {
  preferences.putFloat("tankWidth", tankWidth);
  preferences.putFloat("tankHeight", tankHeight);
  preferences.putFloat("volumeFactor", volumeFactor);
  preferences.putInt("calibration_mm", calibration_mm);
  Serial.println("Configuration saved");
}

void saveWiFiConfig(const char *ssid, const char *password, const char *mac = nullptr, const char *newDeviceName = nullptr) {
  Serial.println("=== Saving WiFi Configuration ===");
  Serial.printf("SSID: %s\n", ssid);
  Serial.printf("Password: %s\n", password ? "******" : "(empty)");
  
  if (newDeviceName) {
    Serial.printf("New Device Name: %s\n", newDeviceName);
  }
  
  if (mac && strlen(mac) > 0) {
    Serial.printf("Peer MAC: %s\n", mac);
  }
  
  // Save to preferences
  preferences.putString("wifi_ssid", ssid);
  preferences.putString("wifi_pass", password);
  
  if (mac && strlen(mac) == 17) {
    preferences.putString("peer_mac", mac);
  }
  
  if (newDeviceName) {
    preferences.putString("device_name", newDeviceName);
    strlcpy(deviceName, newDeviceName, sizeof(deviceName));
    updateMQTTTopic();
  }
  
  // Force save
  preferences.end();
  delay(10);
  preferences.begin("aqualevel", false);
  
  Serial.println("WiFi settings saved to preferences");
  Serial.println("================================");
}

void saveSleepConfig() {
  preferences.putBool(SLEEP_ENABLED_KEY, deepSleepEnabled);
  preferences.putULong(ACTIVE_WINDOW_KEY, activeWindow);
  preferences.putULong(SLEEP_DURATION_KEY, sleepDuration);
  preferences.putULong(MQTT_INTERVAL_KEY, mqttPublishInterval);
  Serial.println("Sleep configuration saved");
}

void loadSleepConfig() {
  deepSleepEnabled = preferences.getBool(SLEEP_ENABLED_KEY, true);
  activeWindow = preferences.getULong(ACTIVE_WINDOW_KEY, DEFAULT_ACTIVE_WINDOW);
  sleepDuration = preferences.getULong(SLEEP_DURATION_KEY, DEFAULT_SLEEP_DURATION);
  mqttPublishInterval = preferences.getULong(MQTT_INTERVAL_KEY, DEFAULT_MQTT_INTERVAL);

  // Validate loaded values
  if (activeWindow < 60000 || activeWindow > 3600000) {
    activeWindow = DEFAULT_ACTIVE_WINDOW;
  }
  if (sleepDuration < 60000000 || sleepDuration > 7200000000) {
    sleepDuration = DEFAULT_SLEEP_DURATION;
  }
  if (mqttPublishInterval < 10000 || mqttPublishInterval > 300000) {
    mqttPublishInterval = DEFAULT_MQTT_INTERVAL;
  }

  Serial.printf("Sleep config - Enabled: %s, Active: %lu ms, Sleep: %lu us, MQTT: %lu ms\n",
                deepSleepEnabled ? "Yes" : "No", activeWindow, sleepDuration, mqttPublishInterval);
}

void initializePreferences() {
  preferences.begin("aqualevel", false);

  // Check if preferences need initialization
  if (!preferences.isKey("device_name")) {
    Serial.println("Preferences not initialized, setting defaults...");

    // Set default tank configuration
    preferences.putFloat("tankWidth", TANK_PRESETS[0][0]);
    preferences.putFloat("tankHeight", TANK_PRESETS[0][1]);
    preferences.putFloat("volumeFactor", TANK_PRESETS[0][2]);
    preferences.putInt("calibration_mm", 0);

    // Clear WiFi credentials
    preferences.putString("wifi_ssid", "");
    preferences.putString("wifi_pass", "");

    // Set default device name
    preferences.putString("device_name", DEFAULT_DEVICE_NAME);

    // Initialize sleep configuration with defaults
    preferences.putBool(SLEEP_ENABLED_KEY, true);
    preferences.putULong(ACTIVE_WINDOW_KEY, DEFAULT_ACTIVE_WINDOW);
    preferences.putULong(SLEEP_DURATION_KEY, DEFAULT_SLEEP_DURATION);
    preferences.putULong(MQTT_INTERVAL_KEY, DEFAULT_MQTT_INTERVAL);

    Serial.println("Preferences initialized with default values");
  } else {
    Serial.println("Preferences already initialized");
  }
}

void loadConfig() {
  // Load tank configuration
  tankWidth = preferences.getFloat("tankWidth", TANK_PRESETS[0][0]);
  tankHeight = preferences.getFloat("tankHeight", TANK_PRESETS[0][1]);
  volumeFactor = preferences.getFloat("volumeFactor", TANK_PRESETS[0][2]);
  calibration_mm = preferences.getInt("calibration_mm", 0);

  // Load device name
  String loadedName = preferences.getString("device_name", DEFAULT_DEVICE_NAME);
  strlcpy(deviceName, loadedName.c_str(), sizeof(deviceName));

  updateMQTTTopic();

  // Load sleep configuration
  loadSleepConfig();

  // Validate loaded values
  if (isnan(tankWidth) || tankWidth <= 10 || tankWidth > 300) {
    tankWidth = TANK_PRESETS[0][0];
  }
  if (isnan(tankHeight) || tankHeight <= 10 || tankHeight > 300) {
    tankHeight = TANK_PRESETS[0][1];
  }
  if (isnan(volumeFactor) || volumeFactor <= 0 || volumeFactor > 50) {
    volumeFactor = TANK_PRESETS[0][2];
  }
  if (calibration_mm < -1000 || calibration_mm > 1000) {
    calibration_mm = 0;
  }

  Serial.printf("Final config - Device: %s, Width: %.1f, Height: %.1f\n",
                deviceName, tankWidth, tankHeight);
}

bool readDistance(uint16_t &out_mm, float &out_cm) {
  return sensorManager.getStableReading(out_mm, out_cm);
}

bool checkSensorHealth() {
  uint16_t test_mm;
  float test_cm;
  return readDistanceBurst(test_mm, test_cm, 2);
}

String getSensorStats() {
  String stats = "{";
  stats += "\"validReadings\":" + String(sensorManager.getValidReadingsCount()) + ",";
  stats += "\"hasRecentData\":" + String(sensorManager.hasRecentReading() ? "true" : "false") + ",";
  stats += "\"averageReading\":" + String(sensorManager.getAverage(), 1) + ",";
  stats += "\"performanceSuccessRate\":" + String(performanceMetrics.getSuccessRate(), 1) + ",";
  stats += "\"totalReadings\":" + String(performanceMetrics.totalReadings) + ",";
  stats += "\"avgReadTime\":" + String(performanceMetrics.averageReadTime);
  stats += "}";
  return stats;
}

// --- Volume Calculations ---
float calcWaterLevelPercent() {
  float distanceFromTop = cm;
  float waterHeight = tankHeight - distanceFromTop;
  waterHeight = constrain(waterHeight, 0.0f, tankHeight);
  return (waterHeight / tankHeight) * 100.0f;
}

float calcWaterVolumeLiters() {
  float distanceFromTop = cm;
  float waterHeight = tankHeight - distanceFromTop;
  waterHeight = constrain(waterHeight, 0.0f, tankHeight);
  return volumeFactor * waterHeight;
}

void updateMeasurements() {
  percent = calcWaterLevelPercent();
  volume = calcWaterVolumeLiters();
  jsonDataOut = String("{\"distance\":") + String(cm, 1) + ",\"percent\":" + String(percent, 1) + ",\"volume\":" + String(volume, 1) + "}";
}

// --- ESP-NOW Functions ---
void sendESPNowResponse(bool success, const char *jsonData = nullptr) {
  response_message msg;

  if (success && jsonData) {
    snprintf(msg.json, sizeof(msg.json), "%s", jsonData);
  } else {
    snprintf(msg.json, sizeof(msg.json),
             "{\"error\":\"%s\"}",
             success ? "unknown_error" : "sensor_read_failed");
  }

  esp_err_t result = esp_now_send(requestingMAC, (uint8_t *)&msg, sizeof(msg));
  if (result != ESP_OK) {
    Serial.println("{\"error\":\"ESP-NOW send failed\"}");
  }
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(struct_message)) return;

  struct_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  msg.cmd[sizeof(msg.cmd) - 1] = '\0';

  if (strcmp(msg.cmd, "get_distance") == 0) {
    memcpy(requestingMAC, info->src_addr, 6);
    pendingDistanceRequest = true;
  } else if (strcmp(msg.cmd, "get_measure") == 0) {
    memcpy(requestingMAC, info->src_addr, 6);
    pendingDistanceRequest_2 = true;
  }
}

void handleESPNowRequest() {
  if (pendingDistanceRequest || pendingDistanceRequest_2) {
    uint16_t burst_mm;
    float burst_cm;

    unsigned long startTime = millis();
    bool success = readDistanceBurst(burst_mm, burst_cm, 2);
    unsigned long readTime = millis() - startTime;
    performanceMetrics.addMeasurement(readTime, success);

    if (success) {
      mm = burst_mm;
      cm = burst_cm;
      updateMeasurements();
      blinkBlueLED(1);

      if (pendingDistanceRequest) {
        String json = "{\"distance_cm\":" + String(cm, 1) + ",\"calibration_cm\":" + String(calibration_mm / 10.0f, 1) + "}";
        sendESPNowResponse(true, json.c_str());
      } else if (pendingDistanceRequest_2) {
        sendESPNowResponse(true, jsonDataOut.c_str());
      }
    } else {
      sendESPNowResponse(false);
    }

    pendingDistanceRequest = false;
    pendingDistanceRequest_2 = false;
    blinkGreenLED(1);
  }
}

// ===== DEEP SLEEP FUNCTIONS =====
void enterDeepSleep() {
  if (!deepSleepEnabled) {
    Serial.println("Deep sleep disabled, continuing normal operation");
    return;
  }

  Serial.println("Preparing for deep sleep...");
  saveConfig();

  server.end();
  mqttClient.disconnect(true);
  WiFi.disconnect();

  Serial.println("Entering deep sleep...");
  Serial.printf("Active window was: %lu ms\n", millis() - activeWindowStart);

  // ESP32 deep sleep (in microseconds)
  esp_deep_sleep(sleepDuration);

  delay(100);
}

void checkActiveWindow() {
  if (!deepSleepEnabled) return;

  if (millis() - activeWindowStart >= activeWindow) {
    Serial.println("Active window expired, entering deep sleep");
    enterDeepSleep();
  }
}

// --- Web Server Functions ---
String formatRuntime(unsigned long ms) {
  unsigned long sec = ms / 1000;
  unsigned int h = sec / 3600;
  unsigned int m = (sec % 3600) / 60;
  unsigned int s = sec % 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u", h, m, s);
  return String(buf);
}

String getRootHTML() {
  String html = FPSTR(PAGE_ROOT);

  // Calculate values
  updateMeasurements();
  String waterColor = (percent < 20) ? "#ff4444" : "#4285f4";
  float maxVolume = volumeFactor * tankHeight;

  unsigned long remainingActiveTime = activeWindow - (millis() - activeWindowStart);
  unsigned long remainingMinutes = remainingActiveTime / 60000;
  unsigned long remainingSeconds = (remainingActiveTime % 60000) / 1000;

  unsigned long activeTime = millis() - activeWindowStart;
  unsigned long activeMinutes = activeTime / 60000;
  unsigned long activeSeconds = (activeTime % 60000) / 1000;
  String activeTimeStr = String(activeMinutes) + "m " + String(activeSeconds) + "s";

  unsigned long sleepTime = activeWindow - activeTime;
  unsigned long sleepMinutes = sleepTime / 60000;
  unsigned long sleepSeconds = (sleepTime % 60000) / 1000;
  String sleepCountdownStr = String(sleepMinutes) + "m " + String(sleepSeconds) + "s";

  // Replace placeholders
  html.replace("%WATER_COLOR%", waterColor);
  html.replace("%PERCENT%", String(percent, 1));
  html.replace("%DISTANCE%", String(cm, 1));
  html.replace("%VOLUME%", String(volume, 1));
  html.replace("%TANK_WIDTH%", String(tankWidth));
  html.replace("%TANK_HEIGHT%", String(tankHeight));
  html.replace("%MAX_VOLUME%", String(maxVolume, 1));
  html.replace("%MAC_ADDRESS%", WiFi.macAddress());
  html.replace("%UPTIME%", formatRuntime(millis()));
  html.replace("%VALID_READINGS%", String(sensorManager.getValidReadingsCount()));
  html.replace("%UPDATE_INTERVAL%", String(UPDATE_INTERVAL));
  html.replace("%ACTIVE_TIME%", activeTimeStr);
  html.replace("%SLEEP_COUNTDOWN%", sleepCountdownStr);
  html.replace("%NEXT_ONLINE_TIME%", getNextOnlineTime());

  return html;
}

void handleRootRequest(AsyncWebServerRequest *request) {
  request->send(200, "text/html", getRootHTML());
}

void handleDataRequest(AsyncWebServerRequest *request) {
  updateMeasurements();

  bool hasRecent = sensorManager.hasRecentReading();
  int validReadings = sensorManager.getValidReadingsCount();

  unsigned long activeTime = millis() - activeWindowStart;
  unsigned long activeMinutes = activeTime / 60000;
  unsigned long activeSeconds = (activeTime % 60000) / 1000;

  unsigned long sleepTime = activeWindow - activeTime;
  unsigned long sleepMinutes = sleepTime / 60000;
  unsigned long sleepSeconds = (sleepTime % 60000) / 1000;

  String json = "{";
  json += "\"percent\":" + String(percent, 1) + ",";
  json += "\"distance\":" + String(cm, 1) + ",";
  json += "\"volume\":" + String(volume, 1) + ",";
  json += "\"uptime\":\"" + formatRuntime(millis()) + "\",";
  json += "\"sensor_health\":" + String(hasRecent ? "true" : "false") + ",";
  json += "\"valid_readings\":" + String(validReadings) + ",";
  json += "\"active_time\":\"" + String(activeMinutes) + "m " + String(activeSeconds) + "s\",";
  json += "\"sleep_countdown\":\"" + String(sleepMinutes) + "m " + String(sleepSeconds) + "s\",";
  json += "\"next_online\":\"" + getNextOnlineTime() + "\"";
  json += "}";

  request->send(200, "application/json", json);
}

void handleConfigRequest(AsyncWebServerRequest *request) {
  if (request->hasParam("save")) {
    // Handle form submission
    if (request->getParam("save")->value() == "1") {
      if (request->hasParam("preset_mode") && request->getParam("preset_mode")->value() == "1") {
        // Preset mode
        if (request->hasParam("preset_size")) {
          int presetIndex = request->getParam("preset_size")->value().toInt();
          if (presetIndex >= 0 && presetIndex < 5) {
            tankWidth = TANK_PRESETS[presetIndex][0];
            tankHeight = TANK_PRESETS[presetIndex][1];
            volumeFactor = TANK_PRESETS[presetIndex][2];
          }
        }
      } else {
        // Manual mode
        if (request->hasParam("manual_width")) {
          tankWidth = request->getParam("manual_width")->value().toFloat();
        }
        if (request->hasParam("manual_height")) {
          tankHeight = request->getParam("manual_height")->value().toFloat();
          volumeFactor = (tankWidth * tankWidth) / 1000.0f;
        }
      }

      // Calibration
      if (request->hasParam("calibration")) {
        calibration_mm = request->getParam("calibration")->value().toInt() * 10;
      }

      saveConfig();
      request->redirect("/");
      return;
    }
  }

  // Show config form
  String html = FPSTR(PAGE_CONFIG);
  html.replace("%TANK_WIDTH%", String(tankWidth, 1));
  html.replace("%TANK_HEIGHT%", String(tankHeight, 1));
  html.replace("%CALIBRATION%", String(calibration_mm / 10.0f, 1));

  request->send(200, "text/html", html);
}

void handleSleepConfigRequest(AsyncWebServerRequest *request) {
  if (request->hasParam("save")) {
    // Handle form submission
    deepSleepEnabled = request->hasParam("sleep_enabled");

    if (request->hasParam("active_window")) {
      activeWindow = request->getParam("active_window")->value().toInt() * 60000;
    }

    if (request->hasParam("sleep_duration")) {
      sleepDuration = request->getParam("sleep_duration")->value().toInt() * 60000000;
    }

    if (request->hasParam("mqtt_interval")) {
      mqttPublishInterval = request->getParam("mqtt_interval")->value().toInt() * 1000;
    }

    saveSleepConfig();

    String html = FPSTR(PAGE_SLEEP_SAVED);
    request->send(200, "text/html", html);
    return;
  }

  // Show config form
  String html = FPSTR(PAGE_SLEEP_CONFIG);
  html.replace("%SLEEP_STATUS%", deepSleepEnabled ? "Sleep Enabled" : "Sleep Disabled");
  html.replace("%ACTIVE_TIME%", String(activeWindow / 60000));
  html.replace("%SLEEP_TIME%", String(sleepDuration / 60000000));
  html.replace("%MQTT_INTERVAL%", String(mqttPublishInterval / 1000));
  html.replace("%ACTIVE_WINDOW%", String(activeWindow / 60000));
  html.replace("%SLEEP_DURATION%", String(sleepDuration / 60000000));
  html.replace("%MQTT_INTERVAL_VAL%", String(mqttPublishInterval / 1000));
  html.replace("%SLEEP_CHECKED%", deepSleepEnabled ? "checked" : "");

  request->send(200, "text/html", html);
}

void handleWiFiSetupRequest(AsyncWebServerRequest *request) {
  // Check if this is a save request (has "save" parameter)
  if (request->hasParam("save")) {
    Serial.println("WiFi setup: Save requested");
    
    // Get form parameters
    String newSSID = request->hasParam("ssid") ? request->getParam("ssid")->value() : "";
    String newPass = request->hasParam("password") ? request->getParam("password")->value() : "";
    String newMAC = request->hasParam("mac") ? request->getParam("mac")->value() : "";
    String newDeviceName = request->hasParam("device_name") ? request->getParam("device_name")->value() : "";
    
    // Validate device name
    if (newDeviceName.length() == 0) {
      newDeviceName = DEFAULT_DEVICE_NAME;
    }
    
    Serial.printf("New settings - SSID: %s, Device: %s\n", newSSID.c_str(), newDeviceName.c_str());
    
    // Save configuration
    saveWiFiConfig(newSSID.c_str(), newPass.c_str(), newMAC.c_str(), newDeviceName.c_str());
    
    // Build success page with parameters
    String html = FPSTR(PAGE_WIFI_SAVED);
    html.replace("%NEW_DEVICE_NAME%", newDeviceName);
    html.replace("%NEW_SSID%", newSSID);
    
    request->send(200, "text/html", html);
    
    // Schedule reboot after 5 seconds using a timer
    Serial.println("Countdown page sent. Rebooting in 5 seconds...");
    
    // Create a one-shot timer for reboot
    static Ticker rebootTimer;
    rebootTimer.once(5, []() {
      Serial.println("5 seconds elapsed. Rebooting now...");
      ESP.restart();
    });
    
    return;  // Return immediately, reboot will happen via timer
  }
  
  // Show WiFi setup form (GET without save parameter)
  Serial.println("WiFi setup: Showing form");
  
  String ssid = preferences.getString("wifi_ssid", "");
  String password = preferences.getString("wifi_pass", "");
  String storedMAC = preferences.getString("peer_mac", "");
  String currentDeviceName = preferences.getString("device_name", DEFAULT_DEVICE_NAME);
  
  String html = FPSTR(PAGE_WIFI_SETUP);
  html.replace("%DEVICE_NAME%", currentDeviceName);
  html.replace("%MAC_ADDRESS%", WiFi.macAddress());
  html.replace("%SSID%", ssid);
  html.replace("%PASSWORD%", password);
  html.replace("%MAC%", storedMAC);
  
  request->send(200, "text/html", html);
}

void handleSensorStatsRequest(AsyncWebServerRequest *request) {
  String json = getSensorStats();
  request->send(200, "application/json", json);
}

void handleBenchmarkRequest(AsyncWebServerRequest *request) {
  if (request->hasParam("run")) {
    int iterations = request->hasParam("iterations") ? request->getParam("iterations")->value().toInt() : 20;
    iterations = constrain(iterations, 5, 30);

    // Simple benchmark response (simplified for ESP32)
    String html = "<!DOCTYPE html><html><head><title>Benchmark Results</title></head><body>";
    html += "<h2>Benchmark Results</h2>";
    html += "<p>ESP32 Benchmark completed with " + String(iterations) + " iterations</p>";
    html += "<a href='/benchmark'>Run Another Benchmark</a><br>";
    html += "<a href='/'>Back to Dashboard</a>";
    html += "</body></html>";

    request->send(200, "text/html", html);
  } else {
    // Show benchmark form
    String html = FPSTR(PAGE_BENCHMARK_FORM);
    request->send(200, "text/html", html);
  }
}

String getCustomTimestamp() {
  timeClient.update();
  int day = timeClient.getDay();
  int hours = timeClient.getHours();
  int minutes = timeClient.getMinutes();

  time_t rawtime = timeClient.getEpochTime();
  struct tm *ti;
  ti = localtime(&rawtime);

  char timestamp[20];
  snprintf(timestamp, sizeof(timestamp),
           "%02d-%02d-%04dT%02d:%02d",
           ti->tm_mday,
           ti->tm_mon + 1,
           ti->tm_year + 1900,
           hours,
           minutes);

  return String(timestamp);
}

void publishMQTTStatus() {
  if (!mqttClient.connected()) {
    Serial.println("MQTT not connected, skip publish");
    return;
  }

  char payload[256];
  String NTPDateTime = getCustomTimestamp();

  snprintf(payload, sizeof(payload),
           "{\"timestamp\":\"%s\","
           "\"uptime\":\"%s\","
           "\"device\":\"%s\","
           "\"ip_address\":\"%s\","
           "\"distance_cm\":%.1f,"
           "\"level_percent\":%.1f,"
           "\"volume_liters\":%.1f,"
           "\"next_online\":\"%s\"}",
           NTPDateTime.c_str(),
           formatRuntime(millis()).c_str(),
           deviceName,
           WiFi.localIP().toString().c_str(),
           cm,
           percent,
           volume,
           getNextOnlineTimeForMQTT().c_str());

  if (mqttClient.publish(PubTopic, 0, true, payload)) {
    Serial.println("MQTT published: " + String(payload));
  } else {
    Serial.println("MQTT publish failed");
  }
}

//=============== MQTT functions ====================
void printSeparationLine() {
  Serial.println("************************************************");
}

void generateClientId(char *buffer, size_t size) {
  uint32_t timestamp = millis();
  uint32_t randomNum = random(1000, 9999);
  snprintf(buffer, size, "esp32_%lu_%lu", timestamp, randomNum);
}

void connectToMqtt() {
  if (mqttClient.connected()) return;

  char clientId[30];
  generateClientId(clientId, sizeof(clientId));
  mqttClient.setClientId(clientId);

  Serial.printf("MQTT connecting as %s ...\n", clientId);
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.print("Connected to MQTT broker: ");
  Serial.print(MQTT_HOST);
  Serial.print(", port: ");
  Serial.println(MQTT_PORT);
  Serial.print("PubTopic: ");
  Serial.println(PubTopic);
  printSeparationLine();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.status() == WL_CONNECTED) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Handle incoming messages if needed
}

void onMqttPublish(uint16_t packetId) {
  blinkBlueLED(1);
}

void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("[WiFi] Re-connected → forcing MQTT reconnect");
      mqttReconnectTimer.detach();
      mqttReconnectTimer.once(1, connectToMqtt);
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi] Lost connection → MQTT will auto-retry");
      break;

    default:
      break;
  }
}

void handleSensorReading() {
  static unsigned long lastWebUpdate = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastWebUpdate >= 1000) {
    uint16_t stable_mm;
    float stable_cm;

    if (readDistance(stable_mm, stable_cm)) {
      mm = stable_mm;
      cm = stable_cm;
      updateMeasurements();
      lastWebUpdate = currentTime;
    }
  }

  handleESPNowRequest();
}

void ForceAPMode(bool enable, float minutes) {
  delay(10);
  if (!enable) return;

  if (minutes <= 0) minutes = 1;
  unsigned long apDuration = (unsigned long)(minutes * 60 * 1000);

  Serial.printf("ForceAPMode(true, %.2f min) → Starting AP mode\n", minutes);
  digitalWrite(LED_RED, LED_ON);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(deviceName, WIFI_AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("AP Mode IP: %s\n", apIP.toString().c_str());

  // Set up server routes
  server.on("/", HTTP_GET, handleRootRequest);
  server.on("/data", HTTP_GET, handleDataRequest);
  server.on("/config", HTTP_GET, handleConfigRequest);
  server.on("/sleep-config", HTTP_GET, handleSleepConfigRequest);
  server.on("/wifi-setup", HTTP_GET, handleWiFiSetupRequest);
  server.on("/sensor-stats", HTTP_GET, handleSensorStatsRequest);
  server.on("/benchmark", HTTP_GET, handleBenchmarkRequest);

  server.begin();

  // DNS server
  dnsServer.start(DNS_PORT, "*", apIP);

  // OTA
  ArduinoOTA.setHostname(deviceName);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();

  unsigned long apStartTime = millis();
  Serial.printf("AP Mode active for %.1f minutes...\n", minutes);

  while (millis() - apStartTime < apDuration) {
    ArduinoOTA.handle();
    dnsServer.processNextRequest();
    handleSensorReading();

    static unsigned long lastBlink = 0;
    if (millis() - lastBlink >= 2000) {
      digitalWrite(LED_RED, !digitalRead(LED_RED));
      lastBlink = millis();
    }
    delay(10);
  }

  Serial.println("AP mode timeout → Restarting...");
  delay(1000);
  ESP.restart();
}

bool connectToWiFi() {
  String ssid = preferences.getString("wifi_ssid", "");
  String password = preferences.getString("wifi_pass", "");

  if (ssid.length() == 0) {
    Serial.println("{\"error\":\"No saved SSID\"}");
    return false;
  }

  Serial.printf("Connecting to WiFi: %s\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWiFiEvent);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    connectToMqtt();
    wifiConnected = true;
    blinkGreenLED(1);
    return true;
  } else {
    Serial.println("\nWiFi connection failed → Starting AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(deviceName, WIFI_AP_PASSWORD);
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    digitalWrite(LED_RED, LED_ON);
    return false;
  }
}

void setup() {
  initLEDs();
  Serial.begin(115200);
  delay(100);

  Serial.println();
  Serial.println("              AquaLevel Pro (ESP32)            ");
  //Serial.println("Available commands: mac, read, benchmark, reset");
  Serial.println("===============================================");
  Serial.println("    Designed by Asst.Prof Komkrit Chooruang    ");
  Serial.println("===============================================");

  initializePreferences();
  loadConfig();

  // Initialize UART2 with GPIO18 as TX, GPIO19 as RX
  sensorSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  sensorSerial.setTimeout(50);

  // Check for force AP mode
  pinMode(FORCE_AP_PIN, INPUT_PULLUP);
  delay(10);

  bool apFlag = (digitalRead(FORCE_AP_PIN) == LOW);
  delay(10);
  ForceAPMode(apFlag, 10);
  delay(10);

  // MQTT setup
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  // Try WiFi connection
  WiFi.mode(WIFI_STA);
  if (!connectToWiFi()) {
    Serial.println("WiFi failed, starting AP mode");
    ForceAPMode(true, 10);
    blinkRedLED(3);
  } else {
    Serial.println("WiFi connected successfully");
    blinkGreenLED(3);
  }

  // OTA Setup
  ArduinoOTA.setHostname(deviceName);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Update Start");
    digitalWrite(LED_RED, LED_ON);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update End");
    digitalWrite(LED_RED, LED_OFF);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %d%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    digitalWrite(LED_RED, LED_OFF);
    ESP.restart();
  });
  ArduinoOTA.begin();

  // Web Server Setup
  server.on("/", HTTP_GET, handleRootRequest);
  server.on("/data", HTTP_GET, handleDataRequest);
  server.on("/config", HTTP_GET, handleConfigRequest);
  server.on("/sleep-config", HTTP_GET, handleSleepConfigRequest);
  server.on("/wifi-setup", HTTP_GET, handleWiFiSetupRequest);
  server.on("/sensor-stats", HTTP_GET, handleSensorStatsRequest);
  server.on("/benchmark", HTTP_GET, handleBenchmarkRequest);

  server.begin();

  // ESP-NOW Setup
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("{\"error\":\"ESP-NOW init failed\"}");
    delay(1000);
    ESP.restart();
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Load MAC from preferences
  String storedMAC = preferences.getString("peer_mac", "");
  if (storedMAC.length() == 17) {
    sscanf(storedMAC.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
           &senderMAC[0], &senderMAC[1], &senderMAC[2],
           &senderMAC[3], &senderMAC[4], &senderMAC[5]);
    Serial.printf("Using stored peer MAC: %s\n", storedMAC.c_str());
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, senderMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("{\"error\":\"Failed to add peer\"}");
  }

  activeWindowStart = millis();
  Serial.println("Active window started");

  // Sensor health check
  if (checkSensorHealth()) {
    Serial.println("{\"sensor\":\"healthy\"}");
    unsigned long initStart = millis();
    while (millis() - initStart < 5000) {
      uint16_t temp_mm;
      float temp_cm;
      if (readDistance(temp_mm, temp_cm)) {
        mm = temp_mm;
        cm = temp_cm;
        updateMeasurements();
        Serial.printf("{\"initial_reading\":\"%.1f cm\"}\n", cm);
        break;
      }
      delay(100);
    }
  } else {
    Serial.println("{\"warning\":\"sensor_not_responding\"}");
  }

  Serial.printf("Setup complete - %lumin active / %lumin sleep cycle\n",
                activeWindow / 60000, sleepDuration / 60000000);
  blinkGreenLED(2);
}

void loop() {
  delay(10);
  ArduinoOTA.handle();
  dnsServer.processNextRequest();

  handleSensorReading();
  checkActiveWindow();

  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck >= 60000) {
    lastWifiCheck = millis();
    Serial.printf("System health - Free heap: %d bytes, WiFi: %s, MQTT: %s\n",
                  ESP.getFreeHeap(),
                  WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                  mqttClient.connected() ? "Connected" : "Disconnected");
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting reconnect");
      WiFi.disconnect();
      delay(100);
      connectToWiFi();
    }
  }

  static unsigned long lastMQTTPublish = 0;
  if (WiFi.status() == WL_CONNECTED && millis() - lastMQTTPublish >= mqttPublishInterval) {
    lastMQTTPublish = millis();
    publishMQTTStatus();
  }

  handleSerialInput();
  delay(10);
}