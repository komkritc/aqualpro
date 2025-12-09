/*
 * AquaLevel Pro
 * Designed by: Komkrit Chooraung
 * Date: 28-Nov-2025
 * Version: 3.1
 * Added: Deep sleep with 15min active window + 20min sleep
 * Added: Battery monitoring via ADC with 47K/10K voltage divider
 */

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include <Ticker.h>
#include <AsyncMqtt_Generic.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600);  // UTC+7 (Bangkok time)

// --- Deep Sleep Configuration ---
#define DEFAULT_ACTIVE_WINDOW 900000       // 15 minutes active (900000 ms = 15 * 60 * 1000)
#define DEFAULT_SLEEP_DURATION 1200000000  // 20 minutes deep sleep (1200000000 microseconds = 20 * 60 * 1000 * 1000)
#define DEFAULT_MQTT_INTERVAL 60000        // 1 minute MQTT publish interval
#define FORCE_WAKE_PIN D0                  // GPIO16 for wakeup from deep sleep

// --- Battery Monitoring (ADC with 47K/10K voltage divider) ---
#define BATTERY_PIN A0                   // A0 for ADC battery reading on ESP8266
#define ADC_RESOLUTION 1024.0f           // ESP8266 has 10-bit ADC (0-1023)
#define ADC_REFERENCE_VOLTAGE 3.3f       // ESP8266 ADC reference is actually ~1.0V internally
#define VOLTAGE_DIVIDER_RATIO 5.7f       // (47000 + 10000) / 10000 = 5.7
#define BATTERY_CALIBRATION_FACTOR 1.0f  // Calibration factor for accuracy
#define BATTERY_READ_INTERVAL 30000      // Check battery every 30 seconds

bool hasBatteryMonitoring = true;
float batteryVoltage = 0.0f;
float batteryPercentage = 0.0f;
bool lowBatteryMode = false;
#define BATTERY_CRITICAL_THRESHOLD 3.0f  // 3.0V - Critical, go to deep sleep
#define BATTERY_WARNING_THRESHOLD 3.3f   // 3.3V - Warning level
#define BATTERY_FULL_THRESHOLD 4.2f      // 4.2V - Full (Li-ion battery)
#define BATTERY_EMPTY_THRESHOLD 3.0f     // 3.0V - Empty

// EEPROM addresses for sleep configuration
#define ADDR_SLEEP_ENABLED 120   // bool (1 byte)
#define ADDR_ACTIVE_WINDOW 121   // unsigned long (4 bytes)
#define ADDR_SLEEP_DURATION 125  // unsigned long (4 bytes)
#define ADDR_MQTT_INTERVAL 129   // unsigned long (4 bytes)

bool deepSleepEnabled = true;  // Set to false to disable deep sleep for debugging
unsigned long activeWindowStart = 0;
bool isActiveWindow = true;
unsigned long activeWindow = DEFAULT_ACTIVE_WINDOW;
unsigned long sleepDuration = DEFAULT_SLEEP_DURATION;
unsigned long mqttPublishInterval = DEFAULT_MQTT_INTERVAL;

// --- Constants ---
#define EEPROM_SIZE 256      // Increased from 128 to accommodate device name and sleep config
#define ADDR_WIDTH 0         // float (4 bytes)
#define ADDR_HEIGHT 4        // float (4 bytes)
#define ADDR_VOL_FACTOR 8    // float (4 bytes)
#define ADDR_CALIB 12        // int (4 bytes)
#define ADDR_WIFI_SSID 16    // 32 bytes
#define ADDR_WIFI_PASS 48    // 32 bytes
#define ADDR_WIFI_MAC 80     // 6 bytes for MAC address
#define ADDR_DEVICE_NAME 86  // 32 bytes for device name

// Default device name (can be changed via web interface)
#define DEFAULT_DEVICE_NAME "TankMonitor144"
#define WIFI_AP_PASSWORD "12345678"
#define OTA_PASSWORD "12345678"
#define MQTT_HOST "test.mosquitto.org"
#define MQTT_PORT 1883

// Global variable for device name (loaded from EEPROM)
char deviceName[32] = DEFAULT_DEVICE_NAME;
const char *PubTopic;  // Will be set dynamically

// Update the MQTT topic to use dynamic device name
void updateMQTTTopic() {
  static char topicBuffer[64];
  snprintf(topicBuffer, sizeof(topicBuffer), "%s/status", deviceName);
  PubTopic = topicBuffer;
}

#define LED_BLUE D4   // Status/Activity LED
#define LED_RED D7    // Error/AP Mode LED
#define LED_GREEN D8  // WiFi Connected LED
#define LED_ON HIGH
#define LED_OFF LOW

#define RX_PIN D1
#define TX_PIN D2
#define SENSOR_READ_TIMEOUT 100        // ms
#define SERIAL_INPUT_TIMEOUT 5000      // ms
#define UPDATE_INTERVAL 2000           // ms for web dashboard updates
#define SERIAL_COMMAND_BUFFER_SIZE 32  // Buffer size for serial commands

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

const byte DNS_PORT = 53;  // DNS server port for captive portal
bool wifiConnected = false;

// Tank presets {Width, Height, VolumeFactor}
const float TANK_PRESETS[][3] = {
  { 68.0, 162.0, 3.086f },   // 500L  (68cm W × 45.4cm L × 162cm H)
  { 79.5, 170.0, 4.118f },   // 700L  (79.5cm W × 51.8cm L × 170cm H)
  { 92.5, 181.0, 5.513f },   // 1000L (92.5cm W × 59.6cm L × 181cm H)
  { 109.0, 196.0, 7.663f },  // 1500L (109cm W × 70.3cm L × 196cm H)
  { 123.0, 205.0, 9.742f }   // 2000L (123cm W × 79.2cm L × 205cm H)
};

// --- Global Variables ---
float tankWidth = TANK_PRESETS[0][0];  // Default to 500L tank
float tankHeight = TANK_PRESETS[0][1];
float volumeFactor = TANK_PRESETS[0][2];
int calibration_mm = 0;
float percent = 0.0f;
float volume = 0.0f;

// Sensor data
uint16_t mm = 0;
float cm = 0.0f;

// ===== FORWARD DECLARATIONS =====
void initBatteryADC();
float readBatteryVoltage();
void checkBattery();
String getBatteryStatusString();
// =================================

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
ESP8266WebServer server(80);
DNSServer dnsServer;
SoftwareSerial sensorSerial(RX_PIN, TX_PIN);

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

      // Simple moving average
      averageReadTime = (averageReadTime * (successfulReadings - 1) + readTime) / successfulReadings;
    }
  }

  float getSuccessRate() {
    if (totalReadings == 0) return 0.0f;
    return (float)successfulReadings / totalReadings * 100.0f;
  }
};

PerformanceMetrics performanceMetrics;

// ===== HTML PAGES IN PROGMEM =====

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
    .battery-info {
      background: #e3f2fd;
      border: 1px solid #bbdefb;
      border-radius: 4px;
      padding: 10px;
      margin: 10px 0;
      text-align: center;
      font-size: 14px;
    }
    .battery-level {
      height: 20px;
      background: linear-gradient(90deg, #f44336 0%, #ff9800 30%, #4CAF50 100%);
      border-radius: 3px;
      margin: 5px 0;
      overflow: hidden;
    }
    .battery-fill {
      height: 100%;
      background: #2196F3;
      transition: width 0.5s ease;
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
    .battery-status {
      display: inline-block;
      padding: 2px 8px;
      border-radius: 12px;
      font-size: 12px;
      font-weight: 500;
      margin-left: 5px;
    }
    .battery-good { background: #e8f5e8; color: #2e7d32; }
    .battery-warning { background: #fff3e0; color: #f57c00; }
    .battery-critical { background: #ffebee; color: #c62828; }
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
    
    <div class="battery-info" id="batteryInfo">
      Battery: <span id="batteryVoltage">%BATTERY_VOLTAGE%</span>V 
      <span class="battery-status" id="batteryStatus">%BATTERY_STATUS%</span>
      <div class="battery-level">
        <div class="battery-fill" id="batteryFill" style="width: %BATTERY_PERCENTAGE%%"></div>
      </div>
      <div>Charge: <span id="batteryPercentage">%BATTERY_PERCENTAGE%</span>%</div>
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
        <p class="footer-meta-item">Battery: <span id="batteryValue">%BATTERY_VOLTAGE%V (%BATTERY_PERCENTAGE%%)</span></p>
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
            document.getElementById('batteryVoltage').textContent = data.battery_voltage ? data.battery_voltage.toFixed(2) : 'N/A';
            document.getElementById('batteryPercentage').textContent = data.battery_percentage ? data.battery_percentage.toFixed(0) : 'N/A';
            
            // Update battery fill
            const batteryFill = document.getElementById('batteryFill');
            if (data.battery_percentage) {
              batteryFill.style.width = data.battery_percentage.toFixed(0) + '%';
            }
            
            // Update battery status indicator
            const batteryStatus = document.getElementById('batteryStatus');
            if (data.battery_voltage >= 3.7) {
              batteryStatus.className = 'battery-status battery-good';
              batteryStatus.textContent = 'Good';
            } else if (data.battery_voltage >= 3.3) {
              batteryStatus.className = 'battery-status battery-warning';
              batteryStatus.textContent = 'Low';
            } else {
              batteryStatus.className = 'battery-status battery-critical';
              batteryStatus.textContent = 'Critical';
            }
            
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
    .battery-info {
      background: #e3f2fd;
      border: 1px solid #bbdefb;
      border-radius: 4px;
      padding: 10px;
      margin-bottom: 20px;
      text-align: center;
      font-size: 14px;
    }
  </style>
</head>
<body>
  <div class="config-form">
    <h2>Tank Configuration</h2>
    
    <div class="battery-info">
      <div>Battery: %BATTERY_VOLTAGE%V (%BATTERY_PERCENTAGE%%)</div>
      <div>Status: %BATTERY_STATUS%</div>
    </div>
    
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
    .battery-info {
      background: #e3f2fd;
      border: 1px solid #bbdefb;
      border-radius: 4px;
      padding: 10px;
      margin: 10px 0;
      text-align: center;
      font-size: 14px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>Sleep Configuration</h2>
    
    <div class="battery-info">
      <div><strong>Battery:</strong> %BATTERY_VOLTAGE%V (%BATTERY_PERCENTAGE%%) - %BATTERY_STATUS%</div>
      <div><strong>Note:</strong> Device will enter extended deep sleep if battery falls below 3.0V</div>
    </div>
    
    <div class="current-info">
      <div class="info-item"><strong>Current Status:</strong> %SLEEP_STATUS%</div>
      <div class="info-item"><strong>Active Time:</strong> %ACTIVE_TIME% minutes</div>
      <div class="info-item"><strong>Sleep Time:</strong> %SLEEP_TIME% minutes</div>
      <div class="info-item"><strong>MQTT Interval:</strong> %MQTT_INTERVAL% seconds</div>
    </div>
    
    <form method="post" action="/sleep-config">
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
    .battery-info {
      background: #e3f2fd;
      border: 1px solid #bbdefb;
      border-radius: 4px;
      padding: 10px;
      margin: 15px 0;
      font-size: 14px;
    }
    .back-link {
      display: inline-block;
      margin-top: 1rem;
      color: var(--primary);
      text-decoration: none;
      font-weight: 500;
      margin: 0 10px;
    }
    .back-link:hover {
      text-decoration: underline;
    }
    .links {
      margin-top: 1.5rem;
    }
  </style>
</head>
<body>
  <div class="card">
    <div class="success-icon">✓</div>
    <h1>Sleep Configuration Saved</h1>
    <p>Sleep settings have been updated successfully.</p>
    
    <div class="battery-info">
      <p><strong>Current Battery:</strong> %BATTERY_VOLTAGE%V (%BATTERY_PERCENTAGE%%)</p>
    </div>
    
    <div class="links">
      <a href="/sleep-config" class="back-link">← Back to Sleep Config</a>
      <a href="/" class="back-link">← Back to Dashboard</a>
    </div>
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
    .battery-info {
      background: #e3f2fd;
      border: 1px solid #bbdefb;
      border-radius: 4px;
      padding: 10px;
      margin: 10px 0;
      text-align: center;
      font-size: 14px;
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
    
    <div class="battery-info">
      <div><strong>Battery:</strong> %BATTERY_VOLTAGE%V (%BATTERY_PERCENTAGE%%)</div>
      <div><strong>Status:</strong> %BATTERY_STATUS%</div>
    </div>
    
    <form method="post" action="/wifi-setup">
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
    }
    .countdown {
      font-size: 1.2rem;
      font-weight: bold;
      color: var(--primary);
      margin-top: 1rem;
    }
    .progress-bar {
      height: 4px;
      background: #e0e0e0;
      border-radius: 2px;
      margin-top: 1rem;
      overflow: hidden;
    }
    .progress {
      height: 100%;
      background: var(--primary);
      width: 100%;
      animation: progress 5s linear forwards;
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
    <h1>Settings Saved Successfully</h1>
    <p>The device will reboot to apply changes.</p>
    <div class="countdown" id="countdown">Rebooting in 5 seconds...</div>
    <div class="progress-bar">
      <div class="progress"></div>
    </div>
  </div>
  <script>
    let seconds = 5;
    const countdownEl = document.getElementById('countdown');
    
    const timer = setInterval(() => {
      seconds--;
      countdownEl.textContent = `Rebooting in ${seconds} second${seconds !== 1 ? 's' : ''}...`;
      
      if (seconds <= 0) {
        clearInterval(timer);
      }
    }, 1000);
    
    // Redirect after 5 seconds
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


// ===== NEXT ONLINE TIME CALCULATION =====
String getNextOnlineTime() {
  if (!deepSleepEnabled) {
    return "Always online (sleep disabled)";
  }

  unsigned long currentTime = millis();
  unsigned long timeSinceActiveStart = currentTime - activeWindowStart;

  if (timeSinceActiveStart < activeWindow) {
    // Still in active window
    unsigned long nextWakeTime = activeWindowStart + activeWindow + (sleepDuration / 1000);

    // Get current time
    time_t now;
    if (timeClient.isTimeSet()) {
      now = timeClient.getEpochTime();
    } else {
      // Fallback to rough estimate based on uptime
      now = 1700000000 + (currentTime / 1000);
    }

    unsigned long secondsUntilNextWake = (nextWakeTime - currentTime) / 1000;
    time_t nextWakeEpoch = now + secondsUntilNextWake;

    struct tm *ti = localtime(&nextWakeEpoch);
    char timeStr[20];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
             ti->tm_hour, ti->tm_min, ti->tm_sec);

    return String(timeStr);
  } else {
    // Should be sleeping now
    unsigned long sleepEndTime = activeWindowStart + activeWindow + (sleepDuration / 1000);
    unsigned long nextActiveTime = sleepEndTime;

    // Get current time
    time_t now;
    if (timeClient.isTimeSet()) {
      now = timeClient.getEpochTime();
    } else {
      // Fallback to rough estimate based on uptime
      now = 1700000000 + (currentTime / 1000);
    }

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

  // Calculate when the device will be online next
  unsigned long currentTime = millis();
  unsigned long timeSinceActiveStart = currentTime - activeWindowStart;

  if (timeSinceActiveStart < activeWindow) {
    // Still in active window
    unsigned long nextWakeTime = activeWindowStart + activeWindow + (sleepDuration / 1000);

    // Get current epoch time
    time_t now;
    if (timeClient.isTimeSet()) {
      now = timeClient.getEpochTime();
    } else {
      // Fallback to rough estimate based on uptime
      now = 1700000000 + (currentTime / 1000);  // Base timestamp + uptime
    }

    unsigned long secondsUntilNextWake = (nextWakeTime - currentTime) / 1000;
    time_t nextWakeEpoch = now + secondsUntilNextWake;

    struct tm *ti = localtime(&nextWakeEpoch);
    char timestamp[30];
    snprintf(timestamp, sizeof(timestamp), "%02d-%02d-%04dT%02d:%02d",
             ti->tm_mday,
             ti->tm_mon + 1,
             ti->tm_year + 1900,
             ti->tm_hour,
             ti->tm_min);

    return String(timestamp);
  } else {
    // Should be sleeping now, calculate next active time
    unsigned long sleepEndTime = activeWindowStart + activeWindow + (sleepDuration / 1000);
    unsigned long nextActiveTime = sleepEndTime;

    // Get current epoch time
    time_t now;
    if (timeClient.isTimeSet()) {
      now = timeClient.getEpochTime();
    } else {
      // Fallback to rough estimate based on uptime
      now = 1700000000 + (currentTime / 1000);
    }

    unsigned long secondsUntilNextActive = (nextActiveTime - currentTime) / 1000;
    time_t nextActiveEpoch = now + secondsUntilNextActive;

    struct tm *ti = localtime(&nextActiveEpoch);
    char timestamp[30];
    snprintf(timestamp, sizeof(timestamp), "%02d-%02d-%04dT%02d:%02d",
             ti->tm_mday,
             ti->tm_mon + 1,
             ti->tm_year + 1900,
             ti->tm_hour,
             ti->tm_min);

    return String(timestamp);
  }
}

// ===== UTILITY FUNCTIONS =====
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
  pinMode(FORCE_WAKE_PIN, WAKEUP_PULLUP);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}

// --- Buffered Reading Manager ---
class SensorReadingManager {
private:
  static const int BUFFER_SIZE = 5;
  float readings[BUFFER_SIZE];
  int bufferIndex;
  int validReadings;
  unsigned long lastSuccessfulRead;
  bool initialized;

  // Statistics for error detection
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

  bool getStableReading(uint16_t &out_mm, float &out_cm);
  float getAverage();
  bool hasRecentReading();
  void reset();
  int getValidReadingsCount();
};

// Global sensor manager instance
SensorReadingManager sensorManager;

// --- Serial Command Functions ---

// High-speed burst reading for ESP-NOW requests
bool readDistanceBurst(uint16_t &out_mm, float &out_cm, int attempts = 3) {
  for (int i = 0; i < attempts; i++) {
    unsigned long startTime = millis();

    // Quick trigger and read cycle
    sensorSerial.flush();
    sensorSerial.write(0x55);

    // Wait for response with shorter timeout
    unsigned long timeoutStart = millis();
    while (millis() - timeoutStart < 60) {  // 60ms timeout
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
      delay(1);  // Small delay to prevent tight polling
    }

    // Small delay between attempts
    if (i < attempts - 1) {
      delay(20);
    }
  }

  return false;
}

// Serial benchmarking function (different from web benchmark)
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

    delay(50);  // Small delay between tests
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

// --- EEPROM Functions ---
void saveConfig() {
  EEPROM.put(ADDR_WIDTH, tankWidth);
  EEPROM.put(ADDR_HEIGHT, tankHeight);
  EEPROM.put(ADDR_VOL_FACTOR, volumeFactor);
  EEPROM.put(ADDR_CALIB, calibration_mm);
  if (!EEPROM.commit()) {
    Serial.println("{\"error\":\"EEPROM commit failed\"}");
  }
}

void saveWiFiConfig(const char *ssid, const char *password, const char *mac = nullptr, const char *newDeviceName = nullptr) {
  char ssidBuf[32] = { 0 };
  char passBuf[32] = { 0 };
  char nameBuf[32] = { 0 };

  strlcpy(ssidBuf, ssid, sizeof(ssidBuf));
  strlcpy(passBuf, password, sizeof(passBuf));

  EEPROM.put(ADDR_WIFI_SSID, ssidBuf);
  EEPROM.put(ADDR_WIFI_PASS, passBuf);

  // Save MAC if provided
  if (mac && strlen(mac) == 17) {
    uint8_t macBytes[6];
    sscanf(mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
           &macBytes[0], &macBytes[1], &macBytes[2],
           &macBytes[3], &macBytes[4], &macBytes[5]);
    EEPROM.put(ADDR_WIFI_MAC, macBytes);
  }

  // Save device name if provided
  if (newDeviceName) {
    strlcpy(nameBuf, newDeviceName, sizeof(nameBuf));
    EEPROM.put(ADDR_DEVICE_NAME, nameBuf);
    strlcpy(deviceName, newDeviceName, sizeof(deviceName));
    updateMQTTTopic();
  }

  if (EEPROM.commit()) {
    Serial.println("WiFi credentials and settings saved to EEPROM");
  } else {
    Serial.println("Error saving WiFi credentials");
  }
}

// Save sleep configuration to EEPROM
void saveSleepConfig() {
  EEPROM.put(ADDR_SLEEP_ENABLED, deepSleepEnabled);
  EEPROM.put(ADDR_ACTIVE_WINDOW, activeWindow);
  EEPROM.put(ADDR_SLEEP_DURATION, sleepDuration);
  EEPROM.put(ADDR_MQTT_INTERVAL, mqttPublishInterval);

  if (EEPROM.commit()) {
    Serial.println("Sleep configuration saved to EEPROM");
  } else {
    Serial.println("Error saving sleep configuration");
  }
}

// Load sleep configuration from EEPROM
void loadSleepConfig() {
  EEPROM.get(ADDR_SLEEP_ENABLED, deepSleepEnabled);
  EEPROM.get(ADDR_ACTIVE_WINDOW, activeWindow);
  EEPROM.get(ADDR_SLEEP_DURATION, sleepDuration);
  EEPROM.get(ADDR_MQTT_INTERVAL, mqttPublishInterval);

  // Validate loaded values
  if (activeWindow < 60000 || activeWindow > 3600000) {  // 1 min to 60 min
    activeWindow = DEFAULT_ACTIVE_WINDOW;
  }
  if (sleepDuration < 60000000 || sleepDuration > 7200000000) {  // 1 min to 120 min
    sleepDuration = DEFAULT_SLEEP_DURATION;
  }
  if (mqttPublishInterval < 10000 || mqttPublishInterval > 300000) {  // 10 sec to 5 min
    mqttPublishInterval = DEFAULT_MQTT_INTERVAL;
  }

  Serial.printf("Sleep config - Enabled: %s, Active: %lu ms, Sleep: %lu us, MQTT: %lu ms\n",
                deepSleepEnabled ? "Yes" : "No", activeWindow, sleepDuration, mqttPublishInterval);
}

// --- EEPROM Functions ---
void initializeEEPROM() {
  EEPROM.begin(EEPROM_SIZE);

  // Check if EEPROM needs initialization using magic number
  uint32_t magicNumber;
  EEPROM.get(EEPROM_SIZE - 4, magicNumber);

  if (magicNumber != 0xDEADBEEF) {
    Serial.println("EEPROM not initialized, setting defaults...");

    // Set default tank configuration
    tankWidth = TANK_PRESETS[0][0];
    tankHeight = TANK_PRESETS[0][1];
    volumeFactor = TANK_PRESETS[0][2];
    calibration_mm = 0;

    // Save tank config
    EEPROM.put(ADDR_WIDTH, tankWidth);
    EEPROM.put(ADDR_HEIGHT, tankHeight);
    EEPROM.put(ADDR_VOL_FACTOR, volumeFactor);
    EEPROM.put(ADDR_CALIB, calibration_mm);

    // Clear WiFi credentials
    char empty[32] = { 0 };
    EEPROM.put(ADDR_WIFI_SSID, empty);
    EEPROM.put(ADDR_WIFI_PASS, empty);

    // Clear MAC address
    uint8_t emptyMAC[6] = { 0 };
    EEPROM.put(ADDR_WIFI_MAC, emptyMAC);

    // **CRITICAL: Initialize device name properly**
    EEPROM.put(ADDR_DEVICE_NAME, DEFAULT_DEVICE_NAME);

    // Initialize sleep configuration with defaults
    EEPROM.put(ADDR_SLEEP_ENABLED, true);
    EEPROM.put(ADDR_ACTIVE_WINDOW, DEFAULT_ACTIVE_WINDOW);
    EEPROM.put(ADDR_SLEEP_DURATION, DEFAULT_SLEEP_DURATION);
    EEPROM.put(ADDR_MQTT_INTERVAL, DEFAULT_MQTT_INTERVAL);

    // Set magic number to mark EEPROM as initialized
    magicNumber = 0xDEADBEEF;
    EEPROM.put(EEPROM_SIZE - 4, magicNumber);

    if (EEPROM.commit()) {
      Serial.println("EEPROM initialized with default values");
      Serial.printf("Device name set to: %s\n", DEFAULT_DEVICE_NAME);
    } else {
      Serial.println("EEPROM commit failed during initialization");
    }
  } else {
    Serial.println("EEPROM already initialized");
  }
}

void loadConfig() {
  // Load tank configuration
  EEPROM.get(ADDR_WIDTH, tankWidth);
  EEPROM.get(ADDR_HEIGHT, tankHeight);
  EEPROM.get(ADDR_VOL_FACTOR, volumeFactor);
  EEPROM.get(ADDR_CALIB, calibration_mm);

  // **CRITICAL: Load device name with proper validation**
  char nameBuf[32] = { 0 };
  EEPROM.get(ADDR_DEVICE_NAME, nameBuf);

  Serial.printf("Raw device name from EEPROM: ");
  for (int i = 0; i < 32; i++) {
    if (nameBuf[i] == 0) break;
    Serial.printf("%c", nameBuf[i]);
  }
  Serial.println();

  // Validate device name - check if it contains only printable characters
  bool validName = true;
  int nameLength = 0;

  for (int i = 0; i < sizeof(nameBuf); i++) {
    if (nameBuf[i] == 0) {
      nameLength = i;
      break;
    }
    // Check if character is printable ASCII (32-126)
    if (nameBuf[i] < 32 || nameBuf[i] > 126) {
      validName = false;
      break;
    }
  }

  if (validName && nameLength > 0 && nameLength < sizeof(deviceName)) {
    strlcpy(deviceName, nameBuf, sizeof(deviceName));
    Serial.printf("Valid device name loaded: %s\n", deviceName);
  } else {
    strlcpy(deviceName, DEFAULT_DEVICE_NAME, sizeof(deviceName));
    Serial.printf("Invalid device name in EEPROM, using default: %s\n", deviceName);

    // Fix the corrupted device name in EEPROM
    EEPROM.put(ADDR_DEVICE_NAME, DEFAULT_DEVICE_NAME);
    EEPROM.commit();
  }

  updateMQTTTopic();

  // Load sleep configuration
  loadSleepConfig();

  // Validate other loaded values
  if (isnan(tankWidth) || tankWidth <= 10 || tankWidth > 300) {
    Serial.printf("Invalid tankWidth: %.1f, using default\n", tankWidth);
    tankWidth = TANK_PRESETS[0][0];
  }
  if (isnan(tankHeight) || tankHeight <= 10 || tankHeight > 300) {
    Serial.printf("Invalid tankHeight: %.1f, using default\n", tankHeight);
    tankHeight = TANK_PRESETS[0][1];
  }
  if (isnan(volumeFactor) || volumeFactor <= 0 || volumeFactor > 50) {
    Serial.printf("Invalid volumeFactor: %.3f, using default\n", volumeFactor);
    volumeFactor = TANK_PRESETS[0][2];
  }
  if (calibration_mm < -1000 || calibration_mm > 1000) {
    Serial.printf("Invalid calibration: %d, resetting to 0\n", calibration_mm);
    calibration_mm = 0;
  }

  Serial.printf("Final config - Device: %s, Width: %.1f, Height: %.1f\n",
                deviceName, tankWidth, tankHeight);
}

// --- Optimized Sensor Functions ---

// Non-blocking sensor reading with improved speed
bool readDistanceFast(uint16_t &out_mm, float &out_cm) {
  static unsigned long lastTrigger = 0;
  static bool waitingForResponse = false;
  static unsigned long responseStartTime = 0;

  unsigned long currentTime = millis();

  // Trigger sensor reading if not already waiting and enough time has passed
  if (!waitingForResponse && (currentTime - lastTrigger >= 50)) {  // Reduced from 100ms
    sensorSerial.flush();
    sensorSerial.write(0x55);
    waitingForResponse = true;
    responseStartTime = currentTime;
    lastTrigger = currentTime;
    return false;  // Data not ready yet
  }

  // Check for response if we're waiting
  if (waitingForResponse) {
    // Timeout check - reduced from 100ms to 80ms
    if (currentTime - responseStartTime > 80) {
      waitingForResponse = false;
      return false;  // Timeout
    }

    // Check if we have enough data
    if (sensorSerial.available() >= 4) {
      uint8_t buffer[4];
      int bytesRead = sensorSerial.readBytes(buffer, 4);

      if (bytesRead == 4 && buffer[0] == 0xFF) {
        uint16_t raw_mm = (buffer[1] << 8) | buffer[2];

        // Basic range validation to filter out obvious errors
        if (raw_mm > 50 && raw_mm < 5000) {  // 5cm to 500cm range
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

// Sensor Reading Manager Implementation
bool SensorReadingManager::getStableReading(uint16_t &out_mm, float &out_cm) {
  uint16_t raw_mm;
  float raw_cm;

  if (readDistanceFast(raw_mm, raw_cm)) {
    // Outlier detection - reject readings that are too different from recent average
    if (initialized && validReadings > 2) {
      float currentAverage = getAverage();
      float difference = abs(raw_cm - currentAverage);

      // Reject reading if it's more than 20cm different from average (adjustable)
      if (difference > 20.0f) {
        consecutiveErrors++;

        // If we have too many consecutive errors, reset the buffer
        if (consecutiveErrors > 3) {
          reset();
        }
        return false;
      }
    }

    // Add reading to buffer
    readings[bufferIndex] = raw_cm;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    if (validReadings < BUFFER_SIZE) {
      validReadings++;
    }

    consecutiveErrors = 0;
    lastSuccessfulRead = millis();
    lastValidReading = raw_cm;
    initialized = true;

    // Return averaged result
    float averaged = getAverage();
    out_cm = averaged;
    out_mm = (uint16_t)(averaged * 10);

    return true;
  }

  return false;
}

float SensorReadingManager::getAverage() {
  if (validReadings == 0) return 0;

  float sum = 0;
  for (int i = 0; i < validReadings; i++) {
    sum += readings[i];
  }
  return sum / validReadings;
}

bool SensorReadingManager::hasRecentReading() {
  return (millis() - lastSuccessfulRead) < 5000;  // 5 seconds
}

void SensorReadingManager::reset() {
  bufferIndex = 0;
  validReadings = 0;
  consecutiveErrors = 0;
  initialized = false;
}

int SensorReadingManager::getValidReadingsCount() {
  return validReadings;
}

// Main reading function that uses the optimized methods
bool readDistance(uint16_t &out_mm, float &out_cm) {
  return sensorManager.getStableReading(out_mm, out_cm);
}

// Function to check sensor health and connectivity
bool checkSensorHealth() {
  uint16_t test_mm;
  float test_cm;

  // Try a burst read to test connectivity
  return readDistanceBurst(test_mm, test_cm, 2);
}

// Function to get sensor statistics
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
  return volumeFactor * waterHeight;  // Uses volumeFactor instead of W×W
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

  int result = esp_now_send(requestingMAC, (uint8_t *)&msg, sizeof(msg));
  if (result != 0) {
    Serial.println("{\"error\":\"ESP-NOW send failed\"}");
  }
}

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (len != sizeof(struct_message)) return;

  struct_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  msg.cmd[sizeof(msg.cmd) - 1] = '\0';

  if (strcmp(msg.cmd, "get_distance") == 0) {
    memcpy(requestingMAC, mac, 6);
    pendingDistanceRequest = true;
  } else if (strcmp(msg.cmd, "get_measure") == 0) {
    memcpy(requestingMAC, mac, 6);
    pendingDistanceRequest_2 = true;
  }
}

// Modified ESP-NOW handler functions for faster response
void handleESPNowRequest() {
  if (pendingDistanceRequest || pendingDistanceRequest_2) {
    uint16_t burst_mm;
    float burst_cm;

    unsigned long startTime = millis();

    // Use burst reading for ESP-NOW requests for faster response
    bool success = readDistanceBurst(burst_mm, burst_cm, 2);

    unsigned long readTime = millis() - startTime;
    performanceMetrics.addMeasurement(readTime, success);

    if (success) {
      // Update global variables for consistency
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

  // Check battery before entering deep sleep
  checkBattery();

  if (batteryVoltage < BATTERY_CRITICAL_THRESHOLD) {
    Serial.println("Battery critically low. Entering extended deep sleep (1 hour)...");

    // Extended sleep for low battery
    ESP.deepSleep(3600000000);  // 1 hour
    return;
  }

  Serial.println("Preparing for deep sleep...");

  // Save current state to EEPROM if needed
  saveConfig();

  // Close connections gracefully
  server.stop();
  mqttClient.disconnect(true);
  WiFi.disconnect();

  Serial.println("Entering deep sleep...");
  Serial.printf("Active window was: %lu ms\n", millis() - activeWindowStart);
  Serial.printf("Battery: %.2fV (%.0f%%)\n", batteryVoltage, batteryPercentage);

  // Blink LED to indicate sleep
  //blinkBlueLED(3, 200);

  // Configure wakeup source and enter deep sleep
  ESP.deepSleep(sleepDuration, WAKE_RF_DEFAULT);

  // Small delay to ensure deep sleep command is processed
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

void handleRoot() {
  updateMeasurements();
  // checkBattery();  // Update battery info

  String waterColor = (percent < 20) ? "#ff4444" : "#4285f4";
  float maxVolume = volumeFactor * tankHeight;

  // Add active window countdown to HTML
  unsigned long remainingActiveTime = activeWindow - (millis() - activeWindowStart);
  unsigned long remainingMinutes = remainingActiveTime / 60000;
  unsigned long remainingSeconds = (remainingActiveTime % 60000) / 1000;

  String html = FPSTR(PAGE_ROOT);

  // Replace placeholders with actual values
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

  // Add sleep info
  unsigned long activeTime = millis() - activeWindowStart;
  unsigned long activeMinutes = activeTime / 60000;
  unsigned long activeSeconds = (activeTime % 60000) / 1000;
  String activeTimeStr = String(activeMinutes) + "m " + String(activeSeconds) + "s";

  unsigned long sleepTime = activeWindow - activeTime;
  unsigned long sleepMinutes = sleepTime / 60000;
  unsigned long sleepSeconds = (sleepTime % 60000) / 1000;
  String sleepCountdownStr = String(sleepMinutes) + "m " + String(sleepSeconds) + "s";

  html.replace("%ACTIVE_TIME%", activeTimeStr);
  html.replace("%SLEEP_COUNTDOWN%", sleepCountdownStr);
  html.replace("%NEXT_ONLINE_TIME%", getNextOnlineTime());

  // Add battery info
  html.replace("%BATTERY_VOLTAGE%", String(batteryVoltage, 2));
  html.replace("%BATTERY_PERCENTAGE%", String(batteryPercentage, 0));
  html.replace("%BATTERY_STATUS%", getBatteryStatusString());

  server.send(200, "text/html", html);
}

void handleData() {
  updateMeasurements();
  // checkBattery();  // Update battery info

  // Include sensor health information
  bool hasRecent = sensorManager.hasRecentReading();
  int validReadings = sensorManager.getValidReadingsCount();

  // Calculate active window times
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
  json += "\"battery_voltage\":" + String(batteryVoltage, 2) + ",";
  json += "\"battery_percentage\":" + String(batteryPercentage, 0) + ",";
  json += "\"next_online\":\"" + getNextOnlineTime() + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleSensorStats() {
  String json = getSensorStats();
  server.send(200, "application/json", json);
}

void handleConfig() {
  // Update battery info first
  // checkBattery();

  // Handle form submission
  if (server.hasArg("save")) {
    if (server.hasArg("preset_mode") && server.arg("preset_mode") == "1") {
      // Preset mode selected
      int presetIndex = server.arg("preset_size").toInt();
      if (presetIndex >= 0 && presetIndex < sizeof(TANK_PRESETS) / sizeof(TANK_PRESETS[0])) {
        tankWidth = TANK_PRESETS[presetIndex][0];
        tankHeight = TANK_PRESETS[presetIndex][1];
        volumeFactor = TANK_PRESETS[presetIndex][2];
      }
    } else {
      // Manual mode selected
      tankWidth = server.arg("manual_width").toFloat();
      tankHeight = server.arg("manual_height").toFloat();

      // Calculate volume factor (assuming rectangular tank)
      volumeFactor = (tankWidth * tankWidth) / 1000.0f;  // converts cm³ to liters per cm height
    }

    // Save calibration if provided
    if (server.hasArg("calibration")) {
      calibration_mm = server.arg("calibration").toInt() * 10;  // convert cm to mm
    }

    saveConfig();
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  // Generate the configuration page
  String html = FPSTR(PAGE_CONFIG);

  // Replace placeholders
  html.replace("%TANK_WIDTH%", String(tankWidth, 1));
  html.replace("%TANK_HEIGHT%", String(tankHeight, 1));
  html.replace("%CALIBRATION%", String(calibration_mm / 10.0f, 1));
  html.replace("%BATTERY_VOLTAGE%", String(batteryVoltage, 2));
  html.replace("%BATTERY_PERCENTAGE%", String(batteryPercentage, 0));
  html.replace("%BATTERY_STATUS%", getBatteryStatusString());

  server.send(200, "text/html", html);
}

// Sleep Configuration Page
void handleSleepConfig() {
  // Update battery info first
  // checkBattery();

  if (server.hasArg("save")) {
    // Handle form submission
    deepSleepEnabled = server.hasArg("sleep_enabled");

    if (server.hasArg("active_window")) {
      activeWindow = server.arg("active_window").toInt() * 60000;  // Convert minutes to milliseconds
    }

    if (server.hasArg("sleep_duration")) {
      sleepDuration = server.arg("sleep_duration").toInt() * 60000000;  // Convert minutes to microseconds
    }

    if (server.hasArg("mqtt_interval")) {
      mqttPublishInterval = server.arg("mqtt_interval").toInt() * 1000;  // Convert seconds to milliseconds
    }

    // Save configuration to EEPROM
    saveSleepConfig();

    // Generate saved page with battery info
    String html = FPSTR(PAGE_SLEEP_SAVED);
    html.replace("%BATTERY_VOLTAGE%", String(batteryVoltage, 2));
    html.replace("%BATTERY_PERCENTAGE%", String(batteryPercentage, 0));

    // Send success response
    server.send(200, "text/html", html);
    return;
  }

  // Generate the sleep configuration page
  String html = FPSTR(PAGE_SLEEP_CONFIG);

  // Replace placeholders
  html.replace("%SLEEP_STATUS%", deepSleepEnabled ? "Sleep Enabled" : "Sleep Disabled");
  html.replace("%ACTIVE_TIME%", String(activeWindow / 60000));
  html.replace("%SLEEP_TIME%", String(sleepDuration / 60000000));
  html.replace("%MQTT_INTERVAL%", String(mqttPublishInterval / 1000));
  html.replace("%ACTIVE_WINDOW%", String(activeWindow / 60000));
  html.replace("%SLEEP_DURATION%", String(sleepDuration / 60000000));
  html.replace("%MQTT_INTERVAL_VAL%", String(mqttPublishInterval / 1000));
  html.replace("%SLEEP_CHECKED%", deepSleepEnabled ? "checked" : "");
  html.replace("%BATTERY_VOLTAGE%", String(batteryVoltage, 2));
  html.replace("%BATTERY_PERCENTAGE%", String(batteryPercentage, 0));
  html.replace("%BATTERY_STATUS%", getBatteryStatusString());

  server.send(200, "text/html", html);
}

// Benchmarking Function (for web)
void runSensorBenchmark(int iterations = 50) {
  Serial.println("{\"benchmark\":\"starting\"}");
  performanceMetrics.reset();

  for (int i = 0; i < iterations; i++) {
    uint16_t test_mm;
    float test_cm;

    unsigned long startTime = micros();
    bool success = readDistanceBurst(test_mm, test_cm, 1);
    unsigned long endTime = micros();

    unsigned long readTime = endTime - startTime;
    performanceMetrics.addMeasurement(readTime / 1000, success);

    Serial.printf("{\"iteration\":%d,\"time_ms\":%lu,\"success\":%s,\"distance\":%.1f}\n",
                  i + 1, readTime / 1000, success ? "true" : "false",
                  success ? test_cm : 0.0f);

    delay(100);
  }

  Serial.printf("{\"benchmark_complete\":{\"success_rate\":%.1f,\"avg_time_ms\":%lu,\"min_time_ms\":%lu,\"max_time_ms\":%lu}}\n",
                performanceMetrics.getSuccessRate(),
                performanceMetrics.averageReadTime,
                performanceMetrics.minReadTime,
                performanceMetrics.maxReadTime);
}

void handleBenchmark() {
  if (server.hasArg("run")) {
    int iterations = server.hasArg("iterations") ? server.arg("iterations").toInt() : 20;
    iterations = constrain(iterations, 5, 30);  // Safe limit for ESP8266

    // Use String with pre-allocation
    String html = String();
    html.reserve(4000);  // Reserve memory to prevent fragmentation

    // HTML Header with styles
    html += R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Benchmark Results</title>
    <style>
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f5f5f5;
            color: #333;
            line-height: 1.6;
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
        .progress-bar {
            height: 20px;
            background-color: #e0e0e0;
            border-radius: 10px;
            overflow: hidden;
            margin: 20px 0;
        }
        .progress-fill {
            height: 100%;
            background-color: #4CAF50;
            width: 0%;
            transition: width 0.5s ease;
        }
        .summary-stats {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
            margin-bottom: 20px;
        }
        .stat-card {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 8px;
        }
        .stat-card h3 {
            margin: 0 0 5px 0;
            font-size: 14px;
            color: #666;
        }
        .stat-card p {
            margin: 0;
            font-size: 18px;
            font-weight: 500;
        }
        .success-rate {
            color: #4CAF50;
            font-weight: 600;
        }
        .readings-table {
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
            font-size: 14px;
        }
        .readings-table th, .readings-table td {
            padding: 8px 12px;
            text-align: left;
            border-bottom: 1px solid #eee;
        }
        .readings-table th {
            background-color: #f8f9fa;
            font-weight: 500;
            position: sticky;
            top: 0;
        }
        .success {
            color: #4CAF50;
        }
        .failure {
            color: #F44336;
        }
        .back-link {
            display: inline-block;
            margin-top: 20px;
            color: #3498db;
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
        <h2>Benchmark Results</h2>
        
        <div class="progress-bar">
            <div class="progress-fill" id="progressFill"></div>
        </div>
        
        <div class="summary-stats">
            <div class="stat-card">
                <h3>Iterations</h3>
                <p id="iterationsCount">)=====";
    html += iterations;
    html += R"=====(</p>
            </div>
            <div class="stat-card">
                <h3>Successful Readings</h3>
                <p id="successCount">0</p>
            </div>
            <div class="stat-card">
                <h3>Success Rate</h3>
                <p class="success-rate" id="successRate">0%</p>
            </div>
            <div class="stat-card">
                <h3>Avg. Time</h3>
                <p id="avgTime">0 ms</p>
            </div>
        </div>
        
        <div class="table-container">
            <table class="readings-table">
                <thead>
                    <tr>
                        <th>#</th>
                        <th>Status</th>
                        <th>Distance (cm)</th>
                        <th>Time (ms)</th>
                    </tr>
                </thead>
                <tbody id="resultsTable">
    )=====";

    // Run benchmark in chunks for stability
    int successCount = 0;
    unsigned long totalTime = 0;
    unsigned long minTime = ULONG_MAX;
    unsigned long maxTime = 0;
    const int chunkSize = 5;

    for (int chunkStart = 0; chunkStart < iterations; chunkStart += chunkSize) {
      int chunkEnd = min(chunkStart + chunkSize, iterations);

      for (int i = chunkStart; i < chunkEnd; i++) {
        uint16_t test_mm;
        float test_cm;

        unsigned long startTime = micros();
        bool success = readDistanceBurst(test_mm, test_cm, 1);
        unsigned long endTime = micros();
        unsigned long readTime = endTime - startTime;

        // Add row to HTML table
        html += "<tr><td>";
        html += i + 1;
        html += "</td><td class='";
        html += success ? "success'>Success" : "failure'>Failed";
        html += "</td><td>";
        html += success ? String(test_cm, 1) : "N/A";
        html += "</td><td>";
        html += String(readTime / 1000.0, 1);
        html += "</td></tr>";

        // Update stats
        if (success) {
          successCount++;
          totalTime += readTime;
          if (readTime < minTime) minTime = readTime;
          if (readTime > maxTime) maxTime = readTime;
        }

        delay(50);

        // Prevent watchdog timeout
        if (i % 3 == 0) {
          server.handleClient();
          yield();
        }
      }

      // Maintain system stability
      ESP.wdtFeed();
      yield();
    }

    float successRate = iterations > 0 ? (float)successCount / iterations * 100.0f : 0;
    unsigned long avgTime = successCount > 0 ? totalTime / successCount : 0;

    // Complete the HTML
    html += R"=====(
                </tbody>
            </table>
        </div>
        
        <a href="/benchmark" class="back-link">← Run Another Benchmark</a>
        <a href="/" class="back-link">← Back to Dashboard</a>
    </div>
    
    <script>
        // Update stats with final values
        document.addEventListener('DOMContentLoaded', function() {
            document.getElementById('successCount').textContent = ')=====";
    html += successCount;
    html += R"=====(';
            document.getElementById('successRate').textContent = ')=====";
    html += String(successRate, 1);
    html += R"=====(%';
            document.getElementById('avgTime').textContent = ')=====";
    html += String(avgTime / 1000.0, 1);
    html += R"=====( ms';
            
            const progressFill = document.getElementById('progressFill');
            const successRate = )=====";
    html += successRate;
    html += R"=====(;
            progressFill.style.width = successRate + '%';
            
            // Color coding based on success rate
            if (successRate < 50) {
                progressFill.style.backgroundColor = '#F44336';
                document.getElementById('successRate').className = 'failure';
            } else if (successRate < 80) {
                progressFill.style.backgroundColor = '#FF9800';
                document.getElementById('successRate').className = '';
            }
        });
    </script>
</body>
</html>
)=====";

    server.send(200, "text/html", html);
    html = String();  // Free memory
  } else {
    // Show benchmark form
    server.send(200, "text/html", FPSTR(PAGE_BENCHMARK_FORM));
  }
}

// Add this function to handle WiFi setup page
void handleWiFiSetup() {
  // Update battery info first
  // checkBattery();

  if (server.hasArg("save")) {
    String newSSID = server.arg("ssid");
    String newPass = server.arg("password");
    String newMAC = server.arg("mac");
    String newDeviceName = server.arg("device_name");

    // Validate device name
    if (newDeviceName.length() == 0) {
      newDeviceName = DEFAULT_DEVICE_NAME;
    }

    saveWiFiConfig(newSSID.c_str(), newPass.c_str(), newMAC.c_str(), newDeviceName.c_str());

    // Send reboot countdown page
    server.send(200, "text/html", FPSTR(PAGE_WIFI_SAVED));
    delay(5000);
    ESP.restart();
  } else {
    char ssidBuf[32] = { 0 }, passBuf[32] = { 0 };
    uint8_t mac[6] = { 0 };
    char nameBuf[32] = { 0 };

    EEPROM.get(ADDR_WIFI_SSID, ssidBuf);
    EEPROM.get(ADDR_WIFI_PASS, passBuf);
    EEPROM.get(ADDR_WIFI_MAC, mac);
    EEPROM.get(ADDR_DEVICE_NAME, nameBuf);

    // Format MAC address for display
    String macStr;
    if (mac[0] != 0 || mac[1] != 0 || mac[2] != 0 || mac[3] != 0 || mac[4] != 0 || mac[5] != 0) {
      char macBuf[18];
      sprintf(macBuf, "%02X:%02X:%02X:%02X:%02X:%02X",
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      macStr = String(macBuf);
    }

    // Get current device name
    String currentDeviceName = (strlen(nameBuf) > 0) ? String(nameBuf) : String(DEFAULT_DEVICE_NAME);

    // Build the HTML response
    String html = FPSTR(PAGE_WIFI_SETUP);

    // Replace placeholders
    html.replace("%DEVICE_NAME%", currentDeviceName);
    html.replace("%MAC_ADDRESS%", WiFi.macAddress());
    html.replace("%SSID%", String(ssidBuf));
    html.replace("%PASSWORD%", String(passBuf));
    html.replace("%MAC%", macStr);
    html.replace("%BATTERY_VOLTAGE%", String(batteryVoltage, 2));
    html.replace("%BATTERY_PERCENTAGE%", String(batteryPercentage, 0));
    html.replace("%BATTERY_STATUS%", getBatteryStatusString());

    server.send(200, "text/html", html);
  }
}

// Optimized sensor handling
void handleSensorReading() {
  static unsigned long lastWebUpdate = 0;
  unsigned long currentTime = millis();

  // For web dashboard, use stable averaged readings
  if (currentTime - lastWebUpdate >= 1000) {  // Update every second for web
    uint16_t stable_mm;
    float stable_cm;

    if (readDistance(stable_mm, stable_cm)) {
      mm = stable_mm;
      cm = stable_cm;
      updateMeasurements();
      lastWebUpdate = currentTime;
    }
  }

  // Handle ESP-NOW requests immediately with burst reading
  handleESPNowRequest();
}

String getCustomTimestamp() {
  if (!timeClient.isTimeSet()) {
    // Try to update time once
    timeClient.update();

    // If still not set, return uptime-based timestamp
    if (!timeClient.isTimeSet()) {
      unsigned long uptime = millis();
      unsigned long hours = uptime / 3600000;
      unsigned long minutes = (uptime % 3600000) / 60000;

      char timestamp[20];
      snprintf(timestamp, sizeof(timestamp),
               "UP%03lu:%02lu",
               hours, minutes);

      return String(timestamp);
    }
  }

  // Get NTP time
  time_t rawtime = timeClient.getEpochTime();
  struct tm *ti = localtime(&rawtime);

  char timestamp[20];
  snprintf(timestamp, sizeof(timestamp),
           "%02d-%02d-%04dT%02d:%02d",
           ti->tm_mday,
           ti->tm_mon + 1,
           ti->tm_year + 1900,
           ti->tm_hour,
           ti->tm_min);

  return String(timestamp);
}

void publishMQTTStatus() {
  if (!mqttClient.connected()) {
    Serial.println("MQTT not connected, skip publish");
    return;
  }

  // Update battery before publishing
  // checkBattery();

  char payload[350];  // Increased buffer size for battery info
  String timestamp = getCustomTimestamp();

  // Calculate next sleep time
  String nextSleepTime;
  if (deepSleepEnabled) {
    unsigned long currentTime = millis();
    unsigned long timeSinceActiveStart = currentTime - activeWindowStart;

    if (timeSinceActiveStart < activeWindow) {
      // Still in active window
      unsigned long sleepStartTime = activeWindowStart + activeWindow;
      time_t now;
      if (timeClient.isTimeSet()) {
        now = timeClient.getEpochTime();
      } else {
        now = 1700000000 + (currentTime / 1000);
      }
      unsigned long secondsUntilSleep = (sleepStartTime - currentTime) / 1000;
      time_t sleepStartEpoch = now + secondsUntilSleep;

      struct tm *ti = localtime(&sleepStartEpoch);
      char sleepTimeStr[30];
      snprintf(sleepTimeStr, sizeof(sleepTimeStr), "%02d-%02d-%04dT%02d:%02d",
               ti->tm_mday,
               ti->tm_mon + 1,
               ti->tm_year + 1900,
               ti->tm_hour,
               ti->tm_min);
      nextSleepTime = String(sleepTimeStr);
    } else {
      // Should already be sleeping
      nextSleepTime = "sleeping_now";
    }
  } else {
    nextSleepTime = "always_online";
  }

  // Calculate next wake time for "next_online"
  String nextOnlineTime = getNextOnlineTimeForMQTT();

  snprintf(payload, sizeof(payload),
           "{\"timestamp\":\"%s\","
           "\"uptime\":\"%s\","
           "\"device\":\"%s\","
           "\"ip_address\":\"%s\","
           "\"distance_cm\":%.1f,"
           "\"level_percent\":%.1f,"
           "\"volume_liters\":%.1f,"
           "\"battery_voltage\":%.2f,"
           "\"battery_percentage\":%.0f,"
           "\"battery_status\":\"%s\","
           "\"next_sleep\":\"%s\","
           "\"next_online\":\"%s\"}",
           timestamp.c_str(),
           formatRuntime(millis()).c_str(),
           deviceName,
           WiFi.localIP().toString().c_str(),
           cm,
           percent,
           volume,
           batteryVoltage,
           batteryPercentage,
           getBatteryStatusString().c_str(),
           nextSleepTime.c_str(),
           nextOnlineTime.c_str());

  if (mqttClient.publish(PubTopic, 0, true, payload)) {
    Serial.println("MQTT published: " + String(payload));
  } else {
    Serial.println("MQTT publish failed");
  }
}
//=============== start of MQTT funtions ====================

void printSeparationLine() {
  Serial.println("************************************************");
}

void generateClientId(char *buffer, size_t size) {
  // Generate unique client ID with timestamp and random number
  uint32_t timestamp = millis();
  uint32_t randomNum = random(1000, 9999);
  snprintf(buffer, size, "esp32_%lu_%lu", timestamp, randomNum);
}

//For Persistent Client ID with Unique Suffix:
void generatePersistentButUniqueClientId(char *buffer, size_t size) {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  uint32_t timestamp = millis();

  // Base ID from MAC + unique timestamp suffix
  snprintf(buffer, size, "esp32_%02x%02x%02x_%lu",
           mac[3], mac[4], mac[5], timestamp);
}

void connectToMqtt() {
  if (mqttClient.connected()) return;  // already connected

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
  (void)reason;

  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttMessage(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties,
                   const size_t &len, const size_t &index, const size_t &total) {
  (void)payload;

  // Serial.println("Publish received.");
  // Serial.print("  topic: ");
  // Serial.println(topic);
  // Serial.print("  qos: ");
  // Serial.println(properties.qos);
  // Serial.print("  dup: ");
  // Serial.println(properties.dup);
  // Serial.print("  retain: ");
  // Serial.println(properties.retain);
  // Serial.print("  len: ");
  // Serial.println(len);
  // Serial.print("  index: ");
  // Serial.println(index);
  // Serial.print("  total: ");
  // Serial.println(total);
}

void onMqttPublish(const uint16_t &packetId) {
  // Serial.println("Publish acknowledged.");
  // Serial.print("  packetId: ");
  // Serial.println(packetId);
  blinkBlueLED(1);
}

//=============== end of MQTT funtions ====================
// ===== NEW: Wi-Fi event handler for MQTT auto-reconnect =====
void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case WIFI_EVENT_STAMODE_GOT_IP:
      Serial.println("[WiFi] Re-connected → forcing MQTT reconnect");
      mqttReconnectTimer.detach();  // cancel any pending attempt
      mqttReconnectTimer.once(1, connectToMqtt);
      break;

    case WIFI_EVENT_STAMODE_DISCONNECTED:
      Serial.println("[WiFi] Lost connection → MQTT will auto-retry");
      break;

    default:
      break;
  }
}
// ============================================================

void ForceAPMode(bool enable, float minutes) {
  if (!enable) return;

  if (minutes <= 0) minutes = 1;  // Safety: minimum 1 minute

  unsigned long apDuration = (unsigned long)(minutes * 60 * 1000);

  Serial.printf("ForceAPMode(true, %.2f min) → Starting AP mode\n", minutes);
  digitalWrite(LED_RED, LED_ON);

  // --- Start AP Mode ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(deviceName, WIFI_AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("AP Mode IP: %s\n", apIP.toString().c_str());

  // Web server routes
  server.on("/", handleRoot);
  server.on("/wifi-setup", handleWiFiSetup);
  server.on("/config", handleConfig);
  server.on("/data", handleData);
  server.on("/sensor-stats", handleSensorStats);
  server.on("/benchmark", handleBenchmark);
  server.on("/sleep-config", handleSleepConfig);
  server.onNotFound(handleRoot);
  server.begin();

  // DNS server
  dnsServer.start(DNS_PORT, "*", apIP);

  // mDNS
  if (MDNS.begin(deviceName)) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS responder started");
    Serial.printf("Access: http://%s.local\n", deviceName);
  }

  // OTA
  ArduinoOTA.setHostname(deviceName);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();

  unsigned long apStartTime = millis();
  Serial.printf("AP Mode active for %.1f minutes...\n", minutes);

  // --- AP Loop ---
  while (millis() - apStartTime < apDuration) {
    ArduinoOTA.handle();
    server.handleClient();
    dnsServer.processNextRequest();
    MDNS.update();
    handleSensorReading();
    yield();

    // LED blink every 2 seconds
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
  char ssidBuf[32] = { 0 };
  char passBuf[32] = { 0 };
  EEPROM.get(ADDR_WIFI_SSID, ssidBuf);
  EEPROM.get(ADDR_WIFI_PASS, passBuf);

  String ssid = String(ssidBuf);
  String password = String(passBuf);

  if (ssid.length() == 0) {
    Serial.println("{\"error\":\"No saved SSID\"}");
    return false;
  }

  Serial.printf("Connecting to WiFi: %s\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWiFiEvent);  // register Wi-Fi events
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());

    // Initialize and sync NTP time
    timeClient.begin();
    timeClient.update();

    // Wait for time to sync (with timeout)
    Serial.println("Syncing NTP time...");
    unsigned long syncStart = millis();
    while (!timeClient.isTimeSet() && (millis() - syncStart < 10000)) {
      timeClient.update();
      delay(1000);
      Serial.print(".");
    }

    if (timeClient.isTimeSet()) {
      Serial.println("\nNTP time synced!");
      Serial.printf("Current time: %s\n", timeClient.getFormattedTime().c_str());
    } else {
      Serial.println("\nNTP sync failed!");
    }

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

// ===== BATTERY FUNCTIONS =====

void initBatteryADC() {
  if (!hasBatteryMonitoring) {
    Serial.println("Battery monitoring disabled in config");
    return;
  }
  
  // Initialize ADC for battery monitoring
  // ESP8266 uses A0 pin for ADC with internal 1.0V reference
  
  Serial.println("==============================================");
  Serial.println("BATTERY MONITORING INITIALIZATION");
  Serial.println("==============================================");
  Serial.printf("ADC Pin: A0 (GPIO %d)\n", BATTERY_PIN);
  Serial.printf("ADC Resolution: 10-bit (0-1023)\n");
  Serial.printf("ADC Reference Voltage: 1.0V (ESP8266 internal)\n");
  Serial.printf("Voltage Divider: 47KΩ / 10KΩ\n");
  Serial.printf("Voltage Divider Ratio: %.1f\n", VOLTAGE_DIVIDER_RATIO);
  Serial.printf("Calibration Factor: %.3f\n", BATTERY_CALIBRATION_FACTOR);
  Serial.printf("Battery Thresholds:\n");
  Serial.printf("  - Full: %.1fV (Li-ion 100%%)\n", BATTERY_FULL_THRESHOLD);
  Serial.printf("  - Warning: %.1fV\n", BATTERY_WARNING_THRESHOLD);
  Serial.printf("  - Critical: %.1fV (deep sleep)\n", BATTERY_CRITICAL_THRESHOLD);
  Serial.printf("  - Empty: %.1fV (0%%)\n", BATTERY_EMPTY_THRESHOLD);
  Serial.println("==============================================");
  
  // Configure ADC pin
  pinMode(BATTERY_PIN, INPUT);
  
  // Optional: Set ADC read resolution
  //analogReadResolution(10);  // 10-bit resolution (default)
  
  // Optional: Set ADC attenuation (if supported - ESP32 only, not ESP8266)
  // For ESP8266, attenuation is fixed
  
  // Take an initial reading to stabilize ADC
  float initialReading = readBatteryVoltage();
  Serial.printf("Initial Battery Reading: %.3fV\n", initialReading);
  
  // Verify reading is reasonable
  if (initialReading < 2.0f || initialReading > 5.0f) {
    Serial.println("[WARNING] Initial battery reading appears incorrect!");
    Serial.println("Check voltage divider wiring and resistor values.");
    
    // Optional: You might want to disable battery monitoring if readings are clearly wrong
    // hasBatteryMonitoring = false;
  }
  
  Serial.println("Battery ADC initialized successfully");
  Serial.println("==============================================");
}

String getBatteryStatusString() {
  if (!hasBatteryMonitoring) return "No ADC";

  if (batteryVoltage >= 3.7f) {
    return "Good";
  } else if (batteryVoltage >= 3.3f) {
    return "Low";
  } else if (batteryVoltage >= BATTERY_CRITICAL_THRESHOLD) {
    return "Warning";
  } else {
    return "Critical";
  }
}

float readBatteryVoltage() {
  if (!hasBatteryMonitoring) return 0.0f;

  // Quick reading - 10 samples for good accuracy
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(5); // Better ADC stability, fewer noisy spikes
  }
  
  float averageADC = (float)sum / 10.0f;
  
  // CALIBRATION FACTOR BASED ON YOUR MEASUREMENTS:
  // Original: 3.89V battery ÷ 234 ADC = 0.0166239
  // Adjusted: 3.89V battery ÷ 229 ADC = 0.016986
  static const float CALIBRATION = 0.016624f;  // Current working value
  
  float batteryVoltage = averageADC * CALIBRATION;
  
  // Simple moving average for smoothing
  static float smoothedVoltage = batteryVoltage;
  static bool firstReading = true;
  
  if (firstReading) {
    smoothedVoltage = batteryVoltage;
    firstReading = false;
  } else {
    // Low-pass filter: 70% new, 30% old
    smoothedVoltage = (batteryVoltage * 0.7f) + (smoothedVoltage * 0.3f);
  }
  
  // Log occasionally to avoid Serial spam
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 60000) {  // Log every 60 seconds
    lastLog = millis();
    Serial.printf("[BATT] ADC: %.1f -> %.2fV (smoothed: %.2fV)\n", 
                  averageADC, batteryVoltage, smoothedVoltage);
  }
  
  return smoothedVoltage;
}

void checkBattery() {
  if (!hasBatteryMonitoring) return;

  // Read battery voltage from ADC
  batteryVoltage = readBatteryVoltage();
  
  // Debug output
  // Serial.printf("DEBUG: Battery voltage = %.3fV\n", batteryVoltage);

  // Calculate battery percentage (simple linear approximation for Li-ion)
  if (batteryVoltage >= BATTERY_FULL_THRESHOLD) {
    batteryPercentage = 100.0f;
  } else if (batteryVoltage <= BATTERY_EMPTY_THRESHOLD) {
    batteryPercentage = 0.0f;
  } else {
    // Linear interpolation between empty and full thresholds
    batteryPercentage = ((batteryVoltage - BATTERY_EMPTY_THRESHOLD) / 
                        (BATTERY_FULL_THRESHOLD - BATTERY_EMPTY_THRESHOLD)) * 100.0f;
    batteryPercentage = constrain(batteryPercentage, 0.0f, 100.0f);
  }

  // Check if battery is critically low
  if (batteryVoltage < BATTERY_CRITICAL_THRESHOLD) {
    lowBatteryMode = true;
    Serial.printf("CRITICAL: Battery voltage %.2fV is below %.2fV threshold\n",
                  batteryVoltage, BATTERY_CRITICAL_THRESHOLD);

    // If battery is critically low, go to extended deep sleep
    if (deepSleepEnabled) {
      Serial.println("Battery critically low. Entering extended deep sleep...");

      // Disable WiFi and MQTT to save power
      WiFi.disconnect();
      mqttClient.disconnect();

      // Save current config
      saveConfig();

      // Enter deep sleep for 1 hour (or longer) to conserve battery
      ESP.deepSleep(3600000000);  // 1 hour in microseconds
      
      // Code execution stops here if deep sleep is successful
    }
  } else if (batteryVoltage < BATTERY_WARNING_THRESHOLD) {
    lowBatteryMode = true;
    Serial.printf("WARNING: Battery voltage %.2fV is low\n", batteryVoltage);
  } else {
    lowBatteryMode = false;
  }

  // Log battery status periodically (every minute)
  static unsigned long lastBatteryLog = 0;
  if (millis() - lastBatteryLog > 60000) {  // Log every minute
    lastBatteryLog = millis();
    
    // Get battery status string for logging
    String statusStr = getBatteryStatusString();
    
    Serial.printf("Battery Status: %.2fV (%.0f%%) - %s\n", 
                  batteryVoltage, batteryPercentage, statusStr.c_str());
    
    // Also log to MQTT if connected (optional)
    if (mqttClient.connected()) {
      char mqttPayload[100];
      snprintf(mqttPayload, sizeof(mqttPayload),
               "{\"battery_voltage\":%.2f,\"battery_percentage\":%.0f,\"status\":\"%s\"}",
               batteryVoltage, batteryPercentage, statusStr.c_str());
      
      // Create topic for battery status
      char batteryTopic[64];
      snprintf(batteryTopic, sizeof(batteryTopic), "%s/battery", deviceName);
      
      mqttClient.publish(batteryTopic, 0, false, mqttPayload);
    }
  }
  
  // Update LED indicator based on battery status
  static unsigned long lastBatteryLEDUpdate = 0;
  if (millis() - lastBatteryLEDUpdate > 5000) {  // Update every 5 seconds
    lastBatteryLEDUpdate = millis();
    
    if (lowBatteryMode) {
      // Blink red LED for low battery warning
      static bool ledState = false;
      digitalWrite(LED_RED, ledState ? LED_ON : LED_OFF);
      ledState = !ledState;
    } else if (batteryVoltage < BATTERY_WARNING_THRESHOLD) {
      // Solid red LED for warning level
      digitalWrite(LED_RED, LED_ON);
    } else {
      // Turn off red LED for normal battery
      digitalWrite(LED_RED, LED_OFF);
    }
  }
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
  } else if (command == "eeprom_reset") {  // ADD THIS NEW COMMAND
    Serial.println("Resetting EEPROM to defaults...");

    // Clear the magic number to force re-initialization
    uint32_t magicNumber = 0;
    EEPROM.put(EEPROM_SIZE - 4, magicNumber);
    EEPROM.commit();

    Serial.println("EEPROM reset. Restarting...");
    delay(1000);
    ESP.restart();
  } else if (command == "eeprom_dump") {  // ADD THIS FOR DEBUGGING
    Serial.println("EEPROM Dump:");
    for (int i = 0; i < EEPROM_SIZE; i++) {
      if (i % 16 == 0) Serial.printf("\n%04X: ", i);
      byte b = EEPROM.read(i);
      Serial.printf("%02X ", b);
    }
    Serial.println();
  } else if (command == "sleep") {  // ADD DEEP SLEEP COMMAND
    Serial.println("Manual deep sleep command received");
    //enterDeepSleep();
  } else if (command == "sleep_disable") {  // DISABLE DEEP SLEEP
    deepSleepEnabled = false;
    Serial.println("Deep sleep disabled");
  } else if (command == "sleep_enable") {  // ENABLE DEEP SLEEP
    deepSleepEnabled = true;
    Serial.println("Deep sleep enabled");
  } else if (command == "battery") {  // ADD BATTERY COMMAND
    checkBattery();
    Serial.printf("Battery: %.2fV (%.0f%%), Status: %s\n",
                  batteryVoltage, batteryPercentage, getBatteryStatusString().c_str());
  } else if (command.length() > 0) {
    Serial.printf("Unknown command: '%s'\n", command.c_str());
    Serial.println("Available commands: mac, read, benchmark, reset, eeprom_reset, eeprom_dump, sleep, sleep_disable, sleep_enable, battery");
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    char inChar = Serial.read();

    // Handle newline characters (CR or LF)
    if (inChar == '\n' || inChar == '\r') {
      if (serialBufferIndex > 0) {
        serialCommandBuffer[serialBufferIndex] = '\0';
        String command = String(serialCommandBuffer);
        processSerialCommand(command);
        serialBufferIndex = 0;
      }
    }
    // Handle backspace
    else if (inChar == '\b' || inChar == 127) {
      if (serialBufferIndex > 0) {
        serialBufferIndex--;
        Serial.print("\b \b");  // Echo backspace
      }
    }
    // Regular characters
    else if (inChar >= 32 && inChar <= 126) {  // Printable characters
      if (serialBufferIndex < SERIAL_COMMAND_BUFFER_SIZE - 1) {
        serialCommandBuffer[serialBufferIndex] = inChar;
        serialBufferIndex++;
        Serial.print(inChar);  // Echo character
      }
    }
  }
}

void setup() {
  // 1. Initialize basic hardware first
  initLEDs();
  Serial.begin(115200);
  delay(100);  // Allow serial to stabilize

  Serial.println();
  Serial.println("=== AquaLevel Pro with RS232 Commands ===");
  Serial.println("Available commands: mac, read, benchmark, reset");
  Serial.println("===============================================");
  Serial.println(deviceName);
  Serial.println("===============================================");

  // 2. Initialize EEPROM and load config EARLY
  initializeEEPROM();
  loadConfig();  // This should happen before any WiFi operations

  // 3. Initialize battery ADC
  initBatteryADC();

  // 4. Check battery status immediately
  checkBattery();

  if (batteryVoltage < BATTERY_CRITICAL_THRESHOLD) {
    Serial.println("CRITICAL: Battery too low. Going to deep sleep immediately.");
    ESP.deepSleep(3600000000);  // 1 hour sleep
  }

  // 5. Initialize sensor serial
  sensorSerial.begin(9600);
  sensorSerial.setTimeout(50);

  // === Check for D6 hold to force AP mode ===
  pinMode(D6, INPUT_PULLUP);
  delay(10);  // Stabilization delay

  bool apFlag = (digitalRead(D6) == LOW);
  ForceAPMode(apFlag, 10);  // 10 minutes


  // === Normal Setup Continues Below ===

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);


  // 6. Try normal Wi-Fi connection
  WiFi.mode(WIFI_STA);  // Explicitly set to station mode
  if (!connectToWiFi()) {
    Serial.println("WiFi failed, starting AP mode");

    ForceAPMode(true, 10);  // 10 minutes
    //ForceAPMode(true, 0.5);  // 30 seconds

    // WiFi.mode(WIFI_AP);
    // WiFi.softAP(deviceName, WIFI_AP_PASSWORD);
    // Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    blinkRedLED(3);
  } else {
    Serial.println("WiFi connected successfully");
    blinkGreenLED(3);
  }

  // 7. OTA Setup (should be after WiFi connection)
  ArduinoOTA.setHostname(deviceName);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Update Start");
    digitalWrite(LED_RED, LED_ON);  // Indicate OTA in progress
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

  // 8. mDNS Setup
  if (MDNS.begin(deviceName)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ota", "tcp", 3232);
    Serial.println("mDNS responder started");
  }

  // 9. DNS Server Setup
  IPAddress localIP = WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP();
  dnsServer.start(DNS_PORT, "*", localIP);

  // 10. Web Server Setup
  server.on("/", handleRoot);
  server.on("/wifi-setup", handleWiFiSetup);
  server.on("/config", handleConfig);
  server.on("/data", handleData);
  server.on("/sensor-stats", handleSensorStats);
  server.on("/benchmark", handleBenchmark);
  server.on("/sleep-config", handleSleepConfig);
  server.onNotFound(handleRoot);
  server.begin();

  // 11. ESP-NOW Setup
  if (esp_now_init() != 0) {
    Serial.println("{\"error\":\"ESP-NOW init failed\"}");
    delay(1000);
    ESP.restart();
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(OnDataRecv);

  // Load MAC from EEPROM
  uint8_t storedMAC[6] = { 0 };
  EEPROM.get(ADDR_WIFI_MAC, storedMAC);
  if (storedMAC[0] || storedMAC[1] || storedMAC[2] || storedMAC[3] || storedMAC[4] || storedMAC[5]) {
    memcpy(senderMAC, storedMAC, 6);
    Serial.printf("Using stored peer MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  senderMAC[0], senderMAC[1], senderMAC[2],
                  senderMAC[3], senderMAC[4], senderMAC[5]);
  }

  if (esp_now_add_peer(senderMAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0) != 0) {
    Serial.println("{\"error\":\"Failed to add peer\"}");
  }

  // 12. Set active window start time
  activeWindowStart = millis();
  Serial.println("Active window started");

  // 13. Initial sensor operations
  Serial.printf("{\"status\":\"Ready\",\"calibration_cm\":%.1f}\n", calibration_mm / 10.0f);

  // Sensor health check
  if (checkSensorHealth()) {
    Serial.println("{\"sensor\":\"healthy\"}");
    // Get initial reading
    unsigned long initStart = millis();
    while (millis() - initStart < 5000) {  // 5 second timeout
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

  // 14. Final setup complete indication
  Serial.printf("Setup complete - %lumin active / %lumin sleep cycle\n",
                activeWindow / 60000, sleepDuration / 60000000);
  Serial.printf("Battery: %.2fV (%.0f%%), Status: %s\n",
                batteryVoltage, batteryPercentage, getBatteryStatusString().c_str());
  blinkGreenLED(2);
}


void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  dnsServer.processNextRequest();
  MDNS.update();

  handleSensorReading();  // Keep this frequent for responsive readings

  // Check active window timeout
  checkActiveWindow();

  //Check battery periodically
  static unsigned long lastBatteryCheck = 0;
  if (millis() - lastBatteryCheck >= BATTERY_READ_INTERVAL) {
    lastBatteryCheck = millis();
    checkBattery();
  }

  // Handle WiFi reconnection less frequently
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck >= 60000) {  // Every 1min
    lastWifiCheck = millis();
    Serial.printf("System health - Free heap: %d bytes, WiFi: %s, MQTT: %s\n",
                  ESP.getFreeHeap(),
                  WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                  mqttClient.connected() ? "Connected" : "Disconnected");
    // Serial.printf("Battery: %.2fV (%.0f%%), Status: %s\n",
    //               batteryVoltage, batteryPercentage, getBatteryStatusString().c_str());
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting reconnect");
      WiFi.disconnect();
      delay(100);
      connectToWiFi();
    }
  }

  // Periodic NTP sync (every hour)
  static unsigned long lastNTPSync = 0;
  if (WiFi.status() == WL_CONNECTED && millis() - lastNTPSync >= 3600000) {
    lastNTPSync = millis();
    if (timeClient.update()) {
      Serial.printf("NTP time synced: %s\n", timeClient.getFormattedTime().c_str());
    } else {
      Serial.println("NTP sync failed");
    }
  }

  // MQTT publishing with proper timing (using configurable interval)
  static unsigned long lastMQTTPublish = 0;
  if (WiFi.status() == WL_CONNECTED && millis() - lastMQTTPublish >= mqttPublishInterval) {
    lastMQTTPublish = millis();
    publishMQTTStatus();
  }

  // Handle serial commands
  handleSerialInput();

  // Watchdog feeding
  ESP.wdtFeed();

  yield();
  delay(10);
}