#include <lvgl.h>                            // lvgl version 9.2.2
#include <PINS_JC4827W543.h>                //dev device pins library
#include "TAMC_GT911.h"                     // TAMC_GT911 library
#include "BH1750.h"
#include <Arduino.h>
#include <esp_efuse.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "time.h"
#include "esp_sntp.h"
//#include <WiFiManager.h> 
#include "ArduinoOTA.h"
#include <BlynkSimpleEsp32.h>
#include <Preferences.h>
#include <math.h>
#include "ui.h"
// Add these global variables near the top of your file (after includes)
#define DUPLICATE_REJECT_MS 2000
float currentAvgWindow = 60.0;   // Default 60 seconds
float currentGustWindow = 1.0;   // Default 1 second
static bool chart_needs_init = false;
static bool chart_needs_update = false;
// Secondary I2C bus for BH1750 (pins 17=SDA, 18=SCL)
TwoWire I2C_BH1750 = TwoWire(1);   // Use I2C bus #1
BH1750 lightMeter;
float windOffsetPercent = 1.0f;   // multiplier applied to wind values (1.0 = no change)
int   brightnessOffset  = 128;      // raw value from Spinbox3 added to BH1750 result or used as manual
bool  autoBrightness    = true;   // Switch1
bool  blynkEnabled      = true;   // Switch2
bool  blynkWasEnabled   = true;   // track previous state to detect "just turned on"

// Helper: clamp int to [lo, hi]
static inline int clampInt(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
// Backlight PWM config
#define BL_PWM_FREQ       5000
#define BL_PWM_RESOLUTION 12         // 12-bit: 0-4095
#define BL_MIN            40         // Minimum brightness (avoids fully off)
#define BL_MAX            4095       // Maximum brightness
#define BL_LUX_MIN       5.0f        // Lux level that maps to BL_MIN
#define BL_LUX_MAX       1000.0f     // Lux level that maps to BL_MAX
float blGamma = 1.0f;  
bool connected = false;
unsigned long reconnectTime = 0;
static unsigned long lastManualBrightnessSet = 0;
#define BRIGHTNESS_DEBOUNCE_MS 1200   // skip auto-update for 1.2s after manual set

enum WifiStartupState {
    WIFI_STARTUP_INIT,           // Initial state - set up WiFi mode
    WIFI_STARTUP_SCAN,           // Scanning networks
    WIFI_STARTUP_CHECK_CREDS,    // Check for saved credentials
    WIFI_STARTUP_CONNECTING,     // Connecting to WiFi
    WIFI_STARTUP_COMPLETE        // Done
};

WifiStartupState wifiStartupState = WIFI_STARTUP_INIT;
unsigned long wifiStartupStepTime = 0;
String startupSSID = "";
String startupPassword = "";
// WiFi connection state
bool wifiConfigured = false;
bool wifiConnecting = false;
unsigned long wifiConnectStartTime = 0;
bool autoConnectMode = false;  // Track if we're auto-connecting with saved creds
#define WIFI_CONNECT_TIMEOUT 20000  // 20 seconds
// UDP Configuration
WiFiUDP udp;
const int UDP_PORT = 4210;

// ---- Touch Configuration ----
#define TOUCH_SDA    8
#define TOUCH_SCL    4
#define TOUCH_INT    3
#define TOUCH_RST    38
#define TOUCH_WIDTH  480
#define TOUCH_HEIGHT 272
unsigned long s1LastMsgTime = 0;
unsigned long s2LastMsgTime = 0;
#define MSG_MIN_INTERVAL 3000  
TAMC_GT911 touchController = TAMC_GT911(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TOUCH_WIDTH, TOUCH_HEIGHT);

// ========== SENSOR HISTORY STRUCTURE ==========
#define NUM_SENSORS 4
#define SENSOR_HISTORY_SIZE 500
// Track last message time for each sensor
unsigned long lastTempMsgTime[NUM_SENSORS] = {0};
unsigned long lastWindMsgTime = 0;
struct SensorReading {
    float temp;
    float hum;
    float pres;
    unsigned long timestamp;
};

SensorReading *sensorHistory[NUM_SENSORS];
int sensorHistoryIndex[NUM_SENSORS] = {0};
int sensorHistoryCount[NUM_SENSORS] = {0};
int sensorHistoryCapacity = SENSOR_HISTORY_SIZE;

// ========== CHART STATE ==========
enum ChartDataType {
    CHART_TEMP,
    CHART_HUM,
    CHART_PRES,
    CHART_WIND_SPEED
};

struct ChartState {
    ChartDataType dataType;
    int sensorIndex;
    const char* title;
    const char* units;
} currentChart;
// Add these global variables near the top with other globals
static unsigned long lastChartUpdate = 0;
static int lastChartDataCount = -1;
static ChartDataType lastChartType = CHART_TEMP;
static int lastChartSensor = -1;

// Custom scale objects for charts
static lv_obj_t * chart_scale_y = NULL;
static lv_obj_t * chart_container = NULL;
static lv_obj_t * chart_x_axis_container = NULL;
// WiFi credential message structure (for ESP-NOW) - NO ENCRYPTION
typedef struct {
  char ssid[32];
  char password[64];
  uint8_t magic[4];  // Magic bytes: 0xCA, 0xFE, 0xBA, 0xBE
} wifi_credentials_t;

// Credential request message structure - NO ENCRYPTION
typedef struct {
  uint8_t magic[4];  // Magic bytes: 0xDE, 0xAD, 0xBE, 0xEF
} credential_request_t;

// Fallback capacities and buffers if PSRAM not available
#define TEMP_FALLBACK_SIZE 128
#define TEMP_HISTORY_SIZE 500
#define WIND_HISTORY_FALLBACK_SIZE 256

enum SFEWeatherMeterKitAnemometerAngles
{
    WMK_ANGLE_0_0 = 0,
    WMK_ANGLE_22_5,
    WMK_ANGLE_45_0,
    WMK_ANGLE_67_5,
    WMK_ANGLE_90_0,
    WMK_ANGLE_112_5,
    WMK_ANGLE_135_0,
    WMK_ANGLE_157_5,
    WMK_ANGLE_180_0,
    WMK_ANGLE_202_5,
    WMK_ANGLE_225_0,
    WMK_ANGLE_247_5,
    WMK_ANGLE_270_0,
    WMK_ANGLE_292_5,
    WMK_ANGLE_315_0,
    WMK_ANGLE_337_5,
    WMK_NUM_ANGLES
};

#define SFE_WIND_VANE_DEGREES_PER_INDEX (360.0f / 16.0f)
#define SFE_WMK_ADC_ANGLE_0_0 3118
#define SFE_WMK_ADC_ANGLE_22_5 1526
#define SFE_WMK_ADC_ANGLE_45_0 1761
#define SFE_WMK_ADC_ANGLE_67_5 199
#define SFE_WMK_ADC_ANGLE_90_0 237
#define SFE_WMK_ADC_ANGLE_112_5 123
#define SFE_WMK_ADC_ANGLE_135_0 613
#define SFE_WMK_ADC_ANGLE_157_5 371
#define SFE_WMK_ADC_ANGLE_180_0 1040
#define SFE_WMK_ADC_ANGLE_202_5 859
#define SFE_WMK_ADC_ANGLE_225_0 2451
#define SFE_WMK_ADC_ANGLE_247_5 2329
#define SFE_WMK_ADC_ANGLE_270_0 3984
#define SFE_WMK_ADC_ANGLE_292_5 3290
#define SFE_WMK_ADC_ANGLE_315_0 3616
#define SFE_WMK_ADC_ANGLE_337_5 2755
#define SFE_WMK_ADC_RESOLUTION 12

struct SFEWeatherMeterKitCalibrationParams
{
    uint16_t vaneADCValues[WMK_NUM_ANGLES];
};

SFEWeatherMeterKitCalibrationParams _calibrationParams;

// Wind Rose data structures
#define WIND_DIRECTIONS 16
#define WIND_SPEED_BINS 5
#define WIND_HISTORY_SIZE 1440

struct WindDataPoint {
    float speed;
    float gust;      // ADD THIS
    float direction;
    unsigned long timestamp;
};

int *windRoseData[WIND_DIRECTIONS];
WindDataPoint *windHistory = nullptr;
int windHistoryIndex = 0;
int windHistoryCount = 0;

struct TempDataPoint {
    float temp;
    unsigned long timestamp;
};

static TempDataPoint tempHistoryFallback[TEMP_FALLBACK_SIZE];
static WindDataPoint windHistoryFallback[WIND_HISTORY_FALLBACK_SIZE];
static int windRoseFallback[WIND_DIRECTIONS][WIND_SPEED_BINS];

int tempHistoryCapacity = 0;
int windHistoryCapacity = WIND_HISTORY_SIZE;

char auth[] = "19oL8t8mImCdoUqYhfhk6DADL7540f8s";
int hours, mins, secs;
unsigned long localTimeUnix = 0; 
unsigned long s1LastUpdate = 0;
struct tm timeinfo;
bool isSetNtp = false;

TempDataPoint *tempHistory = nullptr;
int tempHistoryIndex = 0;
int tempHistoryCount = 0;
void handleWifiStartup();
// Function declarations
void cbSyncTime(struct timeval *tv);
void initSNTP();
void setTimezone();
void onESPNowReceive(const esp_now_recv_info_t * info, const uint8_t *data, int len);
void addTempToHistory(float temp);
void addTempToHistory(int deviceId, float temp, float hum, float pres);
void addSensorReading(int sensor, float temp, float hum, float pres);

void drawWindSpeedGraph();
void addWindData(float speed, float direction, float gust);
void drawWindRose();
void drawGenericChart();
void handleUDP();
SensorReading* getLatestReading(int sensor);
void setup();
void loop();
float getWindDirection(u_int16_t adcValue);
const char* getCardinalDirection(float degrees);
void consoleLog(const char* message);
//void sendSliderValuesToWindTransmitter();

void loadSettings();
void loadSettings() {
    Preferences prefs;
    prefs.begin("appsettings", true);
    autoBrightness    = prefs.getBool("autoBright", true);
    blynkEnabled      = prefs.getBool("blynkOn",    true);
    brightnessOffset  = prefs.getInt ("brightOff",  128);
    int sp4raw        = prefs.getInt ("windOffRaw", 0);
    int savedAvg      = prefs.getInt ("avgWindow",  60);
    int savedGust     = prefs.getInt ("gustWindow", 1);
    int savedGamma    = prefs.getInt ("blGamma",    10);
    prefs.end();

    blGamma           = savedGamma / 10.0f;
    windOffsetPercent = 1.0f + (sp4raw / 1000.0f);
    currentAvgWindow  = (float)savedAvg;
    currentGustWindow = (float)savedGust;
    blynkWasEnabled   = blynkEnabled;

    Serial.printf("loadSettings: autoBright=%d blynk=%d brightOff=%d windOffRaw=%d avg=%d gust=%d gamma=%d\n",
                  autoBrightness, blynkEnabled, brightnessOffset, sp4raw, savedAvg, savedGust, savedGamma);
}

// Console logging helper
void consoleLog(const char* message) {
    // Always print to serial first
    Serial.println(message);
    
    if (ui_LabelConsole == NULL) {
        return;
    }
    
    const char* currentText = lv_label_get_text(ui_LabelConsole);
    static char buffer[1024];
    
    // Add new message to existing text
    snprintf(buffer, sizeof(buffer), "%s%s\n", currentText, message);
    lv_label_set_text(ui_LabelConsole, buffer);
}


bool loadWifiCredentials(String &ssid, String &password);
// Add this function to load WiFi credentials from preferences
bool loadWifiCredentials(String &ssid, String &password) {
    Preferences preferences;
    preferences.begin("wifi", true);  // true = read-only
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");
    preferences.end();
    
    Serial.printf("loadWifiCredentials: SSID='%s', Password='%s'\n", ssid.c_str(), password.c_str());
    
    return (ssid.length() > 0);
}
static int luxToBrightness(float lux, int offset);
static int luxToBrightness(float lux, int offset) {
    float clamped = lux < BL_LUX_MIN ? BL_LUX_MIN
                  : lux > BL_LUX_MAX ? BL_LUX_MAX
                  : lux;

    // Logarithmic normalisation → 0.0 … 1.0
    float logMin  = log(BL_LUX_MIN);
    float logMax  = log(BL_LUX_MAX);
    float norm    = (log(clamped) - logMin) / (logMax - logMin);  // 0–1

    // Gamma correction — controlled by blGamma (Spinbox5 / 10)
    float curved  = powf(norm, blGamma);                          // 0–1

    int brightness = (int)(BL_MIN + (BL_MAX - BL_MIN) * curved);
    brightness    += (offset * 16);
    return brightness;
}

void checkWifiConnection();

// MODIFIED: checkWifiConnection to trigger network scan on timeout
void checkWifiConnection() {
    if (!wifiConnecting) return;
    
    unsigned long elapsed = millis() - wifiConnectStartTime;
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected successfully!");
        Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
        
        consoleLog("WiFi connected successfully!");
        
        char logBuf[128];
        snprintf(logBuf, sizeof(logBuf), "IP: %s", WiFi.localIP().toString().c_str());
        consoleLog(logBuf);
        
        snprintf(logBuf, sizeof(logBuf), "RSSI: %d dBm", WiFi.RSSI());
        consoleLog(logBuf);
        
        wifiConnecting = false;
        wifiConfigured = true;
        
        // SNTP is already initialized in setup() - just log it
        Serial.println("Time sync active");
        consoleLog("Time sync active");
        
        // Initialize Blynk if not already done
        if (!connected && blynkEnabled) {
            Blynk.config(auth, IPAddress(216, 110, 224, 105), 8080);
            Blynk.connect();
            Serial.println("Blynk connecting...");
            consoleLog("Blynk connecting...");
        }
        
        // Only auto-switch to main screen if we connected using saved credentials on boot
        if (autoConnectMode) {
            Serial.println("Auto-connect mode - switching to main screen");
            consoleLog("Loading main screen...");
            delay(1000);  // Brief delay to see the success message
            lv_scr_load(ui_ScreenMain);
            autoConnectMode = false;  // Reset flag
        }
    } 
    else if (elapsed > WIFI_CONNECT_TIMEOUT) {
        Serial.println("WiFi connection timeout!");
        Serial.printf("WiFi status: %d\n", WiFi.status());
        
        consoleLog("WiFi connection timeout!");
        consoleLog("Check credentials and try again");
        wifiConnecting = false;
        wifiConfigured = false;
        autoConnectMode = false;  // Reset flag on failure
        
        // If this was during startup, the state machine will trigger a scan
    }
    else {
        // Still connecting - show progress every 2 seconds
        static unsigned long lastDot = 0;
        if (millis() - lastDot > 2000) {
            Serial.printf("Still connecting... (status: %d, elapsed: %lu ms)\n", WiFi.status(), elapsed);
            consoleLog("...");
            lastDot = millis();
        }
    }
}


// WiFi functions with C linkage (called from C files)
extern "C" {
    void applySettingsToUI(void);
    void populateWifiList();
    void SaveWifiFromUI(lv_event_t * e);
    void updateSettingsWifiInfo();
    //void sendSliderValuesToWindTransmitter();
    void saveSettings();
}

extern "C" void applySettingsToUI() {
    if (ui_ScreenSettings == NULL) return;

    if (ui_Switch1 != NULL) {
        if (autoBrightness) lv_obj_add_state(ui_Switch1, LV_STATE_CHECKED);
        else                lv_obj_remove_state(ui_Switch1, LV_STATE_CHECKED);
    }
    if (ui_Switch2 != NULL) {
        if (blynkEnabled) lv_obj_add_state(ui_Switch2, LV_STATE_CHECKED);
        else              lv_obj_remove_state(ui_Switch2, LV_STATE_CHECKED);
    }
    if (ui_Spinbox1 != NULL) lv_spinbox_set_value(ui_Spinbox1, (int32_t)currentAvgWindow);
    if (ui_Spinbox2 != NULL) lv_spinbox_set_value(ui_Spinbox2, (int32_t)currentGustWindow);
    if (ui_Spinbox3 != NULL) lv_spinbox_set_value(ui_Spinbox3, brightnessOffset);
    if (ui_Spinbox4 != NULL) lv_spinbox_set_value(ui_Spinbox4, (int32_t)((windOffsetPercent - 1.0f) * 1000.0f));
    if (ui_Spinbox5 != NULL) lv_spinbox_set_value(ui_Spinbox5, (int32_t)(blGamma * 10.0f));
    Serial.println("Settings applied to UI");
    Serial.printf("autoBrightness=%d blynkEnabled=%d avgWindow=%.1f gustWindow=%.1f brightOffset=%d windOffsetRaw=%d blGamma=%d\n",
                  autoBrightness, blynkEnabled, currentAvgWindow, currentGustWindow, brightnessOffset, (int32_t)((windOffsetPercent - 1.0f) * 1000.0f), (int32_t)(blGamma * 10.0f));
}

extern "C" void updateBacklightLive(bool autoMode, int offset) {
    autoBrightness   = autoMode;
    brightnessOffset = offset;
    int brightness;
    if (autoMode) {
        float lux = lightMeter.measurementReady() ? lightMeter.readLightLevel() : 0.0f;
        if (lux < 0) lux = 0;
        brightness = (lux > 0) ? luxToBrightness(lux, offset)
                                : clampInt(offset * 16, 0, BL_MAX);
    } else {
        brightness = offset * 16;
    }
    lastManualBrightnessSet = millis();
    ledcWrite(GFX_BL, clampInt(brightness, 0, BL_MAX));
}

extern "C" void saveSettings() {
    if (ui_Switch1  == NULL || ui_Switch2  == NULL ||
        ui_Spinbox3 == NULL || ui_Spinbox4 == NULL ||
        ui_Spinbox1 == NULL || ui_Spinbox2 == NULL) return;

    bool newAutoBright  = lv_obj_has_state(ui_Switch1, LV_STATE_CHECKED);
    bool newBlynk       = lv_obj_has_state(ui_Switch2, LV_STATE_CHECKED);
    int  newBrightOff   = (int)lv_spinbox_get_value(ui_Spinbox3);
    int  newWindOffRaw  = (int)lv_spinbox_get_value(ui_Spinbox4);
    int  newAvgWindow   = (int)lv_spinbox_get_value(ui_Spinbox1);
    int  newGustWindow  = (int)lv_spinbox_get_value(ui_Spinbox2);
    int  newGammaRaw  =   (int)lv_spinbox_get_value(ui_Spinbox5);
    Preferences prefs;
    prefs.begin("appsettings", false);
    prefs.putBool("autoBright", newAutoBright);
    prefs.putBool("blynkOn",    newBlynk);
    prefs.putInt ("brightOff",  newBrightOff);
    prefs.putInt ("windOffRaw", newWindOffRaw);
    prefs.putInt ("avgWindow",  newAvgWindow);
    prefs.putInt ("gustWindow", newGustWindow);
    prefs.putInt ("blGamma",    newGammaRaw);
    prefs.end();

    // Apply immediately
    autoBrightness   = newAutoBright;
    brightnessOffset = newBrightOff;
    windOffsetPercent = 1.0f + (newWindOffRaw / 1000.0f);
    currentAvgWindow  = newAvgWindow;
    currentGustWindow = newGustWindow;
    blGamma = newGammaRaw / 10.0f;
    // Handle Blynk enable/disable transition
    bool wasEnabled = blynkEnabled;
    blynkEnabled    = newBlynk;

    if (blynkEnabled && !wasEnabled) {
        Serial.println("Blynk re-enabled by user — connecting");
        consoleLog("Blynk enabled — connecting...");
        Blynk.config(auth, IPAddress(216, 110, 224, 105), 8080);
        if (WiFi.status() == WL_CONNECTED) Blynk.connect();
    } else if (!blynkEnabled && wasEnabled) {
        Serial.println("Blynk disabled by user");
        consoleLog("Blynk disabled");
        Blynk.disconnect();
    }

    Serial.printf("saveSettings: autoBright=%d blynk=%d brightOff=%d windOffRaw=%d avgWin=%d gustWin=%d gamma=%f\n",
                  autoBrightness, blynkEnabled, brightnessOffset, newWindOffRaw, newAvgWindow, newGustWindow, blGamma);
    consoleLog("Settings saved.");
}


extern "C" void sendSliderValuesToWindTransmitter() {
    if (!wifiConfigured || WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot send slider values - WiFi not connected");
        return;
    }

    // Read directly from spinboxes so we always send current UI values
    // even if the user hasn't pressed Save yet
    float avgVal  = (ui_Spinbox1 != NULL) ? (float)lv_spinbox_get_value(ui_Spinbox1) : currentAvgWindow;
    float gustVal = (ui_Spinbox2 != NULL) ? (float)lv_spinbox_get_value(ui_Spinbox2) : currentGustWindow;

    typedef struct {
        float sliderAvg;
        float sliderGust;
        uint8_t magic[4];
    } slider_values_t;

    slider_values_t sliders;
    sliders.sliderAvg  = avgVal;
    sliders.sliderGust = gustVal;
    sliders.magic[0] = 0xAB;
    sliders.magic[1] = 0xCD;
    sliders.magic[2] = 0xEF;
    sliders.magic[3] = 0x12;

    IPAddress broadcastIP = WiFi.localIP();
    broadcastIP[3] = 255;

    udp.beginPacket(broadcastIP, UDP_PORT);
    udp.write((uint8_t *)&sliders, sizeof(sliders));
    udp.endPacket();

    Serial.printf("Sent slider values via UDP - Avg: %.1f, Gust: %.1f\n", avgVal, gustVal);

    char logBuf[128];
    snprintf(logBuf, sizeof(logBuf), "Sent: Avg=%.1fs, Gust=%.1fs", avgVal, gustVal);
    consoleLog(logBuf);
}

// ========== UPDATE SETTINGS SCREEN WIFI INFO ==========
extern "C" void updateSettingsWifiInfo() {
    if (ui_LabelConnectedTo == NULL || ui_Label9 == NULL || ui_LabelIP2 == NULL) {
        return;
    }
    
    char buf[128];
    
    // Update SSID label
    if (WiFi.status() == WL_CONNECTED) {
        String ssid = WiFi.SSID();
        snprintf(buf, sizeof(buf), "%s", ssid.c_str());
        lv_label_set_text(ui_LabelConnectedTo, buf);
        
        // Update RSSI label
        int rssi = WiFi.RSSI();
        snprintf(buf, sizeof(buf), "RSSI: %d dB", rssi);
        lv_label_set_text(ui_Label9, buf);
        
        // Update IP address label
        IPAddress ip = WiFi.localIP();
        snprintf(buf, sizeof(buf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        lv_label_set_text(ui_LabelIP2, buf);
    } else {
        lv_label_set_text(ui_LabelConnectedTo, "Not Connected");
        lv_label_set_text(ui_Label9, "RSSI: -- dB");
        lv_label_set_text(ui_LabelIP2, "0.0.0.0");
    }
}


const char* getCardinalDirection(float degrees) {
    while (degrees < 0) degrees += 360;
    while (degrees >= 360) degrees -= 360;
    
    if (degrees < 11.25) return "N";
    else if (degrees < 33.75) return "NNE";
    else if (degrees < 56.25) return "NE";
    else if (degrees < 78.75) return "ENE";
    else if (degrees < 101.25) return "E";
    else if (degrees < 123.75) return "ESE";
    else if (degrees < 146.25) return "SE";
    else if (degrees < 168.75) return "SSE";
    else if (degrees < 191.25) return "S";
    else if (degrees < 213.75) return "SSW";
    else if (degrees < 236.25) return "SW";
    else if (degrees < 258.75) return "WSW";
    else if (degrees < 281.25) return "W";
    else if (degrees < 303.75) return "WNW";
    else if (degrees < 326.25) return "NW";
    else if (degrees < 348.75) return "NNW";
    else return "N";
}

lv_obj_t * temp_chart = NULL;
lv_chart_series_t * temp_chart_series = NULL;
lv_obj_t * temp_scale_y = NULL;
lv_obj_t * temp_container = NULL;
static lv_obj_t * temp_label_popup = NULL;

// Chart label popup for generic chart
static lv_obj_t * chart_label_popup = NULL;
static void chart_draw_label_cb(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * chart = (lv_obj_t*) lv_event_get_target(e);
  
  if(code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING) {
    int32_t id = lv_chart_get_pressed_point(chart);
    
    if(id == LV_CHART_POINT_NONE) {
      if(chart_label_popup != NULL) {
        lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    // Special handling for wind speed chart
    if (currentChart.dataType == CHART_WIND_SPEED) {
      if (windHistory == nullptr || windHistoryCount == 0) {
        if(chart_label_popup != NULL) {
          lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
        }
        return;
      }
      
      int chartPoints = lv_chart_get_point_count(chart);
      
      if (id < 0 || id >= chartPoints) {
        if(chart_label_popup != NULL) {
          lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
        }
        return;
      }
      
      int startIdx = (windHistoryIndex - windHistoryCount + windHistoryCapacity) % windHistoryCapacity;
      int historyIdx;
      
      if (windHistoryCount <= chartPoints) {
        historyIdx = id;
      } else {
        if (chartPoints <= 1) {
          historyIdx = 0;
        } else {
          historyIdx = (id * (windHistoryCount - 1)) / (chartPoints - 1);
        }
      }
      
      if (historyIdx < 0 || historyIdx >= windHistoryCount) {
        if(chart_label_popup != NULL) {
          lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
        }
        return;
      }
      
      int idx = (startIdx + historyIdx) % windHistoryCapacity;
      
      if (idx < 0 || idx >= windHistoryCapacity) {
        if(chart_label_popup != NULL) {
          lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
        }
        return;
      }
      
      if (windHistory[idx].timestamp == 0) {
        if(chart_label_popup != NULL) {
          lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
        }
        return;
      }
      
      float speed = windHistory[idx].speed;
      float gust = windHistory[idx].gust;
      time_t timestamp = (time_t)(windHistory[idx].timestamp / 1000);  // Convert millis to seconds
      
      // Create label popup if it doesn't exist
      if(chart_label_popup == NULL) {
        chart_label_popup = lv_label_create(lv_scr_act());
        if (chart_label_popup == NULL) {
          return;
        }
        lv_obj_set_style_bg_color(chart_label_popup, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(chart_label_popup, LV_OPA_80, 0);
        lv_obj_set_style_text_color(chart_label_popup, lv_color_white(), 0);
        lv_obj_set_style_pad_all(chart_label_popup, 8, 0);
        lv_obj_set_style_radius(chart_label_popup, 4, 0);
        lv_obj_set_style_border_width(chart_label_popup, 1, 0);
        lv_obj_set_style_border_color(chart_label_popup, lv_color_white(), 0);
        lv_obj_set_style_border_opa(chart_label_popup, LV_OPA_50, 0);
        lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_FLOATING);
      }
      
      // Format the label text
      static char buf[64];
      if (timestamp > 0 && timestamp >= 1000000000) {
        struct tm timeinfo_buf;
        struct tm* timeinfo = localtime_r(&timestamp, &timeinfo_buf);
        
        if (timeinfo != NULL) {
          int hour = timeinfo->tm_hour;
          int min = timeinfo->tm_min;
          const char* ampm = (hour >= 12) ? "PM" : "AM";
          if (hour > 12) hour -= 12;
          else if (hour == 0) hour = 12;
          snprintf(buf, sizeof(buf), "%.1f kph\nGust: %.1f kph\n%d:%02d %s", speed, gust, hour, min, ampm);
        } else {
          snprintf(buf, sizeof(buf), "%.1f kph\nGust: %.1f kph", speed, gust);
        }
      } else {
        snprintf(buf, sizeof(buf), "%.1f kph\nGust: %.1f kph", speed, gust);
      }
      
      lv_label_set_text(chart_label_popup, buf);
      
      // Get point position on chart
      lv_chart_series_t * ser = lv_chart_get_series_next(chart, NULL);
      if(ser == NULL) {
        return;
      }
      
      lv_point_t p;
      lv_chart_get_point_pos_by_id(chart, ser, id, &p);
      
      lv_area_t chart_coords;
      lv_obj_get_coords(chart, &chart_coords);
      
      // Position label above the data point
      lv_coord_t label_x = chart_coords.x1 + p.x;
      lv_coord_t label_y = chart_coords.y1 + p.y - 45;
      
      lv_obj_update_layout(chart_label_popup);
      lv_coord_t label_width = lv_obj_get_width(chart_label_popup);
      
      lv_obj_set_pos(chart_label_popup, label_x - (label_width / 2), label_y);
      lv_obj_clear_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
      
      return;
    }
    
    // Original code for sensor data (temp/hum/pres)
    int sensor = currentChart.sensorIndex;
    if(sensor < 0 || sensor >= NUM_SENSORS || 
       sensorHistory[sensor] == nullptr || sensorHistoryCount[sensor] == 0) {
      if(chart_label_popup != NULL) {
        lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    int chartPoints = lv_chart_get_point_count(chart);
    
    if (id < 0 || id >= chartPoints) {
      if(chart_label_popup != NULL) {
        lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    int startIdx = (sensorHistoryIndex[sensor] - sensorHistoryCount[sensor] + sensorHistoryCapacity) % sensorHistoryCapacity;
    int historyIdx;
    
    if (sensorHistoryCount[sensor] <= chartPoints) {
      historyIdx = id;
    } else {
      if (chartPoints <= 1) {
        historyIdx = 0;
      } else {
        historyIdx = (id * (sensorHistoryCount[sensor] - 1)) / (chartPoints - 1);
      }
    }
    
    if (historyIdx < 0 || historyIdx >= sensorHistoryCount[sensor]) {
      if(chart_label_popup != NULL) {
        lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    int idx = (startIdx + historyIdx) % sensorHistoryCapacity;
    
    if (idx < 0 || idx >= sensorHistoryCapacity) {
      if(chart_label_popup != NULL) {
        lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    if (sensorHistory[sensor][idx].timestamp == 0) {
      if(chart_label_popup != NULL) {
        lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    // Get the value based on chart type
    float value = 0;
    const char* unit = "";
    switch(currentChart.dataType) {
      case CHART_TEMP:
        value = sensorHistory[sensor][idx].temp;
        unit = "°C";
        break;
      case CHART_HUM:
        value = sensorHistory[sensor][idx].hum;
        unit = "%";
        break;
      case CHART_PRES:
        value = sensorHistory[sensor][idx].pres;
        unit = "mbar";
        break;
      default:
        value = 0;
        unit = "";
    }
    
    time_t timestamp = (time_t)sensorHistory[sensor][idx].timestamp;
    
    // Create label popup if it doesn't exist
    if(chart_label_popup == NULL) {
      chart_label_popup = lv_label_create(lv_scr_act());
      if (chart_label_popup == NULL) {
        return;
      }
      lv_obj_set_style_bg_color(chart_label_popup, lv_color_black(), 0);
      lv_obj_set_style_bg_opa(chart_label_popup, LV_OPA_80, 0);
      lv_obj_set_style_text_color(chart_label_popup, lv_color_white(), 0);
      lv_obj_set_style_pad_all(chart_label_popup, 8, 0);
      lv_obj_set_style_radius(chart_label_popup, 4, 0);
      lv_obj_set_style_border_width(chart_label_popup, 1, 0);
      lv_obj_set_style_border_color(chart_label_popup, lv_color_white(), 0);
      lv_obj_set_style_border_opa(chart_label_popup, LV_OPA_50, 0);
      lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_FLOATING);
    }
    
    // Format the label text
    static char buf[64];
    if (timestamp > 0 && timestamp >= 1000000000) {
      struct tm timeinfo_buf;
      struct tm* timeinfo = localtime_r(&timestamp, &timeinfo_buf);
      
      if (timeinfo != NULL) {
        int hour = timeinfo->tm_hour;
        int min = timeinfo->tm_min;
        const char* ampm = (hour >= 12) ? "PM" : "AM";
        if (hour > 12) hour -= 12;
        else if (hour == 0) hour = 12;
        snprintf(buf, sizeof(buf), "%.2f%s\n%d:%02d %s", value, unit, hour, min, ampm);
      } else {
        snprintf(buf, sizeof(buf), "%.2f%s", value, unit);
      }
    } else {
      snprintf(buf, sizeof(buf), "%.2f%s", value, unit);
    }
    
    lv_label_set_text(chart_label_popup, buf);
    
    // Get point position on chart
    lv_chart_series_t * ser = lv_chart_get_series_next(chart, NULL);
    if(ser == NULL) {
      return;
    }
    
    lv_point_t p;
    lv_chart_get_point_pos_by_id(chart, ser, id, &p);
    
    lv_area_t chart_coords;
    lv_obj_get_coords(chart, &chart_coords);
    
    // Position label above the data point
    lv_coord_t label_x = chart_coords.x1 + p.x;
    lv_coord_t label_y = chart_coords.y1 + p.y - 45;
    
    lv_obj_update_layout(chart_label_popup);
    lv_coord_t label_width = lv_obj_get_width(chart_label_popup);
    
    lv_obj_set_pos(chart_label_popup, label_x - (label_width / 2), label_y);
    lv_obj_clear_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
  }
  else if(code == LV_EVENT_RELEASED) {
    if(chart_label_popup != NULL) {
      lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
    }
  }
  else if(code == LV_EVENT_VALUE_CHANGED) {
    lv_obj_invalidate(chart);
  }
}


static void temp_chart_draw_label_cb(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * chart = (lv_obj_t*) lv_event_get_target(e);
  
  if(code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING) {
    int32_t id = lv_chart_get_pressed_point(chart);
    
    if(id == LV_CHART_POINT_NONE || tempHistory == nullptr || tempHistoryCount == 0 || temp_chart == NULL) {
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    int chartPoints = lv_chart_get_point_count(temp_chart);
    
    if (id < 0 || id >= chartPoints) {
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    int expectedPoints = tempHistoryCount;
    if (expectedPoints > 100) expectedPoints = 100;
    if (expectedPoints < 2) expectedPoints = 2;
    
    if (chartPoints != expectedPoints) {
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    int startIdx = (tempHistoryIndex - tempHistoryCount + tempHistoryCapacity) % tempHistoryCapacity;
    int historyIdx;
    
    if (tempHistoryCount <= chartPoints) {
      historyIdx = id;
    } else {
      if (chartPoints <= 1) {
        historyIdx = 0;
      } else {
        historyIdx = (id * (tempHistoryCount - 1)) / (chartPoints - 1);
      }
    }
    
    if (historyIdx < 0 || historyIdx >= tempHistoryCount) {
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    int idx = (startIdx + historyIdx) % tempHistoryCapacity;
    
    if (idx < 0 || idx >= tempHistoryCapacity) {
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    if (tempHistory[idx].timestamp == 0) {
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    float temp_value = tempHistory[idx].temp;
    time_t timestamp = (time_t)tempHistory[idx].timestamp;
    
    if(temp_label_popup == NULL) {
      temp_label_popup = lv_label_create(lv_scr_act());
      if (temp_label_popup == NULL) {
        return;
      }
      lv_obj_set_style_bg_color(temp_label_popup, lv_color_black(), 0);
      lv_obj_set_style_bg_opa(temp_label_popup, LV_OPA_80, 0);
      lv_obj_set_style_text_color(temp_label_popup, lv_color_white(), 0);
      lv_obj_set_style_pad_all(temp_label_popup, 8, 0);
      lv_obj_set_style_radius(temp_label_popup, 4, 0);
      lv_obj_set_style_border_width(temp_label_popup, 1, 0);
      lv_obj_set_style_border_color(temp_label_popup, lv_color_white(), 0);
      lv_obj_set_style_border_opa(temp_label_popup, LV_OPA_50, 0);
      lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_FLOATING);
    }
    
    static char buf[64];
    if (timestamp > 0 && timestamp >= 1000000000) {
      struct tm timeinfo_buf;
      struct tm* timeinfo = localtime_r(&timestamp, &timeinfo_buf);
      
      if (timeinfo != NULL) {
        int hour = timeinfo->tm_hour;
        int min = timeinfo->tm_min;
        const char* ampm = (hour >= 12) ? "PM" : "AM";
        if (hour > 12) hour -= 12;
        else if (hour == 0) hour = 12;
        snprintf(buf, sizeof(buf), "%.2f°C\n%d:%02d %s", temp_value, hour, min, ampm);
      } else {
        snprintf(buf, sizeof(buf), "%.2f°C", temp_value);
      }
    } else {
      snprintf(buf, sizeof(buf), "%.2f°C", temp_value);
    }
    
    lv_label_set_text(temp_label_popup, buf);
    
    lv_chart_series_t * ser = lv_chart_get_series_next(chart, NULL);
    if(ser == NULL) {
      return;
    }
    
    lv_point_t p;
    lv_chart_get_point_pos_by_id(chart, ser, id, &p);
    
    lv_area_t chart_coords;
    lv_obj_get_coords(chart, &chart_coords);
    
    lv_coord_t label_x = chart_coords.x1 + p.x;
    lv_coord_t label_y = chart_coords.y1 + p.y - 45;
    
    lv_obj_update_layout(temp_label_popup);
    lv_coord_t label_width = lv_obj_get_width(temp_label_popup);
    
    lv_obj_set_pos(temp_label_popup, label_x - (label_width / 2), label_y);
    lv_obj_clear_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
  }
  else if(code == LV_EVENT_RELEASED) {
    if(temp_label_popup != NULL) {
      lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
    }
  }
  else if(code == LV_EVENT_VALUE_CHANGED) {
    lv_obj_invalidate(chart);
  }
}

float s2instwind = 999;
float s2temp, s2windgust, s2avgwind, s2winddirV;
unsigned long s2LastUpdate = 0;

// Removed ESP-NOW queue and message handling - data now comes via UDP

float getWindDirection(u_int16_t adcValue)
{
    uint16_t rawADC = adcValue;
    int16_t closestDifference = 32767;
    uint8_t closestIndex = 0;
    for (uint8_t i = 0; i < WMK_NUM_ANGLES; i++)
    {
        int16_t adcDifference = _calibrationParams.vaneADCValues[i] - rawADC;
        adcDifference = abs(adcDifference);
        if (adcDifference < closestDifference)
        {
            closestDifference = adcDifference;
            closestIndex = i;
        }
    }
    float direction = (float)closestIndex * SFE_WIND_VANE_DEGREES_PER_INDEX;
    return direction;
}

typedef struct {
  float temp;
  float hum;
  float presread;
  float volts0;
} esp_now_payload_t;

typedef struct {
  float temp;
  float windgust;
  float avgwind;
  float instwind;
  float winddirV;
} esp_now_payload_wind_t;

float s1temp = 999;
float s1hum, s1pres, s1vBat;

// Data now received via UDP, not ESP-NOW

#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))

    
// Update addWindData function signature and implementation
void addWindData(float speed, float direction, float gust) {
    if (windHistory == nullptr) return;

    // Apply wind offset multiplier
    float adjSpeed = speed * windOffsetPercent;
    float adjGust  = gust  * windOffsetPercent;

    windHistory[windHistoryIndex].speed     = adjSpeed;
    windHistory[windHistoryIndex].gust      = adjGust;
    windHistory[windHistoryIndex].direction = direction;
    windHistory[windHistoryIndex].timestamp = millis();

    windHistoryIndex = (windHistoryIndex + 1) % windHistoryCapacity;
    if (windHistoryCount < windHistoryCapacity) windHistoryCount++;

    // Rebuild wind rose from history
    for (int i = 0; i < WIND_DIRECTIONS; i++) {
        if (windRoseData[i] != nullptr)
            for (int j = 0; j < WIND_SPEED_BINS; j++)
                windRoseData[i][j] = 0;
    }

    int startIdx = (windHistoryIndex - windHistoryCount + windHistoryCapacity) % windHistoryCapacity;
    for (int i = 0; i < windHistoryCount; i++) {
        int idx   = (startIdx + i) % windHistoryCapacity;
        float spd = windHistory[idx].gust;
        float dir = windHistory[idx].direction;
        int dirBin = (int)((dir + 11.25f) / 22.5f) % WIND_DIRECTIONS;
        int spdBin = (spd < 5) ? 0 : (spd < 10) ? 1 : (spd < 20) ? 2 : (spd < 30) ? 3 : 4;
        if (windRoseData[dirBin] != nullptr)
            windRoseData[dirBin][spdBin]++;
    }
}


void cbSyncTime(struct timeval *tv) {
  Serial.println("NTP time synched");
  getLocalTime(&timeinfo);
  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  Serial.println(asctime(timeinfo));
  time_t now = time(nullptr);
  localTimeUnix = static_cast<uint32_t>(now);
  isSetNtp = true;
}

void initSNTP() {  
  sntp_set_sync_interval(10 * 60 * 1000UL);
  sntp_set_time_sync_notification_cb(cbSyncTime);
  esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "216.110.224.105");
  esp_sntp_init();
  setTimezone();
}

void setTimezone() {  
  setenv("TZ","EST5EDT,M3.2.0,M11.1.0",1);
  tzset();
}

void addTempToHistory(float temp) {
  if (tempHistory == nullptr) return;
  
  time_t now = time(nullptr);
  tempHistory[tempHistoryIndex].temp = temp;
  tempHistory[tempHistoryIndex].timestamp = (unsigned long)now;
  
  tempHistoryIndex = (tempHistoryIndex + 1) % tempHistoryCapacity;
  if (tempHistoryCount < tempHistoryCapacity) {
    tempHistoryCount++;
  }
}

void addSensorReading(int sensor, float temp, float hum, float pres) {
    if (sensor < 0 || sensor >= NUM_SENSORS) return;
    if (sensorHistory[sensor] == nullptr) return;
    
    time_t now = time(nullptr);
    sensorHistory[sensor][sensorHistoryIndex[sensor]].temp = temp;
    sensorHistory[sensor][sensorHistoryIndex[sensor]].hum = hum;
    sensorHistory[sensor][sensorHistoryIndex[sensor]].pres = pres;
    sensorHistory[sensor][sensorHistoryIndex[sensor]].timestamp = (unsigned long)now;
    
    sensorHistoryIndex[sensor] = (sensorHistoryIndex[sensor] + 1) % sensorHistoryCapacity;
    if (sensorHistoryCount[sensor] < sensorHistoryCapacity) {
        sensorHistoryCount[sensor]++;
    }
}

void addTempToHistory(int deviceId, float temp, float hum, float pres) {
    if (deviceId < 0 || deviceId >= NUM_SENSORS) return;
    
    addSensorReading(deviceId, temp, hum, pres);
    
    // Also add to legacy temp history for backward compatibility
    if (deviceId == 0) {
        addTempToHistory(temp);
    }
}

SensorReading* getLatestReading(int sensor) {
    if (sensor < 0 || sensor >= NUM_SENSORS) return nullptr;
    if (sensorHistory[sensor] == nullptr) return nullptr;
    if (sensorHistoryCount[sensor] == 0) return nullptr;
    int idx = (sensorHistoryIndex[sensor] - 1 + sensorHistoryCapacity) % sensorHistoryCapacity;
    return &sensorHistory[sensor][idx];
}
void cleanupChartContainers() ;

void cleanupChartContainers() {
    if (chart_container != NULL) {
        Serial.println("Hiding chart containers (not deleting)");
        
        // Just hide them instead of deleting
        lv_obj_add_flag(chart_container, LV_OBJ_FLAG_HIDDEN);
        if (chart_x_axis_container != NULL) {
            lv_obj_add_flag(chart_x_axis_container, LV_OBJ_FLAG_HIDDEN);
        }
        if (chart_label_popup != NULL) {
            lv_obj_add_flag(chart_label_popup, LV_OBJ_FLAG_HIDDEN);
        }
        
        // DON'T delete, DON'T null the pointers
        // Just mark for rebuild next time
        lastChartUpdate = 0;
        lastChartDataCount = -1;
    }
}


void drawGenericChart() {
    if (ui_Chart1 == NULL) {
        Serial.println("ERROR: ui_Chart1 is NULL");
        return;
    }
    
    lv_obj_t * screen = lv_screen_active();
    if (screen != ui_ScreenChart) {
        return;
    }
    
    // Add event callback for tooltips on first call
    static bool event_added = false;
    if (!event_added) {
        lv_obj_add_event_cb(ui_Chart1, chart_draw_label_cb, LV_EVENT_ALL, NULL);
        event_added = true;
    }
    
    // Check if we need to redraw
    int currentDataCount = 0;
    if (currentChart.dataType == CHART_WIND_SPEED) {
        currentDataCount = windHistoryCount;
    } else {
        int sensor = currentChart.sensorIndex;
        if (sensor >= 0 && sensor < NUM_SENSORS) {
            currentDataCount = sensorHistoryCount[sensor];
        }
    }
    
    bool needsRedraw = false;
    bool needsFullRebuild = false;
    
    if (chart_container == NULL) {
        needsRedraw = true;
        needsFullRebuild = true;
        Serial.println("First draw - creating containers");
        lastChartType = currentChart.dataType;
        lastChartSensor = currentChart.sensorIndex;
    }
    else if (lastChartType != currentChart.dataType || lastChartSensor != currentChart.sensorIndex) {
        needsRedraw = true;
        needsFullRebuild = true;
        Serial.println("Chart type/sensor changed - full rebuild");
        lastChartType = currentChart.dataType;
        lastChartSensor = currentChart.sensorIndex;
    }
    else if (lastChartDataCount != currentDataCount) {
        needsRedraw = true;
        Serial.printf("Data count changed: %d -> %d\n", lastChartDataCount, currentDataCount);
    }
    else if (millis() - lastChartUpdate > 5000) {
        needsRedraw = true;
        Serial.println("Periodic update (5s)");
    }
    
    if (!needsRedraw) {
        return;
    }
    
    lastChartUpdate = millis();
    lastChartDataCount = currentDataCount;
    
    Serial.printf("Drawing chart: type=%d, sensor=%d\n", currentChart.dataType, currentChart.sensorIndex);
    
    // Clean up containers if rebuilding
    if (needsFullRebuild && chart_container != NULL) {
        Serial.println("Deleting old containers for rebuild");
        lv_obj_del(chart_container);
        chart_container = NULL;
        chart_scale_y = NULL;
        chart_x_axis_container = NULL;
        
        // CRITICAL: Give LVGL time to process deletions
        for (int i = 0; i < 3; i++) {
            lv_timer_handler();
            delay(10);
        }
    }
    
    // Create containers if needed
    if (chart_container == NULL) {
        Serial.println("Creating chart scale containers");
        
        lv_obj_update_layout(screen);
        lv_obj_update_layout(ui_Chart1);
        
        lv_area_t chart_area;
        lv_obj_get_coords(ui_Chart1, &chart_area);
        
        lv_coord_t chart_x = chart_area.x1;
        lv_coord_t chart_y = chart_area.y1;
        lv_coord_t chart_w = lv_area_get_width(&chart_area);
        lv_coord_t chart_h = lv_area_get_height(&chart_area);
        
        Serial.printf("Chart position: x=%d, y=%d, w=%d, h=%d\n", chart_x, chart_y, chart_w, chart_h);
        
        if (chart_w <= 0 || chart_h <= 0 || chart_w < 100 || chart_h < 100) {
            Serial.println("ERROR: Chart has invalid size - skipping");
            return;
        }
        
        chart_container = lv_obj_create(screen);
        lv_obj_set_size(chart_container, 50, chart_h);
        lv_obj_set_pos(chart_container, chart_x - 55, chart_y);
        lv_obj_clear_flag(chart_container, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(chart_container, 0, 0);
        lv_obj_set_style_border_width(chart_container, 0, 0);
        lv_obj_set_style_bg_opa(chart_container, LV_OPA_TRANSP, 0);
        lv_obj_move_foreground(chart_container);
        
        chart_scale_y = lv_scale_create(chart_container);
        lv_obj_set_size(chart_scale_y, 50, chart_h);
        lv_obj_align(chart_scale_y, LV_ALIGN_LEFT_MID, 0, 0);
        lv_scale_set_mode(chart_scale_y, LV_SCALE_MODE_VERTICAL_LEFT);
        lv_scale_set_label_show(chart_scale_y, true);
        lv_obj_set_style_length(chart_scale_y, 5, LV_PART_ITEMS);
        lv_obj_set_style_length(chart_scale_y, 10, LV_PART_INDICATOR);
        lv_obj_set_style_text_font(chart_scale_y, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(chart_scale_y, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_color(chart_scale_y, lv_color_white(), LV_PART_INDICATOR);
        lv_obj_set_style_text_color(chart_scale_y, lv_color_white(), LV_PART_ITEMS);
        lv_obj_set_style_pad_all(chart_scale_y, 0, 0);
        
        chart_x_axis_container = lv_obj_create(screen);
        lv_obj_set_size(chart_x_axis_container, chart_w, 25);
        lv_obj_set_pos(chart_x_axis_container, chart_x, chart_y + chart_h + 5);
        lv_obj_set_flex_flow(chart_x_axis_container, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(chart_x_axis_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(chart_x_axis_container, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(chart_x_axis_container, 0, 0);
        lv_obj_set_style_pad_left(chart_x_axis_container, 3, 0);
        lv_obj_set_style_pad_right(chart_x_axis_container, 3, 0);
        lv_obj_set_style_border_width(chart_x_axis_container, 0, 0);
        lv_obj_set_style_bg_opa(chart_x_axis_container, LV_OPA_TRANSP, 0);
        lv_obj_move_foreground(chart_x_axis_container);
        
        for (int i = 0; i < 5; i++) {
            lv_obj_t * time_label = lv_label_create(chart_x_axis_container);
            lv_label_set_text(time_label, "--:--");
            lv_obj_set_style_text_font(time_label, &lv_font_montserrat_8, 0);
            lv_obj_set_style_text_color(time_label, lv_color_white(), 0);
            lv_obj_set_style_pad_all(time_label, 0, 0);
        }
        
        Serial.println("Containers created");
    } else {
    // Containers exist but might be hidden - show them
    Serial.println("Showing existing containers");
    lv_obj_clear_flag(chart_container, LV_OBJ_FLAG_HIDDEN);
    if (chart_x_axis_container != NULL) {
        lv_obj_clear_flag(chart_x_axis_container, LV_OBJ_FLAG_HIDDEN);
    }
  }
    
    // Only remove series if chart type or sensor changed
    static bool seriesInitialized = false;
    if (needsFullRebuild) {
        Serial.println("Removing old series");
        lv_chart_series_t * ser;
        while ((ser = lv_chart_get_series_next(ui_Chart1, NULL)) != NULL) {
            lv_chart_remove_series(ui_Chart1, ser);
        }
        seriesInitialized = false;
    }
    
    // === WIND SPEED CHART ===
    if (currentChart.dataType == CHART_WIND_SPEED) {
        if (windHistory == nullptr || windHistoryCount == 0) {
            Serial.println("No wind history data available");
            lv_label_set_text(ui_LabelChart, "Wind Speed & Gusts - No Data");
            return;
        }
        
        lv_label_set_text(ui_LabelChart, "Wind Speed & Gusts");
        
        int displayPoints = windHistoryCount;
        if (displayPoints > 100) displayPoints = 100;
        if (displayPoints < 2) displayPoints = 2;
        
        Serial.printf("Wind chart: displayPoints=%d, historyCount=%d\n", displayPoints, windHistoryCount);
        
        lv_chart_set_point_count(ui_Chart1, displayPoints);
        
        float minVal = 999.0;
        float maxVal = -999.0;
        int startIdx = (windHistoryIndex - windHistoryCount + windHistoryCapacity) % windHistoryCapacity;
        
        for (int i = 0; i < windHistoryCount; i++) {
            int idx = (startIdx + i) % windHistoryCapacity;
            float speedVal = windHistory[idx].speed;
            float gustVal = windHistory[idx].gust;
            if (speedVal < minVal) minVal = speedVal;
            if (speedVal > maxVal) maxVal = speedVal;
            if (gustVal < minVal) minVal = gustVal;
            if (gustVal > maxVal) maxVal = gustVal;
        }
        
        if (minVal > 0) minVal = 0.0;
        float range = maxVal - minVal;
        if (range < 10.0) {
            maxVal = 10.0;
        } else {
            maxVal += 2.0;
        }
        
        Serial.printf("Wind range: %.1f to %.1f\n", minVal, maxVal);
        
        lv_chart_series_t * speed_series = lv_chart_get_series_next(ui_Chart1, NULL);
        lv_chart_series_t * gust_series = NULL;
        
        if (!seriesInitialized) {
            Serial.println("Creating wind series");
            speed_series = lv_chart_add_series(ui_Chart1, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
            gust_series = lv_chart_add_series(ui_Chart1, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
            seriesInitialized = true;
        } else {
            gust_series = lv_chart_get_series_next(ui_Chart1, speed_series);
        }
        
        if (speed_series == NULL || gust_series == NULL) {
            Serial.println("ERROR: Failed to get/create series");
            return;
        }
        
        Serial.println("Populating wind data...");
        for (int i = 0; i < displayPoints; i++) {
            int historyIdx;
            if (windHistoryCount <= displayPoints) {
                historyIdx = i;
            } else {
                historyIdx = (displayPoints <= 1) ? 0 : (i * (windHistoryCount - 1)) / (displayPoints - 1);
            }
            
            if (historyIdx < windHistoryCount) {
                int idx = (startIdx + historyIdx) % windHistoryCapacity;
                
                float speedVal = windHistory[idx].speed;
                int32_t speed_chart_value = (int32_t)(speedVal * 100);
                lv_chart_set_value_by_id(ui_Chart1, speed_series, i, speed_chart_value);
                
                float gustVal = windHistory[idx].gust;
                int32_t gust_chart_value = (int32_t)(gustVal * 100);
                lv_chart_set_value_by_id(ui_Chart1, gust_series, i, gust_chart_value);
            } else {
                lv_chart_set_value_by_id(ui_Chart1, speed_series, i, LV_CHART_POINT_NONE);
                lv_chart_set_value_by_id(ui_Chart1, gust_series, i, LV_CHART_POINT_NONE);
            }
        }
        
        lv_chart_set_range(ui_Chart1, LV_CHART_AXIS_PRIMARY_Y, (int32_t)(minVal * 100), (int32_t)(maxVal * 100));
        
        if (chart_scale_y != NULL) {
            int totalTicks = (int)(maxVal - minVal) + 1;
            if (totalTicks < 2) totalTicks = 2;
            lv_scale_set_total_tick_count(chart_scale_y, totalTicks);
            if (totalTicks < 10) {
                lv_scale_set_major_tick_every(chart_scale_y, 1);
            } else {
                lv_scale_set_major_tick_every(chart_scale_y, totalTicks / 5);
            }
            lv_scale_set_range(chart_scale_y, (int32_t)minVal, (int32_t)maxVal);
            lv_scale_set_label_show(chart_scale_y, true);
        }
        
        if (chart_x_axis_container != NULL && windHistoryCount > 0) {
            int numLabels = lv_obj_get_child_count(chart_x_axis_container);
            
            for (int i = 0; i < numLabels; i++) {
                lv_obj_t * time_label = lv_obj_get_child(chart_x_axis_container, i);
                // CLEAR FIRST to prevent blur
                lv_label_set_text(time_label, "");
                int dataPoint;
                if (numLabels <= 1) {
                    dataPoint = 0;
                } else {
                    dataPoint = (i * (windHistoryCount - 1)) / (numLabels - 1);
                }
                
                if (dataPoint < windHistoryCount) {
                    int idx = (startIdx + dataPoint) % windHistoryCapacity;
                    
                    if (idx >= 0 && idx < windHistoryCapacity) {
                        time_t timestamp = (time_t)(windHistory[idx].timestamp / 1000);
                        
                        if (timestamp > 0 && timestamp >= 1000000000) {
                            struct tm timeinfo_buf;
                            struct tm* timeinfo = localtime_r(&timestamp, &timeinfo_buf);
                            
                            if (timeinfo != NULL) {
                                int hour = timeinfo->tm_hour;
                                int min = timeinfo->tm_min;
                                const char* ampm = (hour >= 12) ? "PM" : "AM";
                                if (hour > 12) hour -= 12;
                                else if (hour == 0) hour = 12;
                                
                                char time_buf[16];
                                snprintf(time_buf, sizeof(time_buf), "%d:%02d%s", hour, min, ampm);
                                lv_label_set_text(time_label, time_buf);
                            }
                        }
                    }
                }
            }
        }
        
        lv_chart_refresh(ui_Chart1);
        Serial.println("Wind chart complete");
        return;
    }
    
    // === SENSOR DATA CHART (temp/hum/pres) ===
    int sensor = currentChart.sensorIndex;
    if (sensor < 0 || sensor >= NUM_SENSORS || sensorHistory[sensor] == nullptr || sensorHistoryCount[sensor] == 0) {
        char titleBuf[64];
        snprintf(titleBuf, sizeof(titleBuf), "%s - Sensor %d - No Data", currentChart.title, sensor + 1);
        lv_label_set_text(ui_LabelChart, titleBuf);
        Serial.printf("No data for sensor %d\n", sensor);
        return;
    }
    
    char titleBuf[64];
    snprintf(titleBuf, sizeof(titleBuf), "%s - Sensor %d", currentChart.title, sensor + 1);
    lv_label_set_text(ui_LabelChart, titleBuf);
    
    int displayPoints = sensorHistoryCount[sensor];
    if (displayPoints > 100) displayPoints = 100;
    if (displayPoints < 2) displayPoints = 2;
    
    Serial.printf("Sensor %d chart: displayPoints=%d, historyCount=%d\n", sensor, displayPoints, sensorHistoryCount[sensor]);
    
    lv_chart_set_point_count(ui_Chart1, displayPoints);
    
    float minVal = 999.0;
    float maxVal = -999.0;
    int startIdx = (sensorHistoryIndex[sensor] - sensorHistoryCount[sensor] + sensorHistoryCapacity) % sensorHistoryCapacity;
    
    for (int i = 0; i < sensorHistoryCount[sensor]; i++) {
        int idx = (startIdx + i) % sensorHistoryCapacity;
        float val = 0;
        
        switch(currentChart.dataType) {
            case CHART_TEMP: val = sensorHistory[sensor][idx].temp; break;
            case CHART_HUM: val = sensorHistory[sensor][idx].hum; break;
            case CHART_PRES: val = sensorHistory[sensor][idx].pres; break;
            default: val = 0;
        }
        
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
    }
    
    float range = maxVal - minVal;
    if (range < 5.0) {
        float center = (minVal + maxVal) / 2.0;
        minVal = center - 2.5;
        maxVal = center + 2.5;
    } else {
        minVal -= 1.0;
        maxVal += 1.0;
    }
    
    Serial.printf("Sensor range: %.1f to %.1f\n", minVal, maxVal);
    
    lv_chart_series_t * series = lv_chart_get_series_next(ui_Chart1, NULL);
    if (!seriesInitialized) {
        Serial.println("Creating sensor series");
        series = lv_chart_add_series(ui_Chart1, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
        seriesInitialized = true;
    }
    
    if (series == NULL) {
        Serial.println("ERROR: Failed to get/create series");
        return;
    }
    
    Serial.println("Populating sensor data...");
    for (int i = 0; i < displayPoints; i++) {
        int historyIdx;
        if (sensorHistoryCount[sensor] <= displayPoints) {
            historyIdx = i;
        } else {
            historyIdx = (displayPoints <= 1) ? 0 : (i * (sensorHistoryCount[sensor] - 1)) / (displayPoints - 1);
        }
        
        if (historyIdx < sensorHistoryCount[sensor]) {
            int idx = (startIdx + historyIdx) % sensorHistoryCapacity;
            float val = 0;
            
            switch(currentChart.dataType) {
                case CHART_TEMP: val = sensorHistory[sensor][idx].temp; break;
                case CHART_HUM: val = sensorHistory[sensor][idx].hum; break;
                case CHART_PRES: val = sensorHistory[sensor][idx].pres; break;
                default: val = 0;
            }
            
            int32_t chart_value = (int32_t)(val * 100);
            lv_chart_set_value_by_id(ui_Chart1, series, i, chart_value);
        } else {
            lv_chart_set_value_by_id(ui_Chart1, series, i, LV_CHART_POINT_NONE);
        }
    }
    
    lv_chart_set_range(ui_Chart1, LV_CHART_AXIS_PRIMARY_Y, (int32_t)(minVal * 100), (int32_t)(maxVal * 100));
    
    if (chart_scale_y != NULL) {
        int totalTicks = (int)(maxVal - minVal) + 1;
        if (totalTicks < 2) totalTicks = 2;
        lv_scale_set_total_tick_count(chart_scale_y, totalTicks);
        if (totalTicks < 10) {
            lv_scale_set_major_tick_every(chart_scale_y, 1);
        } else {
            lv_scale_set_major_tick_every(chart_scale_y, totalTicks / 5);
        }
        lv_scale_set_range(chart_scale_y, (int32_t)minVal, (int32_t)maxVal);
        lv_scale_set_label_show(chart_scale_y, true);
    }
    
    if (chart_x_axis_container != NULL && sensorHistoryCount[sensor] > 0) {
        int numLabels = lv_obj_get_child_count(chart_x_axis_container);
        
        for (int i = 0; i < numLabels; i++) {
            lv_obj_t * time_label = lv_obj_get_child(chart_x_axis_container, i);
            // CLEAR FIRST to prevent blur
            lv_label_set_text(time_label, "");
            int dataPoint;
            if (numLabels <= 1) {
                dataPoint = 0;
            } else {
                dataPoint = (i * (sensorHistoryCount[sensor] - 1)) / (numLabels - 1);
            }
            
            if (dataPoint < sensorHistoryCount[sensor]) {
                int idx = (startIdx + dataPoint) % sensorHistoryCapacity;
                
                if (idx >= 0 && idx < sensorHistoryCapacity) {
                    time_t timestamp = (time_t)sensorHistory[sensor][idx].timestamp;
                    
                    if (timestamp > 0 && timestamp >= 1000000000) {
                        struct tm timeinfo_buf;
                        struct tm* timeinfo = localtime_r(&timestamp, &timeinfo_buf);
                        
                        if (timeinfo != NULL) {
                            int hour = timeinfo->tm_hour;
                            int min = timeinfo->tm_min;
                            const char* ampm = (hour >= 12) ? "PM" : "AM";
                            if (hour > 12) hour -= 12;
                            else if (hour == 0) hour = 12;
                            
                            char time_buf[16];
                            snprintf(time_buf, sizeof(time_buf), "%d:%02d%s", hour, min, ampm);
                            lv_label_set_text(time_label, time_buf);
                        }
                    }
                }
            }
        }
    }
    
    lv_chart_refresh(ui_Chart1);
    Serial.println("Sensor chart complete");
}

// MODIFIED: New WiFi startup state machine to skip scanning if we have credentials
void handleWifiStartup() {
    unsigned long now = millis();
    
    switch (wifiStartupState) {
        case WIFI_STARTUP_INIT:
            Serial.println("=== WiFi Startup: Initializing ===");
            
            WiFi.mode(WIFI_AP_STA);
            WiFi.setTxPower(WIFI_POWER_8_5dBm);
            
            // Initialize SNTP now that WiFi mode is set
            if (!isSetNtp) {
                initSNTP();
                Serial.println("SNTP initialized");
            }
            
            // Initialize ESP-NOW now that WiFi is initialized
            if (esp_now_init() != ESP_OK) {
                Serial.println("Error initializing ESP-NOW");
            } else {
                esp_now_register_recv_cb(onESPNowReceive);
                Serial.println("ESP-NOW initialized");
            }
            
            // Start UDP listener
            udp.begin(UDP_PORT);
            Serial.printf("UDP listener started on port %d\n", UDP_PORT);
            
            wifiStartupState = WIFI_STARTUP_CHECK_CREDS;  // Skip SCAN, go straight to credential check
            wifiStartupStepTime = now;
            break;
            
        case WIFI_STARTUP_SCAN:
            // Wait a bit before scanning
            if (now - wifiStartupStepTime < 500) break;
            
            Serial.println("=== WiFi Startup: Scanning networks ===");
            consoleLog("Scanning for networks...");
            lv_timer_handler();  // Update display
            
            populateWifiList();
            
            wifiStartupState = WIFI_STARTUP_COMPLETE;  // After scan, we're done
            wifiStartupStepTime = now;
            break;
            
        case WIFI_STARTUP_CHECK_CREDS:
            // Wait a bit after init
            if (now - wifiStartupStepTime < 500) break;
            
            Serial.println("=== WiFi Startup: Checking for saved credentials ===");
            
            // Check for saved credentials
            if (loadWifiCredentials(startupSSID, startupPassword)) {
                Serial.printf("Found saved credentials: SSID='%s'\n", startupSSID.c_str());
                
                consoleLog("Found saved credentials!");
                lv_timer_handler();  // Update display
                
                char logBuf[128];
                snprintf(logBuf, sizeof(logBuf), "SSID: %s", startupSSID.c_str());
                consoleLog(logBuf);
                lv_timer_handler();  // Update display
                
                consoleLog("Connecting to WiFi...");
                lv_timer_handler();  // Update display
                
                delay(500);  // Brief pause so user can see the messages
                
                // Start connection attempt
                Serial.printf("Starting WiFi.begin('%s', '***')\n", startupSSID.c_str());
                WiFi.begin(startupSSID.c_str(), startupPassword.c_str());
                wifiConnecting = true;
                wifiConnectStartTime = now;
                autoConnectMode = true;
                
                wifiStartupState = WIFI_STARTUP_CONNECTING;
                wifiStartupStepTime = now;
            } else {
                Serial.println("No saved WiFi credentials found - will scan networks");
                consoleLog("No saved credentials");
                lv_timer_handler();
                
                // No credentials - scan networks now
                wifiStartupState = WIFI_STARTUP_SCAN;
                wifiStartupStepTime = now;
            }
            break;
            
        case WIFI_STARTUP_CONNECTING:
            // checkWifiConnection() handles the actual connection monitoring
            // We just need to detect when it's done
            if (!wifiConnecting) {
                // Connection attempt finished (success or failure)
                if (WiFi.status() == WL_CONNECTED) {
                    // Success - we're done
                    wifiStartupState = WIFI_STARTUP_COMPLETE;
                } else {
                    // Failed - scan networks so user can try again
                    Serial.println("Connection failed - scanning networks for manual selection");
                    consoleLog("Connection failed");
                    lv_timer_handler();
                    wifiStartupState = WIFI_STARTUP_SCAN;
                    wifiStartupStepTime = now;
                }
            }
            break;
            
        case WIFI_STARTUP_COMPLETE:
            // Nothing to do - startup complete
            break;
    }
}

// ============================================
// UDP Packet Handler
// ============================================

// UDP packet structures matching the transmitters
typedef struct {
  uint8_t sensor_id;
  float temp;
  float hum;
  float pres;
  uint8_t magic[4];  // Magic bytes: 0x54, 0x45, 0x4D, 0x50 ("TEMP")
} temp_sensor_payload_t;

typedef struct {
  uint8_t sensor_id;
  float temp;
  float windgust;
  float avgwind;
  float instwind;
  float winddirV;
  float pres;
  float hum;
  float avgWindow;      // ADD THIS
  float gustWindow;     // ADD THIS
  uint8_t magic[4];     // Magic bytes: 0x57, 0x49, 0x4E, 0x44 ("WIND")
} wind_sensor_payload_t;

// UPDATE the handleUDP() function's wind packet handler:
void handleUDP() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        uint8_t buffer[256];
        int len = udp.read(buffer, sizeof(buffer));
        
        Serial.printf("UDP packet received: %d bytes\n", len);
        
        // Check for temperature sensor data
        if (len == sizeof(temp_sensor_payload_t)) {
            temp_sensor_payload_t payload;
            memcpy(&payload, buffer, sizeof(payload));
            
            // Verify magic bytes for TEMP packet
            if (payload.magic[0] == 0x54 && payload.magic[1] == 0x45 &&
                payload.magic[2] == 0x4D && payload.magic[3] == 0x50) {
                
                // Check if sensor_id is valid
                if (payload.sensor_id >= NUM_SENSORS) {
                    Serial.printf("Invalid sensor_id: %d (max: %d)\n", payload.sensor_id, NUM_SENSORS - 1);
                    return;
                }
                
                // Check for duplicate within 2 seconds
                unsigned long now = millis();
                if ((now - lastTempMsgTime[payload.sensor_id]) < DUPLICATE_REJECT_MS) {
                    Serial.printf("Duplicate TEMP packet from sensor %d rejected (%.1fs since last)\n", 
                                  payload.sensor_id, 
                                  (now - lastTempMsgTime[payload.sensor_id]) / 1000.0);
                    return;
                }
                
                // Update last message time
                lastTempMsgTime[payload.sensor_id] = now;
                
                Serial.printf("Sensor %d: Temp=%.2f°C, Hum=%.1f%%, Pres=%.1fmbar\n", 
                              payload.sensor_id, payload.temp, payload.hum, payload.pres);
                
                addTempToHistory(payload.sensor_id, payload.temp, payload.hum, payload.pres);
            }
        }
        // Check for wind sensor data
        else if (len == sizeof(wind_sensor_payload_t)) {
            wind_sensor_payload_t payload;
            memcpy(&payload, buffer, sizeof(payload));
            
            // Verify magic bytes for WIND packet
            if (payload.magic[0] == 0x57 && payload.magic[1] == 0x49 &&
                payload.magic[2] == 0x4E && payload.magic[3] == 0x44) {
                
                // Check for duplicate within 2 seconds
                unsigned long now = millis();
                if ((now - lastWindMsgTime) < DUPLICATE_REJECT_MS) {
                    Serial.printf("Duplicate WIND packet rejected (%.1fs since last)\n", 
                                  (now - lastWindMsgTime) / 1000.0);
                    return;
                }
                
                // Update last message time
                lastWindMsgTime = now;
                
                Serial.printf("Wind Sensor: Speed=%.1fkph, Dir=%.1f, Gust=%.1fkph, AvgWin=%.1fs, GustWin=%.1fs\n", 
                              payload.avgwind, payload.winddirV, payload.windgust, 
                              payload.avgWindow, payload.gustWindow);
                
                s2windgust = payload.windgust;
                
                // UPDATE: Store the window settings from transmitter
                currentAvgWindow = payload.avgWindow;
                currentGustWindow = payload.gustWindow;
                
                // UPDATE: Update the spinbox values if we're on the settings screen
                if (lv_screen_active() == ui_ScreenSettings) {
                    if (ui_Spinbox1 != NULL && ui_Spinbox2 != NULL) {
                        lv_spinbox_set_value(ui_Spinbox1, (int32_t)currentAvgWindow);
                        lv_spinbox_set_value(ui_Spinbox2, (int32_t)currentGustWindow);
                        Serial.printf("Updated spinboxes: Avg=%.1f, Gust=%.1f\n", 
                                      currentAvgWindow, currentGustWindow);
                    }
                }
                
                // Add wind data WITH GUST parameter
                addWindData(payload.avgwind, getWindDirection(payload.winddirV), payload.windgust);
                
                // Also store temp/hum/pres from wind sensor
                if (payload.sensor_id < NUM_SENSORS) {
                    // Use separate duplicate check for wind sensor temp data
                    if ((now - lastTempMsgTime[payload.sensor_id]) >= DUPLICATE_REJECT_MS) {
                        lastTempMsgTime[payload.sensor_id] = now;
                        addTempToHistory(payload.sensor_id, payload.temp, payload.hum, payload.pres);
                    } else {
                        Serial.printf("Wind sensor temp data for sensor %d skipped (duplicate)\n", payload.sensor_id);
                    }
                }
            }
        } else {
            Serial.printf("Unknown packet size: %d bytes\n", len);
        }
    }
}

void onESPNowReceive(const esp_now_recv_info_t * info, const uint8_t *data, int len)
{
  // Check for credential request
  if (len == sizeof(credential_request_t)) {
    credential_request_t request;
    memcpy(&request, data, sizeof(request));
    
    // Verify magic bytes
    if (request.magic[0] == 0xDE && request.magic[1] == 0xAD &&
        request.magic[2] == 0xBE && request.magic[3] == 0xEF) {
      
      Serial.println("Received credential request!");
      
      // Get current WiFi credentials
      String ssid = WiFi.SSID();
      String password = WiFi.psk();
      
      if (ssid.length() > 0) {
        wifi_credentials_t creds;
        memset(&creds, 0, sizeof(creds));
        strncpy(creds.ssid, ssid.c_str(), sizeof(creds.ssid) - 1);
        strncpy(creds.password, password.c_str(), sizeof(creds.password) - 1);
        creds.magic[0] = 0xCA;
        creds.magic[1] = 0xFE;
        creds.magic[2] = 0xBA;
        creds.magic[3] = 0xBE;
        
        // Add peer if not exists - NO ENCRYPTION
        uint8_t peerMAC[6];
        memcpy(peerMAC, info->src_addr, 6);
        
        if (!esp_now_is_peer_exist(peerMAC)) {
          esp_now_peer_info_t peerInfo = {};
          memcpy(peerInfo.peer_addr, peerMAC, 6);
          peerInfo.channel = 0;
          peerInfo.encrypt = false;  // NO ENCRYPTION!
          esp_now_add_peer(&peerInfo);
        }
        
        // Send credentials unencrypted
        esp_err_t result = esp_now_send(peerMAC, (uint8_t *)&creds, sizeof(creds));
        
        if (result == ESP_OK) {
          Serial.printf("Sent credentials to sensor: %s\n", ssid.c_str());
        } else {
          Serial.printf("Failed to send credentials: %d\n", result);
        }
      } else {
        Serial.println("No WiFi credentials to send");
      }
    }
  }
}

uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_display_t *disp;
lv_color_t *disp_draw_buf;

void my_print(lv_log_level_t level, const char *buf) {
    LV_UNUSED(level);
    Serial.println(buf);
    Serial.flush();
}

uint32_t millis_cb(void) {
    return millis();
}

void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
    touchController.read();
    if (touchController.isTouched && touchController.touches > 0) {
        data->point.x = touchController.points[0].x;
        data->point.y = touchController.points[0].y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void drawWindRose() {
    if (windHistory == nullptr || windHistoryCount == 0) return;
    
    static lv_obj_t * wind_rose_canvas = NULL;
    static lv_draw_buf_t draw_buf;
    static bool buf_initialized = false;
    
    const int16_t canvasWidth = 180;
    const int16_t canvasHeight = 180;
    const int16_t centerX = canvasWidth / 2;
    const int16_t centerY = canvasHeight / 2;
    const int16_t maxRadius = 57;
    int WINDROSECENTER_X = -55;
    int WINDROSECENTER_Y = 0;
    int DRAW_CENTER_X = 240 + WINDROSECENTER_X;
    int DRAW_CENTER_Y = 136 + WINDROSECENTER_Y;
    lv_color_t speedColors[WIND_SPEED_BINS] = {
        lv_color_hex(0x00FFFF),
        lv_color_hex(0x00FF00),
        lv_color_hex(0xFFFF00),
        lv_color_hex(0xFF8800),
        lv_color_hex(0xFF0000)
    };
    
    if (wind_rose_canvas == NULL) {
        wind_rose_canvas = lv_canvas_create(lv_scr_act());
        
        uint32_t buf_size = canvasWidth * canvasHeight * 4;
        void * buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (buf == NULL) {
            buf = malloc(buf_size);
        }
        
        if (buf != NULL) {
            lv_draw_buf_init(&draw_buf, canvasWidth, canvasHeight, LV_COLOR_FORMAT_ARGB8888, 0, buf, buf_size);
            lv_canvas_set_draw_buf(wind_rose_canvas, &draw_buf);
            lv_obj_set_pos(wind_rose_canvas, (DRAW_CENTER_X + 3) - centerX, DRAW_CENTER_Y - centerY);
            lv_obj_set_style_bg_opa(wind_rose_canvas, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(wind_rose_canvas, 0, 0);
            lv_obj_set_style_pad_all(wind_rose_canvas, 0, 0);
            buf_initialized = true;
        } else {
            Serial.println("Failed to allocate canvas buffer");
            return;
        }
    }
    
    if (!buf_initialized) return;
    
    lv_canvas_fill_bg(wind_rose_canvas, lv_color_black(), LV_OPA_TRANSP);
    
    int directionTotals[WIND_DIRECTIONS] = {0};
    int maxDirectionCount = 0;
    
    for (int dir = 0; dir < WIND_DIRECTIONS; dir++) {
        if (windRoseData[dir] != nullptr) {
            for (int spd = 0; spd < WIND_SPEED_BINS; spd++) {
                directionTotals[dir] += windRoseData[dir][spd];
            }
            if (directionTotals[dir] > maxDirectionCount) {
                maxDirectionCount = directionTotals[dir];
            }
        }
    }
    
    if (maxDirectionCount == 0) return;
    
    lv_layer_t layer;
    lv_canvas_init_layer(wind_rose_canvas, &layer);
    
    for (int dir = 0; dir < WIND_DIRECTIONS; dir++) {
        if (windRoseData[dir] == nullptr || directionTotals[dir] == 0) continue;
        
        float spokeLength = (float)directionTotals[dir] / (float)maxDirectionCount * maxRadius;
        float dirAngle = (dir * 22.5f) - 90.0f;
        float currentRadius = 0;
        
        for (int spd = 0; spd < WIND_SPEED_BINS; spd++) {
            int count = windRoseData[dir][spd];
            if (count == 0) continue;
            
            float bandLength = (float)count / (float)directionTotals[dir] * spokeLength;
            float innerRadius = currentRadius;
            float outerRadius = currentRadius + bandLength;
            
            int16_t startAngle = (int16_t)(dirAngle - 11.25f);
            int16_t endAngle = (int16_t)(dirAngle + 11.25f);
            
            while (startAngle < 0) startAngle += 360;
            while (endAngle < 0) endAngle += 360;
            
            int numArcs = (int)(bandLength / 2) + 1;
            if (numArcs < 1) numArcs = 1;
            
            for (int a = 0; a < numArcs; a++) {
                float radius = innerRadius + (bandLength * a / numArcs) + (bandLength / numArcs / 2);
                int arcWidth = (int)(bandLength / numArcs) + 1;
                if (arcWidth < 1) arcWidth = 1;
                
                lv_draw_arc_dsc_t arc_dsc;
                lv_draw_arc_dsc_init(&arc_dsc);
                arc_dsc.color = speedColors[spd];
                arc_dsc.width = arcWidth;
                arc_dsc.opa = LV_OPA_COVER;
                arc_dsc.rounded = 0;
                arc_dsc.center.x = centerX;
                arc_dsc.center.y = centerY;
                arc_dsc.radius = (int16_t)radius;
                arc_dsc.start_angle = startAngle;
                arc_dsc.end_angle = endAngle;
                
                lv_draw_arc(&layer, &arc_dsc);
            }
            
            currentRadius = outerRadius;
        }
    }
    
    lv_canvas_finish_layer(wind_rose_canvas, &layer);
}

// ============================================
// WiFi Functions (called from ui_events.c)
// ============================================

extern "C" void populateWifiList() {
	if (ui_DropdownSSID == NULL) return;
	
	Serial.println("Scanning for WiFi networks...");
	int n = WiFi.scanNetworks();
	
	if (n == 0) {
		lv_dropdown_set_options(ui_DropdownSSID, "No networks found");
		Serial.println("No WiFi networks found");
		return;
	}
	
	// Build dropdown string with all SSIDs
	String options = "";
	for (int i = 0; i < n; i++) {
		if (i > 0) options += "\n";
		options += WiFi.SSID(i);
		
		// Log SSID and signal strength
		Serial.printf("%d: %s (%d dBm)\n", i, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
	}
	
	lv_dropdown_set_options(ui_DropdownSSID, options.c_str());
	Serial.printf("Found %d networks\n", n);
}

// Modify the SaveWifiFromUI function
extern "C" void SaveWifiFromUI(lv_event_t * e)
{
    Serial.println("=== SaveWifiFromUI called ===");
    
    if (ui_DropdownSSID == NULL || ui_TextAreaPassword == NULL) {
        consoleLog("ERROR: WiFi UI elements not found");
        Serial.println("ERROR: ui_DropdownSSID or ui_TextAreaPassword is NULL");
        return;
    }
    
    // Clear console
    lv_label_set_text(ui_LabelConsole, "");
    
    // Get selected SSID from dropdown
    char ssidBuffer[33];
    memset(ssidBuffer, 0, sizeof(ssidBuffer));
    lv_dropdown_get_selected_str(ui_DropdownSSID, ssidBuffer, sizeof(ssidBuffer));
    
    Serial.printf("Selected SSID from dropdown: '%s'\n", ssidBuffer);
    
    // Get password from text area
    const char* password = lv_textarea_get_text(ui_TextAreaPassword);
    
    Serial.printf("Password from text area: '%s'\n", password);
    Serial.printf("Password length: %d\n", strlen(password));
    
    consoleLog("Saving WiFi credentials...");
    
    char logBuf[128];
    snprintf(logBuf, sizeof(logBuf), "SSID: %s", ssidBuffer);
    consoleLog(logBuf);
    
    snprintf(logBuf, sizeof(logBuf), "Password length: %d chars", strlen(password));
    consoleLog(logBuf);
    
    // Save to Preferences (persistent storage)
    Preferences preferences;
    preferences.begin("wifi", false);
    preferences.putString("ssid", ssidBuffer);
    preferences.putString("password", password);
    preferences.end();
    
    Serial.println("Credentials saved to Preferences");
    consoleLog("Credentials saved to flash");
    
    // Disconnect if already connected
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Disconnecting from current WiFi");
        WiFi.disconnect();
        delay(100);
    }
    
    // Set flags to attempt connection (manual mode, don't auto-switch screens)
    wifiConnecting = true;
    wifiConnectStartTime = millis();
    autoConnectMode = false;  // Manual connection via UI
    
    // Attempt to connect
    Serial.printf("Calling WiFi.begin('%s', '%s')\n", ssidBuffer, password);
    WiFi.begin(ssidBuffer, password);
    consoleLog("Connecting to WiFi...");
    
    Serial.println("=== SaveWifiFromUI complete ===");
}



void updateBacklight();
void updateBacklight() {
    if (millis() - lastManualBrightnessSet < BRIGHTNESS_DEBOUNCE_MS) return;

    // blGamma is maintained by loadSettings(), saveSettings(), and applySettingsToUI().
    // Do NOT read it from the spinbox here — the spinbox default (10) overwrites
    // the loaded value before applySettingsToUI() has a chance to correct it.

    int brightness;
    if (autoBrightness) {
        if (!lightMeter.measurementReady()) return;
        float lux = lightMeter.readLightLevel();
        if (lux < 0) return;
        brightness = luxToBrightness(lux, brightnessOffset);
    } else {
        brightness = brightnessOffset * 16;
    }
    ledcWrite(GFX_BL, clampInt(brightness, 0, BL_MAX));
}



void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  
  _calibrationParams.vaneADCValues[WMK_ANGLE_0_0] = SFE_WMK_ADC_ANGLE_0_0;
  _calibrationParams.vaneADCValues[WMK_ANGLE_22_5] = SFE_WMK_ADC_ANGLE_22_5;
  _calibrationParams.vaneADCValues[WMK_ANGLE_45_0] = SFE_WMK_ADC_ANGLE_45_0;
  _calibrationParams.vaneADCValues[WMK_ANGLE_67_5] = SFE_WMK_ADC_ANGLE_67_5;
  _calibrationParams.vaneADCValues[WMK_ANGLE_90_0] = SFE_WMK_ADC_ANGLE_90_0;
  _calibrationParams.vaneADCValues[WMK_ANGLE_112_5] = SFE_WMK_ADC_ANGLE_112_5;
  _calibrationParams.vaneADCValues[WMK_ANGLE_135_0] = SFE_WMK_ADC_ANGLE_135_0;
  _calibrationParams.vaneADCValues[WMK_ANGLE_157_5] = SFE_WMK_ADC_ANGLE_157_5;
  _calibrationParams.vaneADCValues[WMK_ANGLE_180_0] = SFE_WMK_ADC_ANGLE_180_0;
  _calibrationParams.vaneADCValues[WMK_ANGLE_202_5] = SFE_WMK_ADC_ANGLE_202_5;
  _calibrationParams.vaneADCValues[WMK_ANGLE_225_0] = SFE_WMK_ADC_ANGLE_225_0;
  _calibrationParams.vaneADCValues[WMK_ANGLE_247_5] = SFE_WMK_ADC_ANGLE_247_5;
  _calibrationParams.vaneADCValues[WMK_ANGLE_270_0] = SFE_WMK_ADC_ANGLE_270_0;
  _calibrationParams.vaneADCValues[WMK_ANGLE_292_5] = SFE_WMK_ADC_ANGLE_292_5;
  _calibrationParams.vaneADCValues[WMK_ANGLE_315_0] = SFE_WMK_ADC_ANGLE_315_0;
  _calibrationParams.vaneADCValues[WMK_ANGLE_337_5] = SFE_WMK_ADC_ANGLE_337_5;
  
  // Allocate sensor history arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
      if (psramFound()) {
          sensorHistory[i] = (SensorReading*)ps_malloc(SENSOR_HISTORY_SIZE * sizeof(SensorReading));
      }
      if (sensorHistory[i] == nullptr) {
          sensorHistory[i] = (SensorReading*)malloc(SENSOR_HISTORY_SIZE * sizeof(SensorReading));
      }
      if (sensorHistory[i] != nullptr) {
          for (int j = 0; j < SENSOR_HISTORY_SIZE; j++) {
              sensorHistory[i][j].temp = 0;
              sensorHistory[i][j].hum = 0;
              sensorHistory[i][j].pres = 0;
              sensorHistory[i][j].timestamp = 0;
          }
      }
  }
  
  if (psramFound()) {
    tempHistory = (TempDataPoint*)ps_malloc(TEMP_HISTORY_SIZE * sizeof(TempDataPoint));
    if (tempHistory == nullptr) {
      Serial.println("Failed to allocate PSRAM for temperature history - using SRAM fallback");
      tempHistory = tempHistoryFallback;
      tempHistoryCapacity = TEMP_FALLBACK_SIZE;
      for (int i = 0; i < tempHistoryCapacity; i++) {
        tempHistory[i].temp = 0.0;
        tempHistory[i].timestamp = 0;
      }
    } else {
      Serial.println("Temperature history allocated in PSRAM");
      tempHistoryCapacity = TEMP_HISTORY_SIZE;
      for (int i = 0; i < tempHistoryCapacity; i++) {
        tempHistory[i].temp = 0.0;
        tempHistory[i].timestamp = 0;
      }
    }
    
    windHistory = (WindDataPoint*)ps_malloc(WIND_HISTORY_SIZE * sizeof(WindDataPoint));
    if (windHistory == nullptr) {
      Serial.println("Failed to allocate PSRAM for wind history - using SRAM fallback");
      windHistory = windHistoryFallback;
      windHistoryCapacity = WIND_HISTORY_FALLBACK_SIZE;
      for (int i = 0; i < windHistoryCapacity; i++) {
        windHistory[i].speed = 0.0;
        windHistory[i].direction = 0.0;
        windHistory[i].timestamp = 0;
      }
    } else {
      Serial.println("Wind history allocated in PSRAM");
      windHistoryCapacity = WIND_HISTORY_SIZE;
      for (int i = 0; i < windHistoryCapacity; i++) {
        windHistory[i].speed = 0.0;
        windHistory[i].direction = 0.0;
        windHistory[i].timestamp = 0;
      }
    }
    
    for (int i = 0; i < WIND_DIRECTIONS; i++) {
      windRoseData[i] = (int*)ps_malloc(WIND_SPEED_BINS * sizeof(int));
      if (windRoseData[i] != nullptr) {
        for (int j = 0; j < WIND_SPEED_BINS; j++) {
          windRoseData[i][j] = 0;
        }
      } else {
        windRoseData[i] = &windRoseFallback[i][0];
        for (int j = 0; j < WIND_SPEED_BINS; j++) windRoseData[i][j] = 0;
      }
    }
  } else {
    Serial.println("PSRAM not found! Using SRAM fallbacks.");
    tempHistory = tempHistoryFallback;
    tempHistoryCapacity = TEMP_FALLBACK_SIZE;
    for (int i = 0; i < tempHistoryCapacity; i++) {
      tempHistory[i].temp = 0.0;
      tempHistory[i].timestamp = 0;
    }
    windHistory = windHistoryFallback;
    windHistoryCapacity = WIND_HISTORY_FALLBACK_SIZE;
    for (int i = 0; i < windHistoryCapacity; i++) {
      windHistory[i].speed = 0.0;
      windHistory[i].direction = 0.0;
      windHistory[i].timestamp = 0;
    }
    for (int i = 0; i < WIND_DIRECTIONS; i++) {
      windRoseData[i] = &windRoseFallback[i][0];
      for (int j = 0; j < WIND_SPEED_BINS; j++) windRoseData[i][j] = 0;
    }
  }
  
    if (!gfx->begin()) {
        Serial.println("× gfx->begin() failed!");
        while (true);
    }
    gfx->invertDisplay(false);
    
    pinMode(GFX_BL, OUTPUT);

    // Set up PWM for backlight (ESP Arduino Core v3.x API)
    ledcAttach(GFX_BL, BL_PWM_FREQ, BL_PWM_RESOLUTION);
    ledcWrite(GFX_BL, BL_MAX);  // Full brightness on startup

    // Initialize secondary I2C bus for BH1750
    I2C_BH1750.begin(17, 18);   // SDA=17, SCL=18
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2C_BH1750)) {
        Serial.println("BH1750 initialized on secondary I2C bus (SDA=17, SCL=18)");
    } else {
        Serial.println("BH1750 init failed - check wiring on pins 17/18");
    }

    gfx->fillScreen(RGB565_BLACK);

    touchController.begin();
    touchController.setRotation(ROTATION_INVERTED);

    lv_init();
    lv_tick_set_cb(millis_cb);

#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print);
#endif

    screenWidth = gfx->width();
    screenHeight = gfx->height();
    bufSize = screenWidth * 40;

    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!disp_draw_buf)
        disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);

    if (!disp_draw_buf) {
        Serial.println("× LVGL disp_draw_buf allocation failed!");
        while (true);
    }

    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touchpad_read);
    
    ui_init();
    
    lv_scr_load(ui_ScreenWifi);
    
    // Give LVGL time to render
    lv_timer_handler();
    delay(100);
    
    Serial.println("UI initialized, starting WiFi setup...");
    
    // Don't do ANY WiFi/network setup here - let the state machine handle it
    // Just initialize the state machine
    wifiStartupState = WIFI_STARTUP_INIT;
    wifiStartupStepTime = millis();
    loadSettings();

    // Apply initial brightness (before first BH1750 read)
    if (!autoBrightness) {
        ledcWrite(GFX_BL, clampInt(brightnessOffset * 16, 0, BL_MAX));
    }

    // Conditionally init Blynk
    if (blynkEnabled) {
        Blynk.config(auth, IPAddress(216, 110, 224, 105), 8080);
    }
}


void loop() {
    // CRITICAL: Handle LVGL first
    lv_timer_handler();
    // Detect screen changes and cleanup
 
    // Handle WiFi startup state machine
    if (wifiStartupState != WIFI_STARTUP_COMPLETE) {
        handleWifiStartup();
    }

    // Handle ongoing WiFi connection monitoring
    checkWifiConnection();
  if ((WiFi.status() == WL_CONNECTED) && (!connected)) {
    connected = true;
    ArduinoOTA.setHostname("SeanTemp");
    ArduinoOTA.begin();
  }
  if ((WiFi.status() == WL_CONNECTED) && connected) {
    ArduinoOTA.handle();
    if (blynkEnabled) Blynk.run();
    handleUDP();
  }
    // === SEPARATE CHART UPDATE - OUTSIDE every(500) ===
    static unsigned long lastChartCheck = 0;
    if (millis() - lastChartCheck > 1000) {  // Only check every 1 second
        lastChartCheck = millis();
        
        // Screen change detection
        static lv_obj_t * prev_screen = NULL;
        static unsigned long screen_change_time = 0;
        lv_obj_t * curr_screen = lv_screen_active();
        
        if (prev_screen != curr_screen) {
            Serial.printf("Screen changed: %p -> %p\n", prev_screen, curr_screen);
            screen_change_time = millis();
            
            if (prev_screen == ui_ScreenChart && curr_screen != ui_ScreenChart) {
                Serial.println("Leaving chart screen - cleanup");
                cleanupChartContainers();
            }
            
            if (curr_screen == ui_ScreenChart && prev_screen != ui_ScreenChart) {
                Serial.println("Entering chart screen - will initialize");
            }
            
            prev_screen = curr_screen;
        }
        
        // Only draw if on chart screen and stable
        if (curr_screen == ui_ScreenChart && (millis() - screen_change_time > 500)) {
            Serial.println("About to call drawGenericChart()");
            unsigned long start = millis();
            drawGenericChart();
            unsigned long duration = millis() - start;
            Serial.printf("drawGenericChart() took %lu ms\n", duration);
        }
    }
    
  
  every(500) {
    char buf[64];
    if (lv_screen_active() == ui_ScreenSettings) {
        updateSettingsWifiInfo();
    }
    // Update sensor 0 displays (LabelTemp0, LabelHum0, LabelPres0)
    SensorReading* s0 = getLatestReading(0);
    if (s0 != nullptr) {
      snprintf(buf, sizeof(buf), "%.1f°C", s0->temp);
      lv_label_set_text(ui_LabelTemp0, buf);
      
      snprintf(buf, sizeof(buf), "%.0f%%", s0->hum);
      lv_label_set_text(ui_LabelHum0, buf);
      
      snprintf(buf, sizeof(buf), "%.1fmbar", s0->pres);
      lv_label_set_text(ui_LabelPres0, buf);
      
      // Calculate time since last update
      time_t now = time(nullptr);
      if (s0->timestamp > 1577836800 && s0->timestamp <= now) {
        unsigned long secondsSince = now - s0->timestamp;
        snprintf(buf, sizeof(buf), "%lus", secondsSince);
      } else {
        snprintf(buf, sizeof(buf), "--");
      }
      lv_label_set_text(ui_LabelLU0, buf);
    }
    if (lv_screen_active() == ui_ScreenChart) {
        chart_needs_update = true;
    }
    // Update sensor 1 displays (LabelTemp1, LabelHum1, LabelPres1)
    SensorReading* s1 = getLatestReading(1);
    if (s1 != nullptr) {
      snprintf(buf, sizeof(buf), "%.1f°C", s1->temp);
      lv_label_set_text(ui_LabelTemp1, buf);
      
      snprintf(buf, sizeof(buf), "%.0f%%", s1->hum);
      lv_label_set_text(ui_LabelHum1, buf);
      
      snprintf(buf, sizeof(buf), "%.1fmbar", s1->pres);
      lv_label_set_text(ui_LabelPres1, buf);
      
      time_t now = time(nullptr);
      if (s1->timestamp > 1577836800 && s1->timestamp <= now) {
        unsigned long secondsSince = now - s1->timestamp;
        snprintf(buf, sizeof(buf), "%lus", secondsSince);
      } else {
        snprintf(buf, sizeof(buf), "--");
      }
      lv_label_set_text(ui_LabelLU1, buf);
    }
    
    // Update sensor 2 displays (LabelTemp2, LabelHum2, LabelPres2)
    SensorReading* s2 = getLatestReading(2);
    if (s2 != nullptr) {
      snprintf(buf, sizeof(buf), "%.1f°C", s2->temp);
      lv_label_set_text(ui_LabelTemp2, buf);
      
      snprintf(buf, sizeof(buf), "%.0f%%", s2->hum);
      lv_label_set_text(ui_LabelHum2, buf);
      
      snprintf(buf, sizeof(buf), "%.1fmbar", s2->pres);
      lv_label_set_text(ui_LabelPres2, buf);
      
      time_t now = time(nullptr);
      if (s2->timestamp > 1577836800 && s2->timestamp <= now) {
        unsigned long secondsSince = now - s2->timestamp;
        snprintf(buf, sizeof(buf), "%lus", secondsSince);
      } else {
        snprintf(buf, sizeof(buf), "--");
      }
      lv_label_set_text(ui_LabelLU2, buf);
    }
    
    // Update sensor 3 displays (LabelTemp3, LabelHum3, LabelPres3)
    SensorReading* s3 = getLatestReading(3);
    if (s3 != nullptr) {
      snprintf(buf, sizeof(buf), "%.1f°C", s3->temp);
      lv_label_set_text(ui_LabelTemp3, buf);
      
      snprintf(buf, sizeof(buf), "%.0f%%", s3->hum);
      lv_label_set_text(ui_LabelHum3, buf);
      
      snprintf(buf, sizeof(buf), "%.1fmbar", s3->pres);
      lv_label_set_text(ui_LabelPres3, buf);
      
      time_t now = time(nullptr);
      if (s3->timestamp > 1577836800 && s3->timestamp <= now) {
        unsigned long secondsSince = now - s3->timestamp;
        snprintf(buf, sizeof(buf), "%lus", secondsSince);
      } else {
        snprintf(buf, sizeof(buf), "--");
      }
      lv_label_set_text(ui_LabelLU3, buf);
    }
    
    // Update wind data (if available from windHistory)
    if (windHistory != nullptr && windHistoryCount > 0) {
      int lastIdx = (windHistoryIndex - 1 + windHistoryCapacity) % windHistoryCapacity;
      float speed = windHistory[lastIdx].speed;
      float gust = windHistory[lastIdx].gust;
      float direction = windHistory[lastIdx].direction;
      
      snprintf(buf, sizeof(buf), "%.1fkph", speed);
      lv_label_set_text(ui_LabelWind, buf);
      snprintf(buf, sizeof(buf), "%.1fkph", gust);
      lv_label_set_text(ui_LabelWind1, buf);
      
      const char* cardinal = getCardinalDirection(direction);
      lv_label_set_text(ui_LabelWindDir, cardinal);
    }
    
    // Update time display (LabelTime)
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    int hour = timeinfo->tm_hour;
    int min = timeinfo->tm_min;
    int sec = timeinfo->tm_sec;
    const char* ampm = (hour >= 12) ? "PM" : "AM";
    if (hour > 12) hour -= 12;
    else if (hour == 0) hour = 12;
    snprintf(buf, sizeof(buf), "%d:%02d:%02d %s", hour, min, sec, ampm);
    lv_label_set_text(ui_LabelTime, buf);

    
    
    // Always update wind rose on main screen
    if (lv_screen_active() == ui_ScreenMain) {
        drawWindRose();
    }
    updateBacklight();
  }

  every (2000) {
    if (lv_screen_active() != ui_ScreenChart) {
        chart_needs_init = false;
        chart_needs_update = false;
        return;
    }
    

  }
  
  every(30000) {
    // Send latest wind data to Blynk every 30 seconds
    if (!blynkEnabled) return; 
    if (windHistory != nullptr && windHistoryCount > 0) {
      int lastIdx = (windHistoryIndex - 1 + windHistoryCapacity) % windHistoryCapacity;
      Blynk.virtualWrite(V12, windHistory[lastIdx].speed);
      Blynk.virtualWrite(V13, s2windgust);
      Blynk.virtualWrite(V15, windHistory[lastIdx].direction);
    }
    Blynk.virtualWrite(V14, WiFi.RSSI());

    // Send latest sensor data to Blynk

      if (sensorHistory[0] != nullptr && sensorHistoryCount[0] > 0) {
        int idx = (sensorHistoryIndex[0] - 1 + sensorHistoryCapacity) % sensorHistoryCapacity;
        Blynk.virtualWrite(V16, sensorHistory[0][idx].temp);
      }
      if (sensorHistory[1] != nullptr && sensorHistoryCount[1] > 0) {
        int idx = (sensorHistoryIndex[1] - 1 + sensorHistoryCapacity) % sensorHistoryCapacity;
        Blynk.virtualWrite(V17, sensorHistory[1][idx].temp);
      }
      if (sensorHistory[2] != nullptr && sensorHistoryCount[2] > 0) {
        int idx = (sensorHistoryIndex[2] - 1 + sensorHistoryCapacity) % sensorHistoryCapacity;
        Blynk.virtualWrite(V18, sensorHistory[2][idx].temp);
      }
      if (sensorHistory[3] != nullptr && sensorHistoryCount[3] > 0) {
        int idx = (sensorHistoryIndex[3] - 1 + sensorHistoryCapacity) % sensorHistoryCapacity;
        Blynk.virtualWrite(V19, sensorHistory[3][idx].temp);
      }

      if (sensorHistory[0] != nullptr && sensorHistoryCount[0] > 0) {
        int idx = (sensorHistoryIndex[0] - 1 + sensorHistoryCapacity) % sensorHistoryCapacity;
        float hum = sensorHistory[0][idx].hum;
        float temp = sensorHistory[0][idx].temp;
        float abshum =  (6.112 * pow(2.71828, ((17.67 * temp)/(temp + 243.5))) * hum * 2.1674)/(273.15 + temp);
        Blynk.virtualWrite(V21, abshum);
      }
      if (sensorHistory[1] != nullptr && sensorHistoryCount[1] > 0) {
        int idx = (sensorHistoryIndex[1] - 1 + sensorHistoryCapacity) % sensorHistoryCapacity;
        float hum = sensorHistory[1][idx].hum;
        float temp = sensorHistory[1][idx].temp;
        float abshum =  (6.112 * pow(2.71828, ((17.67 * temp)/(temp + 243.5))) * hum * 2.1674)/(273.15 + temp);
        Blynk.virtualWrite(V22, abshum);
      }
      if (sensorHistory[2] != nullptr && sensorHistoryCount[2] > 0) {
        int idx = (sensorHistoryIndex[2] - 1 + sensorHistoryCapacity) % sensorHistoryCapacity;
        float hum = sensorHistory[2][idx].hum;
        float temp = sensorHistory[2][idx].temp;
        float abshum =  (6.112 * pow(2.71828, ((17.67 * temp)/(temp + 243.5))) * hum * 2.1674)/(273.15 + temp);
        Blynk.virtualWrite(V23, abshum);
      }
      if (sensorHistory[3] != nullptr && sensorHistoryCount[3] > 0) {
        int idx = (sensorHistoryIndex[3] - 1 + sensorHistoryCapacity) % sensorHistoryCapacity;
        float hum = sensorHistory[3][idx].hum;
        float temp = sensorHistory[3][idx].temp;
        float abshum =  (6.112 * pow(2.71828, ((17.67 * temp)/(temp + 243.5))) * hum * 2.1674)/(273.15 + temp);
        Blynk.virtualWrite(V24, abshum);
      }
  }
 // every(60000){
 //   sendSliderValuesToWindTransmitter();
 // }
}