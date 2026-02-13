#include <lvgl.h>                            // lvgl version 9.2.2
#include <PINS_JC4827W543.h>                //dev device pins library
#include "TAMC_GT911.h"                     // TAMC_GT911 library
#include <Preferences.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <esp_efuse.h>
#include <esp_mac.h>
#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "time.h"
#include "esp_sntp.h"
#include <WiFiManager.h> 
#include "ArduinoOTA.h"
#include <BlynkSimpleEsp32.h>
#include <math.h>
bool credentialRequestReceived = false;
uint8_t requestingDeviceMAC[6] = {0};

#define MSG_MIN_INTERVAL 3000 
// Add near other globals
struct TimelinePoint {
  time_t timestamp;
  float temp1;
  float temp2;
  bool has_temp1;
  bool has_temp2;
};
TimelinePoint* g_currentTimeline = nullptr;
int g_currentTimelineCount = 0;
static uint8_t espnow_pmk[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
                                  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
static uint8_t espnow_lmk[16] = {0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA,
                                  0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22};

WiFiUDP udp;
const uint16_t UDP_PORT = 4210;
uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
  char ssid[32];
  char password[64];
  uint8_t magic[4];  // Magic bytes to identify valid credential packets
} wifi_credentials_t;



void sendWiFiCredentialsToDevice(uint8_t* targetMAC) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, cannot send credentials");
    return;
  }
  
  uint8_t currentChannel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&currentChannel, &second);
  Serial.printf("Sending credentials on WiFi channel: %d\n", currentChannel);
  
  String currentSSID = WiFi.SSID();
  String currentPSK = WiFi.psk();
  
  wifi_credentials_t creds;
  memset(&creds, 0, sizeof(creds));
  strncpy(creds.ssid, currentSSID.c_str(), sizeof(creds.ssid) - 1);
  strncpy(creds.password, currentPSK.c_str(), sizeof(creds.password) - 1);
  creds.magic[0] = 0xCA;
  creds.magic[1] = 0xFE;
  creds.magic[2] = 0xBA;
  creds.magic[3] = 0xBE;
  
  Serial.printf("Sending credentials to %02X:%02X:%02X:%02X:%02X:%02X\n",
                targetMAC[0], targetMAC[1], targetMAC[2],
                targetMAC[3], targetMAC[4], targetMAC[5]);
  
  if (esp_now_is_peer_exist(targetMAC)) {
    esp_now_del_peer(targetMAC);
  }
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, targetMAC, 6);
  peerInfo.channel = currentChannel;
  peerInfo.encrypt = false;  // ✅ TEMPORARY - test without encryption
  // memcpy(peerInfo.lmk, espnow_lmk, 16);  // Comment out
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Peer added successfully");
    
    for (int attempt = 0; attempt < 10; attempt++) {
      esp_err_t result = esp_now_send(targetMAC, (uint8_t *)&creds, sizeof(creds));
      Serial.printf("Attempt %d: %s\n", attempt + 1,
                    result == ESP_OK ? "OK" : "FAIL");
      delay(100);
    }
  } else {
    Serial.println("Failed to add peer!");
  }
  
  if (esp_now_is_peer_exist(targetMAC)) {
    esp_now_del_peer(targetMAC);
  }
  
  Serial.println("Credentials transmission complete");
}
// ---- squareline Studio exported UI headers ----
#include "ui.h"

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
// Fallback capacities and buffers if PSRAM not available
#define TEMP_FALLBACK_SIZE 128
#define WIND_HISTORY_FALLBACK_SIZE 256
#define NUM_TEMP_SENSORS 4
#define TEMP_HISTORY_SIZE 1440  // 24 hours at 1 minute intervals

struct SensorData {
  float temp;
  float hum;
  float pres;
  time_t timestamp;
};


SensorData* sensorHistory[NUM_TEMP_SENSORS] = {nullptr, nullptr, nullptr, nullptr};
int sensorHistoryIndex[NUM_TEMP_SENSORS] = {0};
int sensorHistoryCount[NUM_TEMP_SENSORS] = {0};
unsigned long sensorLastUpdate[NUM_TEMP_SENSORS] = {0};

// Wind sensor (sensor 0)
float s0temp = 999, s0hum = 0, s0pres = 0;
float s0windgust = 0, s0avgwind = 0, s0instwind = 999, s0winddirV = 0;
unsigned long s0LastUpdate = 0;

// Current temperature values for display
float sensorTemps[NUM_TEMP_SENSORS] = {999, 999, 999, 999};

// Chart series for each sensor
lv_chart_series_t* temp_chart_series[NUM_TEMP_SENSORS] = {nullptr, nullptr, nullptr, nullptr};
bool seriesVisible[NUM_TEMP_SENSORS] = {true, true, true, true};  // All visible by default

// Legend objects
lv_obj_t* legend_container = NULL;
lv_obj_t* legend_labels[NUM_TEMP_SENSORS] = {nullptr, nullptr, nullptr, nullptr};

// Colors for each sensor
lv_color_t sensorColors[NUM_TEMP_SENSORS] = {
  lv_palette_main(LV_PALETTE_RED),
  lv_palette_main(LV_PALETTE_BLUE),
  lv_palette_main(LV_PALETTE_GREEN),
  lv_palette_main(LV_PALETTE_ORANGE)
};

const char* sensorNames[NUM_TEMP_SENSORS] = {
  "Sensor 1",
  "Sensor 2", 
  "Sensor 3",
  "Sensor 4"
};

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
#define SFE_WMK_ADC_ANGLE_0_0 3080
#define SFE_WMK_ADC_ANGLE_22_5 1560
#define SFE_WMK_ADC_ANGLE_45_0 1781
#define SFE_WMK_ADC_ANGLE_67_5 290
#define SFE_WMK_ADC_ANGLE_90_0 330
#define SFE_WMK_ADC_ANGLE_112_5 230
#define SFE_WMK_ADC_ANGLE_135_0 680
#define SFE_WMK_ADC_ANGLE_157_5 460
#define SFE_WMK_ADC_ANGLE_180_0 1080
#define SFE_WMK_ADC_ANGLE_202_5 910
#define SFE_WMK_ADC_ANGLE_225_0 2420
#define SFE_WMK_ADC_ANGLE_247_5 2300
#define SFE_WMK_ADC_ANGLE_270_0 3954
#define SFE_WMK_ADC_ANGLE_292_5 3290
#define SFE_WMK_ADC_ANGLE_315_0 3586
#define SFE_WMK_ADC_ANGLE_337_5 2712
#define SFE_WMK_ADC_RESOLUTION 12

typedef struct {
  uint8_t magic[4];  // Magic bytes: 0xDE, 0xAD, 0xBE, 0xEF
} credential_request_t;

typedef struct {
  float sliderAvg;
  float sliderGust;
  uint8_t magic[4];  // Magic bytes: 0xAB, 0xCD, 0xEF, 0x12
} slider_values_t;

// Add these global variables:
float sliderAvg = 10.0;   // Default slider values
float sliderGust = 20.0;

struct SFEWeatherMeterKitCalibrationParams
{
    uint16_t vaneADCValues[WMK_NUM_ANGLES];
};

SFEWeatherMeterKitCalibrationParams _calibrationParams;

// Wind Rose data structures
#define WIND_DIRECTIONS 16  // 16 compass directions (N, NNE, NE, ENE, E, etc.)
#define WIND_SPEED_BINS 5   // 5 speed ranges: 0-8, 8-16, 16-24, 24-32, >32
#define WIND_HISTORY_SIZE 1440  // 24 hours at 1 minute intervals

struct WindDataPoint {
    float speed;
    float direction;
    unsigned long timestamp;
};

// Wind rose histogram: [direction][speed_bin]
int *windRoseData[WIND_DIRECTIONS];
WindDataPoint *windHistory = nullptr;
int windHistoryIndex = 0;
int windHistoryCount = 0;

struct TempDataPoint {
    float temp;      // s1temp
    float temp2;      // s2temp
    unsigned long timestamp;
};

static TempDataPoint tempHistory1Fallback[TEMP_FALLBACK_SIZE];
static TempDataPoint tempHistory2Fallback[TEMP_FALLBACK_SIZE];
static WindDataPoint windHistoryFallback[WIND_HISTORY_FALLBACK_SIZE];
static int windRoseFallback[WIND_DIRECTIONS][WIND_SPEED_BINS];

// dynamic capacities used at runtime
int tempHistoryCapacity = 0;
int windHistoryCapacity = WIND_HISTORY_SIZE;


char auth[] = "19oL8t8mImCdoUqYhfhk6DADL7540f8s";
int hours, mins, secs;
unsigned long localTimeUnix = 0; 
unsigned long s1LastUpdate = 0;
struct tm timeinfo;
bool isSetNtp = false;


#define TEMP_HISTORY_SIZE 500
TempDataPoint *tempHistory1 = nullptr;  // For s1temp
TempDataPoint *tempHistory2 = nullptr;  // For s2temp
int tempHistoryIndex1 = 0;
int tempHistoryIndex2 = 0;
int tempHistoryCount1 = 0;
int tempHistoryCount2 = 0;




// Function declarations
void cbSyncTime(struct timeval *tv);
void initSNTP();
void setTimezone();
void onESPNowReceive(const esp_now_recv_info_t * info, const uint8_t *data, int len);
void addTempToHistory(float temp, float hum, float pres, int sensorId);
void drawTempGraph();
void drawWindSpeedGraph();
void addWindData(float speed, float direction);
void drawWindRose();
void setup();
void loop();
float getWindDirection();
static void legend_click_cb(lv_event_t* e);
void createLegend();

// Create legend
void createLegend() {
  if (legend_container != NULL) return;
  
  lv_obj_t* parent = lv_obj_get_parent(ui_Chart1);
  
  legend_container = lv_obj_create(parent);
  lv_obj_set_size(legend_container, 200, 80);
  lv_obj_align_to(legend_container, ui_Chart1, LV_ALIGN_OUT_RIGHT_TOP, 10, 0);
  lv_obj_set_style_bg_opa(legend_container, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(legend_container, 0, 0);
  lv_obj_set_style_pad_all(legend_container, 5, 0);
  lv_obj_set_flex_flow(legend_container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(legend_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  
  for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
    legend_labels[i] = lv_label_create(legend_container);
    lv_label_set_text(legend_labels[i], sensorNames[i]);
    lv_obj_set_style_text_color(legend_labels[i], sensorColors[i], 0);
    lv_obj_set_style_text_font(legend_labels[i], &lv_font_montserrat_12, 0);
    lv_obj_add_flag(legend_labels[i], LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(legend_labels[i], legend_click_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_pad_all(legend_labels[i], 3, 0);
  }
}

// Legend click callback
static void legend_click_cb(lv_event_t* e) {
  lv_obj_t* label = (lv_obj_t*)lv_event_get_target(e);
  
  // Find which sensor this label belongs to
  for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
    if (legend_labels[i] == label) {
      // Toggle visibility
      seriesVisible[i] = !seriesVisible[i];
      
      // Update label appearance
      if (seriesVisible[i]) {
        lv_obj_set_style_text_color(label, sensorColors[i], 0);
        lv_obj_set_style_text_opa(label, LV_OPA_COVER, 0);
      } else {
        lv_obj_set_style_text_color(label, lv_color_hex(0x808080), 0);
        lv_obj_set_style_text_opa(label, LV_OPA_50, 0);
      }
      
      // Redraw chart
      drawTempGraph();
      break;
    }
  }
}
// Add this helper function near the top with other function declarations
const char* getCardinalDirection(float degrees);

// Implement the function (add this before setup() or after getWindDirection())
const char* getCardinalDirection(float degrees) {
    // Normalize degrees to 0-360
    while (degrees < 0) degrees += 360;
    while (degrees >= 360) degrees -= 360;
    
    // 16-point compass rose
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


lv_obj_t * temp_container = NULL;
// Temperature chart label popup (simpler overlay approach for LVGL v9.3)
//lv_chart_series_t * temp_chart_series = NULL;
lv_chart_series_t * temp_chart_series2 = NULL;  // ADD THIS LINE
static lv_obj_t * temp_label_popup = NULL;
static lv_obj_t * temp_scale_y = NULL;
static lv_obj_t * time_labels[5] = {NULL, NULL, NULL, NULL, NULL};

static void temp_chart_draw_label_cb(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * chart = (lv_obj_t*) lv_event_get_target(e);
  
  if(code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING) {
    int32_t id = lv_chart_get_pressed_point(chart);
    
    if(id == LV_CHART_POINT_NONE) {
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    int chartPoints = lv_chart_get_point_count(chart);
    if (id < 0 || id >= chartPoints) {
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    // Create label popup if needed
    if(temp_label_popup == NULL) {
      temp_label_popup = lv_label_create(lv_scr_act());
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
    
    // Get time range for this point
    time_t earliest_time = UINT32_MAX;
    time_t latest_time = 0;
    
    for (int s = 0; s < NUM_TEMP_SENSORS; s++) {
      if (sensorHistory[s] == nullptr || sensorHistoryCount[s] == 0) continue;
      
      int startIdx = (sensorHistoryIndex[s] - sensorHistoryCount[s] + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
      int endIdx = (sensorHistoryIndex[s] - 1 + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
      
      time_t first = sensorHistory[s][startIdx].timestamp;
      time_t last = sensorHistory[s][endIdx].timestamp;
      
      if (first > 0 && first < earliest_time) earliest_time = first;
      if (last > latest_time) latest_time = last;
    }
    
    time_t time_span = latest_time - earliest_time;
    if (time_span <= 0) time_span = 60;
    
    // Calculate timestamp for this chart point
    time_t point_time = earliest_time + ((time_span * id) / (chartPoints - 1));
    
    // Find data from each sensor near this timestamp (within ±30 seconds for minute normalization)
    static char buf[256];
    char time_str[32] = "";
    int line_count = 0;
    
    if (point_time > 0 && point_time >= 1000000000) {
      struct tm timeinfo_buf;
      struct tm* timeinfo = localtime_r(&point_time, &timeinfo_buf);
      
      if (timeinfo != NULL) {
        int hour = timeinfo->tm_hour;
        int min = timeinfo->tm_min;
        const char* ampm = (hour >= 12) ? "PM" : "AM";
        if (hour > 12) hour -= 12;
        else if (hour == 0) hour = 12;
        snprintf(time_str, sizeof(time_str), "%d:%02d %s", hour, min, ampm);
      }
    }
    
    buf[0] = '\0';  // Clear buffer
    
    // Check each sensor for data at this time
    for (int s = 0; s < NUM_TEMP_SENSORS; s++) {
      if (!seriesVisible[s] || sensorHistory[s] == nullptr || sensorHistoryCount[s] == 0) {
        continue;
      }
      
      // Find closest data point within ±30 seconds
      int startIdx = (sensorHistoryIndex[s] - sensorHistoryCount[s] + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
      float closest_temp = -999.0;
      time_t closest_time_diff = UINT32_MAX;
      
      for (int i = 0; i < sensorHistoryCount[s]; i++) {
        int idx = (startIdx + i) % TEMP_HISTORY_SIZE;
        time_t ts = sensorHistory[s][idx].timestamp;
        time_t diff = (ts > point_time) ? (ts - point_time) : (point_time - ts);
        
        if (diff < closest_time_diff && diff <= 30) {  // Within 30 seconds
          closest_time_diff = diff;
          closest_temp = sensorHistory[s][idx].temp;
        }
      }
      
      if (closest_temp > -999.0) {
        char line[64];
        snprintf(line, sizeof(line), "%s: %.2f°C\n", sensorNames[s], closest_temp);
        strcat(buf, line);
        line_count++;
      }
    }
    
    if (line_count > 0) {
      strcat(buf, time_str);
      lv_label_set_text(temp_label_popup, buf);
      
      // Position label
      lv_chart_series_t * ser = lv_chart_get_series_next(chart, NULL);
      if(ser == NULL) return;
      
      lv_point_t p;
      lv_chart_get_point_pos_by_id(chart, ser, id, &p);
      
      lv_area_t chart_coords;
      lv_obj_get_coords(chart, &chart_coords);
      
      lv_obj_update_layout(temp_label_popup);
      lv_coord_t label_width = lv_obj_get_width(temp_label_popup);
      lv_coord_t label_height = lv_obj_get_height(temp_label_popup);
      
      int32_t label_x = chart_coords.x1 + p.x - (label_width / 2);
      int32_t label_y = chart_coords.y1 + p.y - label_height - 10;
      
      int32_t screen_width = lv_obj_get_width(lv_scr_act());
      int32_t screen_height = lv_obj_get_height(lv_scr_act());
      
      if (label_x < 0) label_x = 0;
      if (label_x + label_width > screen_width) label_x = screen_width - label_width;
      if (label_y < 0) label_y = 0;
      if (label_y + label_height > screen_height) label_y = screen_height - label_height;
      
      lv_obj_set_pos(temp_label_popup, label_x, label_y);
      lv_obj_clear_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
    }
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


// Queue and message for ESP-NOW processing
typedef struct {
  uint8_t mac[6];
  uint8_t payload[32];
  int len;
} EspNowMsg;

static QueueHandle_t espNowQueue = NULL;
volatile uint32_t sensor1MsgCount = 0;
volatile uint32_t sensor2MsgCount = 0;
volatile uint32_t espNowQueueDroppedCount = 0;



float getWindDirection()
{
    uint16_t rawADC = s0winddirV;
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

// Updated payload structures
typedef struct {
  uint8_t sensor_id;
  float temp;
  float hum;
  float pres;
  uint8_t magic[4];  // "TEMP"
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
  uint8_t magic[4];  // "WIND"
} esp_now_payload_wind_t;

time_t normalizeToMinute(time_t timestamp);

time_t normalizeToMinute(time_t timestamp) {
  return (timestamp / 60) * 60;  // Round down to nearest minute
}

float s1temp = 999;
float s1hum, s1pres, s1vBat;


void handleUDPData();

// Updated handleUDPData function
void handleUDPData() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    uint8_t buffer[256];
    int len = udp.read(buffer, sizeof(buffer));
    
    // Check for wind sensor data
    if (len == sizeof(esp_now_payload_wind_t)) {
      esp_now_payload_wind_t payload;
      memcpy(&payload, buffer, sizeof(payload));
      
      // Verify magic bytes for wind data
      if (payload.magic[0] == 0x57 && payload.magic[1] == 0x49 &&
          payload.magic[2] == 0x4E && payload.magic[3] == 0x44) {
        
        s0LastUpdate = millis();
        s0temp = payload.temp;
        s0hum = payload.hum;
        s0pres = payload.pres;
        s0windgust = payload.windgust;
        s0avgwind = payload.avgwind;
        s0instwind = payload.instwind;
        s0winddirV = payload.winddirV;
        
        float windDir = getWindDirection();
        addWindData(s0avgwind, windDir);
        addTempToHistory(payload.temp, payload.hum, payload.pres, 3);
        Serial.printf("Wind Sensor: temp=%.1f, wind=%.1f, dir=%.1f°\n", 
                      s0temp, s0avgwind, windDir);
      }
    }
    // Check for temperature sensor data
    else if (len == sizeof(temp_sensor_payload_t)) {
      temp_sensor_payload_t payload;
      memcpy(&payload, buffer, sizeof(payload));
      
      // Verify magic bytes for temp data
      if (payload.magic[0] == 0x54 && payload.magic[1] == 0x45 &&
          payload.magic[2] == 0x4D && payload.magic[3] == 0x50) {
        
        int sensorId = payload.sensor_id - 1;  // Convert 1-4 to 0-3
        
        if (sensorId >= 0 && sensorId < NUM_TEMP_SENSORS) {
          sensorLastUpdate[sensorId] = millis();
          sensorTemps[sensorId] = payload.temp;
          
          addTempToHistory(payload.temp, payload.hum, payload.pres, sensorId);
          
          Serial.printf("Sensor %d: temp=%.2f°C, hum=%.2f%%, pres=%.2fhPa\n",
                        payload.sensor_id, payload.temp, payload.hum, payload.pres);
        }
      }
    }
    // Check for slider values
    else if (len == sizeof(slider_values_t)) {
      slider_values_t sliders;
      memcpy(&sliders, buffer, sizeof(sliders));
      
      if (sliders.magic[0] == 0xAB && sliders.magic[1] == 0xCD &&
          sliders.magic[2] == 0xEF && sliders.magic[3] == 0x12) {
        // Handle slider values if needed
      }
    }
  }
}

#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))


void addWindData(float speed, float direction) {
    if (windHistory == nullptr) return;
    
    // Add to circular buffer with timestamp
    windHistory[windHistoryIndex].speed = speed;
    windHistory[windHistoryIndex].direction = direction;
    windHistory[windHistoryIndex].timestamp = millis();
    
    windHistoryIndex = (windHistoryIndex + 1) % windHistoryCapacity;
    if (windHistoryCount < windHistoryCapacity) {
        windHistoryCount++;
    }
    
    // Find min/max in actual wind data
    float minVal = 1000.0;
    float maxVal = 0.0;
    int startIdx = (windHistoryIndex - windHistoryCount + windHistoryCapacity) % windHistoryCapacity;
    
    for (int i = 0; i < windHistoryCount; i++) {
        int idx = (startIdx + i) % windHistoryCapacity;
        float val = windHistory[idx].speed;
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
    }
    
    // Ensure minimum range
    if (minVal > 0) minVal = 0.0;  // Wind speed starts at 0
    float range = maxVal - minVal;
    if (range < 10.0) range = 10.0;
    
    // Rebuild wind rose histogram
    for (int i = 0; i < WIND_DIRECTIONS; i++) {
        if (windRoseData[i] != nullptr) {
            for (int j = 0; j < WIND_SPEED_BINS; j++) {
                windRoseData[i][j] = 0;
            }
        }
    }
    
    for (int i = 0; i < windHistoryCount; i++) {
        int idx = (startIdx + i) % windHistoryCapacity;
        float spd = windHistory[idx].speed;
        float dir = windHistory[idx].direction;
        int dirBin = (int)((dir + 11.25) / 22.5) % WIND_DIRECTIONS;
        int spdBin;
        if (spd < 8) spdBin = 0;
        else if (spd < 16) spdBin = 1;
        else if (spd < 24) spdBin = 2;
        else if (spd < 32) spdBin = 3;
        else spdBin = 4;
        if (windRoseData[dirBin] != nullptr) {
            windRoseData[dirBin][spdBin]++;
        }
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

// Add temperature data to history with minute normalization
void addTempToHistory(float temp, float hum, float pres, int sensorId) {
  if (sensorId < 0 || sensorId >= NUM_TEMP_SENSORS) return;
  if (sensorHistory[sensorId] == nullptr) return;
  
  time_t now = time(nullptr);
  time_t normalized = normalizeToMinute(now);
  
  // Check if we already have data for this minute
  int currentIdx = sensorHistoryIndex[sensorId];
  int prevIdx = (currentIdx - 1 + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
  
  if (sensorHistoryCount[sensorId] > 0 && 
      sensorHistory[sensorId][prevIdx].timestamp == normalized) {
    // Update existing minute's data (average if desired, or just replace)
    sensorHistory[sensorId][prevIdx].temp = temp;
    sensorHistory[sensorId][prevIdx].hum = hum;
    sensorHistory[sensorId][prevIdx].pres = pres;
    return;
  }
  
  // Add new entry
  sensorHistory[sensorId][currentIdx].temp = temp;
  sensorHistory[sensorId][currentIdx].hum = hum;
  sensorHistory[sensorId][currentIdx].pres = pres;
  sensorHistory[sensorId][currentIdx].timestamp = normalized;
  
  sensorHistoryIndex[sensorId] = (currentIdx + 1) % TEMP_HISTORY_SIZE;
  if (sensorHistoryCount[sensorId] < TEMP_HISTORY_SIZE) {
    sensorHistoryCount[sensorId]++;
  }
}
void drawTempGraph() {
  if (ui_Chart1 == NULL) return;
  
  // Count total data points across all sensors
  int totalPoints = 0;
  for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
    if (sensorHistoryCount[i] > totalPoints) {
      totalPoints = sensorHistoryCount[i];
    }
  }
  
  if (totalPoints == 0) return;
  
  // Initialize chart series on first call
  if (temp_chart_series[0] == NULL) {
    // Remove default series
    lv_chart_remove_series(ui_Chart1, lv_chart_get_series_next(ui_Chart1, NULL));
    
    // Create series for each sensor
    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
      temp_chart_series[i] = lv_chart_add_series(ui_Chart1, sensorColors[i], LV_CHART_AXIS_PRIMARY_Y);
    }
    
    lv_chart_set_type(ui_Chart1, LV_CHART_TYPE_LINE);
    lv_obj_set_style_line_width(ui_Chart1, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(ui_Chart1, 4, 4, LV_PART_INDICATOR);
    
    lv_obj_add_event_cb(ui_Chart1, temp_chart_draw_label_cb, LV_EVENT_ALL, NULL);
    
    lv_obj_t* parent = lv_obj_get_parent(ui_Chart1);
    
    // Create Y-axis scale
    temp_scale_y = lv_scale_create(parent);
    lv_obj_set_size(temp_scale_y, 38, lv_obj_get_height(ui_Chart1));
    lv_obj_align_to(temp_scale_y, ui_Chart1, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_scale_set_mode(temp_scale_y, LV_SCALE_MODE_VERTICAL_LEFT);
    lv_scale_set_label_show(temp_scale_y, true);
    lv_obj_set_style_length(temp_scale_y, 5, LV_PART_ITEMS);
    lv_obj_set_style_length(temp_scale_y, 10, LV_PART_INDICATOR);
    lv_obj_set_style_text_font(temp_scale_y, &lv_font_montserrat_10, 0);
    lv_obj_set_style_pad_all(temp_scale_y, 0, 0);
    
    // Create time labels
    int chart_width = lv_obj_get_width(ui_Chart1);
    int label_spacing = chart_width / 4;
    
    for (int i = 0; i < 5; i++) {
      time_labels[i] = lv_label_create(parent);
      lv_label_set_text(time_labels[i], "--:--");
      lv_obj_set_style_text_font(time_labels[i], &lv_font_montserrat_8, 0);
      int x_offset = (i * label_spacing) - (chart_width / 2);
      lv_obj_align_to(time_labels[i], ui_Chart1, LV_ALIGN_OUT_BOTTOM_MID, x_offset, 5);
    }
    
    // Create legend
    createLegend();
    
    Serial.println("Chart initialized with 4 sensor series");
  }
  
  // Determine display points (limit to reasonable amount)
  int displayPoints = totalPoints;
  if (displayPoints > 100) displayPoints = 100;
  if (displayPoints < 2) displayPoints = 2;
  
  lv_chart_set_point_count(ui_Chart1, displayPoints);
  
  // Find global min/max temperature across all visible sensors
  float minTemp = 999.0;
  float maxTemp = -999.0;
  
  for (int s = 0; s < NUM_TEMP_SENSORS; s++) {
    if (!seriesVisible[s] || sensorHistory[s] == nullptr || sensorHistoryCount[s] == 0) {
      continue;
    }
    
    int startIdx = (sensorHistoryIndex[s] - sensorHistoryCount[s] + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
    for (int i = 0; i < sensorHistoryCount[s]; i++) {
      int idx = (startIdx + i) % TEMP_HISTORY_SIZE;
      float temp = sensorHistory[s][idx].temp;
      if (temp < minTemp) minTemp = temp;
      if (temp > maxTemp) maxTemp = temp;
    }
  }
  
  // Ensure minimum range
  if (minTemp > maxTemp) {
    minTemp = 20.0;
    maxTemp = 25.0;
  }
  
  float range = maxTemp - minTemp;
  if (range < 5.0) {
    float center = (minTemp + maxTemp) / 2.0;
    minTemp = center - 2.5;
    maxTemp = center + 2.5;
  } else {
    minTemp -= 1.0;
    maxTemp += 1.0;
  }
  
  // Get time range from all sensors
  time_t earliest_time = UINT32_MAX;
  time_t latest_time = 0;
  
  for (int s = 0; s < NUM_TEMP_SENSORS; s++) {
    if (sensorHistory[s] == nullptr || sensorHistoryCount[s] == 0) continue;
    
    int startIdx = (sensorHistoryIndex[s] - sensorHistoryCount[s] + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
    int endIdx = (sensorHistoryIndex[s] - 1 + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
    
    time_t first = sensorHistory[s][startIdx].timestamp;
    time_t last = sensorHistory[s][endIdx].timestamp;
    
    if (first > 0 && first < earliest_time) earliest_time = first;
    if (last > latest_time) latest_time = last;
  }
  
  time_t time_span = latest_time - earliest_time;
  if (time_span <= 0) time_span = 60;  // At least 1 minute
  
  // Clear all points
  for (int s = 0; s < NUM_TEMP_SENSORS; s++) {
    for (int i = 0; i < displayPoints; i++) {
      lv_chart_set_value_by_id(ui_Chart1, temp_chart_series[s], i, LV_CHART_POINT_NONE);
    }
  }
  
  // Plot each sensor's data at time-proportional positions
  for (int s = 0; s < NUM_TEMP_SENSORS; s++) {
    if (!seriesVisible[s] || sensorHistory[s] == nullptr || sensorHistoryCount[s] == 0) {
      continue;
    }
    
    int startIdx = (sensorHistoryIndex[s] - sensorHistoryCount[s] + TEMP_HISTORY_SIZE) % TEMP_HISTORY_SIZE;
    
    for (int i = 0; i < sensorHistoryCount[s]; i++) {
      int idx = (startIdx + i) % TEMP_HISTORY_SIZE;
      time_t ts = sensorHistory[s][idx].timestamp;
      float temp = sensorHistory[s][idx].temp;
      
      // Calculate X position based on time
      int x_pos = 0;
      if (time_span > 0) {
        x_pos = ((ts - earliest_time) * (displayPoints - 1)) / time_span;
      }
      
      if (x_pos >= 0 && x_pos < displayPoints) {
        lv_chart_set_value_by_id(ui_Chart1, temp_chart_series[s], x_pos, (int32_t)(temp * 100));
      }
    }
  }
  
  // Update Y-axis range
  lv_chart_set_range(ui_Chart1, LV_CHART_AXIS_PRIMARY_Y, 
                      (int32_t)(minTemp * 100), (int32_t)(maxTemp * 100));
  
  if (temp_scale_y != NULL) {
    int totalTicks = (int)(maxTemp - minTemp) + 1;
    lv_scale_set_total_tick_count(temp_scale_y, totalTicks);
    if (totalTicks < 10) {
      lv_scale_set_major_tick_every(temp_scale_y, 1);
    } else {
      lv_scale_set_major_tick_every(temp_scale_y, totalTicks / 5);
    }
    lv_scale_set_range(temp_scale_y, (int32_t)minTemp, (int32_t)maxTemp);
  }
  
  // Update X-axis time labels
  for (int i = 0; i < 5; i++) {
    if (time_labels[i] == NULL) continue;
    
    time_t label_time = earliest_time + (time_span * i / 4);
    
    if (label_time > 0 && label_time >= 1000000000) {
      struct tm timeinfo_buf;
      struct tm* timeinfo = localtime_r(&label_time, &timeinfo_buf);
      
      if (timeinfo != NULL) {
        int hour = timeinfo->tm_hour;
        int min = timeinfo->tm_min;
        const char* ampm = (hour >= 12) ? "PM" : "AM";
        if (hour > 12) hour -= 12;
        else if (hour == 0) hour = 12;
        
        char time_buf[16];
        snprintf(time_buf, sizeof(time_buf), "%d:%02d%s", hour, min, ampm);
        lv_label_set_text(time_labels[i], time_buf);
      }
    }
  }
  
  lv_chart_refresh(ui_Chart1);
}
void onESPNowReceive(const esp_now_recv_info_t * info, const uint8_t *data, int len)
{
  // Log EVERY message received
  Serial.println("========================================");
  Serial.printf("📨 ESP-NOW message received! Length: %d\n", len);
  Serial.printf("From MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                info->src_addr[0], info->src_addr[1], info->src_addr[2],
                info->src_addr[3], info->src_addr[4], info->src_addr[5]);
  
  // Print raw data
  Serial.print("Raw data: ");
  for (int i = 0; i < len && i < 32; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();
  
  // Check for credential request
  if (len == sizeof(credential_request_t)) {
    Serial.printf("Length matches credential_request_t (%d bytes)\n", sizeof(credential_request_t));
    
    credential_request_t request;
    memcpy(&request, data, sizeof(request));
    
    Serial.printf("Magic bytes: %02X %02X %02X %02X\n", 
                  request.magic[0], request.magic[1], 
                  request.magic[2], request.magic[3]);
    
    // Verify magic bytes
    if (request.magic[0] == 0xDE && request.magic[1] == 0xAD &&
        request.magic[2] == 0xBE && request.magic[3] == 0xEF) {
      
      // Store requesting device MAC and set flag
      memcpy(requestingDeviceMAC, info->src_addr, 6);
      credentialRequestReceived = true;
      
      Serial.println("✅ VALID Credential request received!");
      Serial.printf("Will respond to: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    requestingDeviceMAC[0], requestingDeviceMAC[1], requestingDeviceMAC[2],
                    requestingDeviceMAC[3], requestingDeviceMAC[4], requestingDeviceMAC[5]);
      Serial.println("========================================");
      return;
    } else {
      Serial.println("❌ Invalid magic bytes for credential request");
    }
  } else {
    Serial.printf("Length mismatch - expected %d, got %d\n", sizeof(credential_request_t), len);
  }
  
  Serial.println("========================================");
  
  // Original ESP-NOW message handling for sensor data
  EspNowMsg msg;
  memcpy(msg.mac, info->src_addr, 6);
  msg.len = (len > (int)sizeof(msg.payload)) ? sizeof(msg.payload) : len;
  memcpy(msg.payload, data, msg.len);
  BaseType_t ok = pdFALSE;
  if (espNowQueue != NULL) {
    ok = xQueueSend(espNowQueue, &msg, 0);
  }
  if (ok != pdTRUE) {
    espNowQueueDroppedCount++;
  }
}


void sendSliderValues();

void sendSliderValues() {
  if (WiFi.status() != WL_CONNECTED) return;
  
  slider_values_t sliders;
  sliders.sliderAvg = g_sliderAvgValue;
  sliders.sliderGust = g_sliderGustValue;
  sliders.magic[0] = 0xAB;
  sliders.magic[1] = 0xCD;
  sliders.magic[2] = 0xEF;
  sliders.magic[3] = 0x12;
  
  // Broadcast to subnet
  IPAddress broadcastIP = WiFi.localIP();
  broadcastIP[3] = 255;
  
  udp.beginPacket(broadcastIP, UDP_PORT);
  udp.write((uint8_t *)&sliders, sizeof(sliders));
  udp.endPacket();
  
  Serial.printf("Sent slider values - Avg: %.1f, Gust: %.1f\n", g_sliderAvgValue, g_sliderGustValue);
  
  // Clear the flag
  g_sliderValuesChanged = false;
}



// ---- LVGL Globals ----
uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_display_t *disp;
lv_color_t *disp_draw_buf;

// -----------------------------------------------------------------------------
//  LVGL Helper Functions
// -----------------------------------------------------------------------------
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
    
    // Wind rose parameters - centered at x=169, y=196 on screen
    static lv_obj_t * wind_rose_canvas = NULL;
    static lv_draw_buf_t draw_buf;
    static bool buf_initialized = false;
    
    const int16_t canvasWidth = 180;
    const int16_t canvasHeight = 180;
    const int16_t centerX = canvasWidth / 2;   // Center within canvas
    const int16_t centerY = canvasHeight / 2;
    const int16_t maxRadius = 57;  // Maximum spoke length
    
    // Color mapping for speed bins
    lv_color_t speedColors[WIND_SPEED_BINS] = {
        lv_color_hex(0x00FFFF),  // Cyan: 0-8 kph
        lv_color_hex(0x00FF00),  // Green: 8-16 kph
        lv_color_hex(0xFFFF00),  // Yellow: 16-24 kph
        lv_color_hex(0xFF8800),  // Orange: 24-32 kph
        lv_color_hex(0xFF0000)   // Red: >32 kph
    };
    
    // Create canvas on first call
    if (wind_rose_canvas == NULL) {
        wind_rose_canvas = lv_canvas_create(lv_scr_act());
        
        // Allocate and initialize draw buffer (ARGB8888 for transparency)
        uint32_t buf_size = canvasWidth * canvasHeight * 4;  // 4 bytes per pixel
        void * buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (buf == NULL) {
            buf = malloc(buf_size);
        }
        
        if (buf != NULL) {
            lv_draw_buf_init(&draw_buf, canvasWidth, canvasHeight, LV_COLOR_FORMAT_ARGB8888, 0, buf, buf_size);
            lv_canvas_set_draw_buf(wind_rose_canvas, &draw_buf);
            lv_obj_set_pos(wind_rose_canvas, 172 - centerX, 196 - centerY);
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
    
    // Clear canvas with transparency
    lv_canvas_fill_bg(wind_rose_canvas, lv_color_black(), LV_OPA_TRANSP);
    
    // Calculate total count per direction and max frequency
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
    
    if (maxDirectionCount == 0) return;  // No data to display
    
    // Initialize layer for drawing
    lv_layer_t layer;
    lv_canvas_init_layer(wind_rose_canvas, &layer);
    
    // Draw wind rose spokes - each direction is a spoke broken into speed bands
    for (int dir = 0; dir < WIND_DIRECTIONS; dir++) {
        if (windRoseData[dir] == nullptr || directionTotals[dir] == 0) continue;
        
        // Calculate spoke length based on frequency (proportional to max)
        float spokeLength = (float)directionTotals[dir] / (float)maxDirectionCount * maxRadius;
        
        // Calculate angle for this direction (dir 0 = N = 0°, clockwise)
        // Subtract 90° to rotate the display so North points up
        float dirAngle = (dir * 22.5f) - 90.0f;
        
        // Draw speed bands from inside out
        float currentRadius = 0;
        
        for (int spd = 0; spd < WIND_SPEED_BINS; spd++) {
            int count = windRoseData[dir][spd];
            if (count == 0) continue;
            
            // Calculate this band's contribution to the spoke length
            float bandLength = (float)count / (float)directionTotals[dir] * spokeLength;
            float innerRadius = currentRadius;
            float outerRadius = currentRadius + bandLength;
            
            // Draw arc segment for this speed band
            // Each spoke gets ~22.5° width (360/16)
            int16_t startAngle = (int16_t)(dirAngle - 11.25f);
            int16_t endAngle = (int16_t)(dirAngle + 11.25f);
            
            // Normalize angles
            while (startAngle < 0) startAngle += 360;
            while (endAngle < 0) endAngle += 360;
            
            // We need to draw the band as an arc segment
            // Draw multiple arcs to fill the band from innerRadius to outerRadius
            int numArcs = (int)(bandLength / 2) + 1;  // Number of arcs to fill the band
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

void sliderAvg_event_handler(lv_event_t * e) {
  lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
  sliderAvg = (float)lv_slider_get_value(slider);
  sendSliderValues();
}

void sliderGust_event_handler(lv_event_t * e) {
  lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
  sliderGust = (float)lv_slider_get_value(slider);
  sendSliderValues();
}

// -----------------------------------------------------------------------------
//  SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  //while (!Serial) ;
  Serial.println("Starting...");
  
  // Initialize wind vane calibration
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
    if (psramFound()) {
    Serial.println("PSRAM found, allocating sensor histories...");
    
    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
      sensorHistory[i] = (SensorData*)ps_malloc(TEMP_HISTORY_SIZE * sizeof(SensorData));
      
      if (sensorHistory[i] == nullptr) {
        Serial.printf("Failed to allocate PSRAM for sensor %d\n", i + 1);
      } else {
        Serial.printf("Sensor %d history allocated in PSRAM\n", i + 1);
        for (int j = 0; j < TEMP_HISTORY_SIZE; j++) {
          sensorHistory[i][j].temp = 0.0;
          sensorHistory[i][j].hum = 0.0;
          sensorHistory[i][j].pres = 0.0;
          sensorHistory[i][j].timestamp = 0;
        }
      }
    }
    
    // Allocate wind history (existing code)
    windHistory = (WindDataPoint*)ps_malloc(WIND_HISTORY_SIZE * sizeof(WindDataPoint));
    if (windHistory == nullptr) {
      Serial.println("Failed to allocate PSRAM for wind history");
      windHistory = windHistoryFallback;
      windHistoryCapacity = WIND_HISTORY_FALLBACK_SIZE;
    } else {
      Serial.println("Wind history allocated in PSRAM");
      windHistoryCapacity = WIND_HISTORY_SIZE;
      for (int i = 0; i < windHistoryCapacity; i++) {
        windHistory[i].speed = 0.0;
        windHistory[i].direction = 0.0;
        windHistory[i].timestamp = 0;
      }
    }
    
    // Allocate wind rose data
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
  } else  {
    Serial.println("PSRAM not found! Using SRAM fallbacks (limited capacity)");
    tempHistory1 = tempHistory1Fallback;
    tempHistory2 = tempHistory2Fallback;
    tempHistoryCapacity = TEMP_FALLBACK_SIZE;
    for (int i = 0; i < tempHistoryCapacity; i++) {
      tempHistory1[i].temp = 0.0;
      tempHistory1[i].timestamp = 0;
      tempHistory2[i].temp = 0.0;
      tempHistory2[i].timestamp = 0;
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
        Serial.println("❌ gfx->begin() failed!");
        while (true);
    }
    gfx->invertDisplay(false);   // first make sure inversion isn't on
    
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);   // turn on backlight
    gfx->fillScreen(RGB565_BLACK);

    // ---- Initialize Touch ----
    touchController.begin();
    touchController.setRotation(ROTATION_INVERTED);

    // ---- Initialize LVGL ----
    lv_init();
    lv_tick_set_cb(millis_cb);

#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print);
#endif

    screenWidth = gfx->width();
    screenHeight = gfx->height();
    bufSize = screenWidth * 40; // Partial buffer for speed/memory balance

    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!disp_draw_buf)
        disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);

    if (!disp_draw_buf) {
        Serial.println("❌ LVGL disp_draw_buf allocation failed!");
        while (true);
    }

    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touchpad_read);
    // ---- Initialize squareline UI ----
    ui_init(); // Create all squareline screens
  lv_slider_set_value(ui_SliderAvg, (int32_t)g_sliderAvgValue, LV_ANIM_OFF);
  lv_slider_set_value(ui_SliderGust, (int32_t)g_sliderGustValue, LV_ANIM_OFF);
  
  // Update labels to show initial values
  char buf[8];
  snprintf(buf, sizeof(buf), "%.0fs", g_sliderAvgValue);
  lv_label_set_text(ui_LabelAvgSlider, buf);
  snprintf(buf, sizeof(buf), "%.0fs", g_sliderGustValue);
  lv_label_set_text(ui_LabelGustSlider, buf);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFiManager wm;
  bool res;
  res = wm.autoConnect("MrWeather");
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  initSNTP();


  uint8_t currentChannel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&currentChannel, &second);
  Serial.printf("WiFi Channel: %d\n", currentChannel);
  
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("This device MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println("========================================");
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  Serial.println("ESP-NOW initialized successfully");
  esp_now_set_pmk(espnow_pmk);
  Serial.println("PMK set");
  esp_now_register_recv_cb(onESPNowReceive);
  Serial.println("Receive callback registered");
  espNowQueue = xQueueCreate(32, sizeof(EspNowMsg));
  if (espNowQueue == NULL) {
    Serial.println("Failed to create espNowQueue!");
  } else {
    Serial.println("espNowQueue created");
  }
  Serial.println("ESP-NOW initialized and listening...");
  esp_now_set_pmk(espnow_pmk);

  // Start UDP server
  udp.begin(UDP_PORT);
  Serial.printf("UDP server started on port %d\n", UDP_PORT);
  
  // Send WiFi credentials to transmitters after WiFi connects
  delay(2000);  // Wait for WiFi to stabilize
  //sendWiFiCredentials();
  Blynk.config(auth, IPAddress(216, 110, 224, 105), 8080);
  Blynk.connect();
}

bool connected = false;
unsigned long reconnectTime = 0;




// MODIFIED loop() - handle credential requests outside callback
void loop() {
    lv_timer_handler();
    
  if ((WiFi.status() == WL_CONNECTED) && (!connected)) {
    connected = true;
    ArduinoOTA.setHostname("SeanTemp");
    ArduinoOTA.begin();
    Serial.println("Connecting blynk...");
  }

  if ((WiFi.status() == WL_CONNECTED) && (connected)) {
    ArduinoOTA.handle();
    Blynk.run();
    handleUDPData();
  }
  
  // NEW: Handle credential requests outside callback
  if (credentialRequestReceived) {
    credentialRequestReceived = false;
    sendWiFiCredentialsToDevice(requestingDeviceMAC);
  }
  
  // Process queued ESP-NOW messages with deduplication
  EspNowMsg msg;
  while (espNowQueue != NULL && xQueueReceive(espNowQueue, &msg, 0) == pdTRUE) {
    const uint8_t sensor1mac[6] = {0x8C, 0xD0, 0xB2, 0xA9, 0x45, 0x45};
    const uint8_t sensor2mac[6] = {0xD0, 0xCF, 0x13, 0x0B, 0xA0, 0x70};
    
    unsigned long currentTime = millis();
    
    if (memcmp(msg.mac, sensor1mac, 6) == 0) {
      if (currentTime - s1LastMsgTime < MSG_MIN_INTERVAL) {
        Serial.printf("Sensor1: Duplicate rejected (only %lums since last)\n", currentTime - s1LastMsgTime);
        sensor1MsgCount++;
        continue;
      }
      
      if (msg.len == sizeof(esp_now_payload_t)) {
        esp_now_payload_t payload;
        memcpy(&payload, msg.payload, sizeof(payload));
        s1LastUpdate = millis();
        s1LastMsgTime = currentTime;
        s1temp = payload.temp;
        s1hum = payload.hum;
        s1pres = payload.presread;
        s1vBat = payload.volts0;
       // addTempToHistory(s1temp, 1);
        sensor1MsgCount++;
        Serial.printf("Sensor1: Message processed (temp=%.1f)\n", s1temp);
      }
    } else if (memcmp(msg.mac, sensor2mac, 6) == 0) {
      if (currentTime - s2LastMsgTime < MSG_MIN_INTERVAL) {
        Serial.printf("Sensor2: Duplicate rejected (only %lums since last)\n", currentTime - s2LastMsgTime);
        sensor2MsgCount++;
        continue;
      }
      
      if (msg.len == sizeof(esp_now_payload_wind_t)) {
        esp_now_payload_wind_t payload;
        memcpy(&payload, msg.payload, sizeof(payload));
        s2LastUpdate = millis();
        s2LastMsgTime = currentTime;
        s2temp = payload.temp;
        //addTempToHistory(s2temp, 2);
        s2windgust = payload.windgust;
        s2avgwind = payload.avgwind;
        s2instwind = payload.instwind;
        s2winddirV = payload.winddirV;
        float windDir = getWindDirection();
        Serial.printf("Wind: speed=%.1f, direction=%.1f°\n", s2avgwind, windDir);
        addWindData(s2avgwind, getWindDirection());
        sensor2MsgCount++;
        Serial.printf("Sensor2: Message processed (wind=%.1f)\n", windDir);
      }
    }
  }
  
  unsigned long secondsSinceUpdate1 = (millis() - s1LastUpdate) / 1000;
  unsigned long secondsSinceUpdate2 = (millis() - s2LastUpdate) / 1000;
  
  // Update UI labels every 500ms
  every(500) {
    char buf[64];
    
    // Display wind sensor data (sensor 0)
    if (s0temp < 200) {
      snprintf(buf, sizeof(buf), "%.2f°C", s0temp);
      lv_label_set_text(ui_LabelTemp, buf);
    }
    
    if (s0avgwind != 999) {
      snprintf(buf, sizeof(buf), "%.1fkph", s0avgwind);
      lv_label_set_text(ui_Label4, buf);
    }

    if (s0winddirV != 999) {
      float windDir = getWindDirection();
      const char* cardinal = getCardinalDirection(windDir);
      lv_label_set_text(ui_Label5, cardinal);
    }
    
    // Display sensor last update times
    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
      unsigned long secondsSinceUpdate = (millis() - sensorLastUpdate[i]) / 1000;
      // Update your UI labels for each sensor's last update time
      // Example: snprintf(buf, sizeof(buf), "%lus", secondsSinceUpdate);
    }
    
    // Display wind sensor last update
    unsigned long secondsSinceUpdate0 = (millis() - s0LastUpdate) / 1000;
    snprintf(buf, sizeof(buf), "%lus", secondsSinceUpdate0);
    lv_label_set_text(ui_LabelLU2, buf);
    
    if (WiFi.status() == WL_CONNECTED) {
      snprintf(buf, sizeof(buf), "%s", WiFi.localIP().toString().c_str());
      lv_label_set_text(ui_LabelIP, buf);
      
      snprintf(buf, sizeof(buf), "RSSI: %ddB", WiFi.RSSI());
      lv_label_set_text(ui_Label3, buf);
    }
    
    // Display time
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
  }
  
  every(2000) {
    drawTempGraph();
    drawWindRose();
  }
  
  every(30000) {
    if (s0instwind != 999) {
      Blynk.virtualWrite(V11, s0instwind);
      Blynk.virtualWrite(V12, s0avgwind);
      Blynk.virtualWrite(V13, s0windgust);
      Blynk.virtualWrite(V15, getWindDirection());
    }
    Blynk.virtualWrite(V14, WiFi.RSSI());
    if (sensorTemps[0] != 999) {
      Blynk.virtualWrite(V16, sensorTemps[0]);
    }
    if (s0temp < 200) {
      
      Blynk.virtualWrite(V17, s0temp);
    }

    //Serial.printf("ESPNow: C1=%lu C2=%lu QDropped=%lu\n", sensor1MsgCount, sensor2MsgCount, espNowQueueDroppedCount);
  }
  
  if (g_sliderValuesChanged && WiFi.status() == WL_CONNECTED) {
    sendSliderValues();
  }
  
  every(60000) {
    if (WiFi.status() == WL_CONNECTED) {
      sendSliderValues();
    }
  }
}