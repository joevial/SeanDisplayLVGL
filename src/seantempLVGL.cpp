#include <lvgl.h>                            // lvgl version 9.2.2
#include <PINS_JC4827W543.h>                //dev device pins library
#include "TAMC_GT911.h"                     // TAMC_GT911 library

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
    float temp;
    unsigned long timestamp;
};

static TempDataPoint tempHistoryFallback[TEMP_FALLBACK_SIZE];
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
TempDataPoint *tempHistory = nullptr;
int tempHistoryIndex = 0;
int tempHistoryCount = 0;

// Function declarations
void cbSyncTime(struct timeval *tv);
void initSNTP();
void setTimezone();
void onESPNowReceive(const esp_now_recv_info_t * info, const uint8_t *data, int len);
void addTempToHistory(float temp);
void drawTempGraph();
void drawWindSpeedGraph();
void addWindData(float speed, float direction);
void drawWindRose();
void setup();
void loop();
float getWindDirection();
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
lv_chart_series_t * temp_chart_series = NULL;
lv_obj_t * temp_scale_y = NULL;
lv_obj_t * temp_container = NULL;
// Temperature chart label popup (simpler overlay approach for LVGL v9.3)
static lv_obj_t * temp_label_popup = NULL;

static void temp_chart_draw_label_cb(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * chart = (lv_obj_t*) lv_event_get_target(e);
  
  if(code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING) {
    int32_t id = lv_chart_get_pressed_point(chart);
    
    Serial.printf("Touch event: id=%d, tempHistoryCount=%d\n", id, tempHistoryCount);
    
    if(id == LV_CHART_POINT_NONE || tempHistory == nullptr || tempHistoryCount == 0 || temp_chart == NULL) {
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      Serial.println("Touch rejected: no data or chart");
      return;
    }
    
    // Get current chart point count
    int chartPoints = lv_chart_get_point_count(temp_chart);
    
    Serial.printf("Chart points=%d, id=%d\n", chartPoints, id);
    
    // Validate id is within chart range
    if (id < 0 || id >= chartPoints) {
      Serial.printf("ERROR: id %d out of chart range [0-%d]\n", id, chartPoints-1);
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    // Safety check: if chartPoints doesn't match what we expect, bail out
    int expectedPoints = tempHistoryCount;
    if (expectedPoints > 100) expectedPoints = 100;
    if (expectedPoints < 2) expectedPoints = 2;
    
    if (chartPoints != expectedPoints) {
      Serial.printf("WARNING: Chart points mismatch! expected=%d, actual=%d\n", expectedPoints, chartPoints);
      // Don't crash, just hide label and return
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    // Map chart point ID to history index (same logic as drawTempGraph)
    int startIdx = (tempHistoryIndex - tempHistoryCount + tempHistoryCapacity) % tempHistoryCapacity;
    int historyIdx;
    
    if (tempHistoryCount <= chartPoints) {
      // Direct mapping - all data shown
      historyIdx = id;
    } else {
      // Sample mapping - data is sampled
      if (chartPoints <= 1) {
        historyIdx = 0;
      } else {
        historyIdx = (id * (tempHistoryCount - 1)) / (chartPoints - 1);
      }
    }
    
    Serial.printf("historyIdx=%d, tempHistoryCount=%d, tempHistoryCapacity=%d\n", 
                  historyIdx, tempHistoryCount, tempHistoryCapacity);
    
    // Validate historyIdx
    if (historyIdx < 0 || historyIdx >= tempHistoryCount) {
      Serial.printf("ERROR: historyIdx %d out of range [0-%d]\n", historyIdx, tempHistoryCount-1);
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    // Calculate actual array index
    int idx = (startIdx + historyIdx) % tempHistoryCapacity;
    
    Serial.printf("Final idx=%d (startIdx=%d)\n", idx, startIdx);
    
    // Validate final index
    if (idx < 0 || idx >= tempHistoryCapacity) {
      Serial.printf("ERROR: final idx %d out of capacity range [0-%d]\n", idx, tempHistoryCapacity-1);
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    // Extra paranoid check: verify this slot actually has valid data
    if (tempHistory[idx].timestamp == 0) {
      Serial.printf("ERROR: idx %d has uninitialized timestamp\n", idx);
      if(temp_label_popup != NULL) {
        lv_obj_add_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
      }
      return;
    }
    
    // NOW safe to access the data
    float temp_value = tempHistory[idx].temp;
    time_t timestamp = (time_t)tempHistory[idx].timestamp;
    
    Serial.printf("Data: temp=%.1f, timestamp=%ld\n", temp_value, (long)timestamp);
    
    // Create label popup if it doesn't exist
    if(temp_label_popup == NULL) {
      Serial.println("Creating label popup...");
      temp_label_popup = lv_label_create(lv_scr_act());
      if (temp_label_popup == NULL) {
        Serial.println("ERROR: Failed to create label");
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
      Serial.println("Label popup created");
    }
    
    // Format the label text
    Serial.println("Formatting label text...");
    static char buf[64];  // Make static to avoid stack issues
    if (timestamp > 0 && timestamp >= 1000000000) {
      Serial.println("Calling localtime...");
      struct tm timeinfo_buf;
      struct tm* timeinfo = localtime_r(&timestamp, &timeinfo_buf);  // Use thread-safe version
      Serial.printf("localtime returned: %p\n", timeinfo);
      
      if (timeinfo != NULL) {
        Serial.printf("Time: %d:%d\n", timeinfo->tm_hour, timeinfo->tm_min);
        int hour = timeinfo->tm_hour;
        int min = timeinfo->tm_min;
        const char* ampm = (hour >= 12) ? "PM" : "AM";
        if (hour > 12) hour -= 12;
        else if (hour == 0) hour = 12;
        snprintf(buf, sizeof(buf), "%.2f°C\n%d:%02d %s", temp_value, hour, min, ampm);
      } else {
        Serial.println("localtime failed, using temp only");
        snprintf(buf, sizeof(buf), "%.2f°C", temp_value);
      }
    } else {
      snprintf(buf, sizeof(buf), "%.2f°C", temp_value);
    }
    
    Serial.printf("Setting label text: %s\n", buf);
    lv_label_set_text(temp_label_popup, buf);
    Serial.println("Label text set");
    
    // Get point position on chart
    Serial.println("Getting chart series...");
    lv_chart_series_t * ser = lv_chart_get_series_next(chart, NULL);
    if(ser == NULL) {
      Serial.println("ERROR: No series found");
      return;
    }
    
    Serial.println("Getting point position...");
    lv_point_t p;
    lv_chart_get_point_pos_by_id(chart, ser, id, &p);
    
    Serial.println("Getting chart coordinates...");
    lv_area_t chart_coords;
    lv_obj_get_coords(chart, &chart_coords);
    
    // Position label above the data point
    lv_coord_t label_x = chart_coords.x1 + p.x;
    lv_coord_t label_y = chart_coords.y1 + p.y - 45;
    
    Serial.println("Updating label layout...");
    lv_obj_update_layout(temp_label_popup);
    lv_coord_t label_width = lv_obj_get_width(temp_label_popup);
    
    Serial.printf("Positioning label at %d, %d\n", label_x - (label_width / 2), label_y);
    lv_obj_set_pos(temp_label_popup, label_x - (label_width / 2), label_y);
    lv_obj_clear_flag(temp_label_popup, LV_OBJ_FLAG_HIDDEN);
    
    Serial.println("Label displayed successfully");
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
    uint16_t rawADC = s2winddirV;
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
// ============================================
// Updated addTempToHistory Function
// ============================================

void addTempToHistory(float temp) {
  if (tempHistory == nullptr) return;
  
  // Store temperature and ACTUAL timestamp (not millis)
  time_t now = time(nullptr);
  tempHistory[tempHistoryIndex].temp = temp;
  tempHistory[tempHistoryIndex].timestamp = (unsigned long)now;  // Store as Unix timestamp
  
  tempHistoryIndex = (tempHistoryIndex + 1) % tempHistoryCapacity;
  if (tempHistoryCount < tempHistoryCapacity) {
    tempHistoryCount++;
  }
}

void drawTempGraph() {
  if (tempHistory == nullptr || tempHistoryCount == 0) return;
  
  // Create chart on first call
  if (temp_chart == NULL) {
    // Create container - minimal borders/padding
    temp_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(temp_container, 240, 272);
    lv_obj_align(temp_container, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_flex_flow(temp_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(temp_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(temp_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(temp_container, 2, 0);  // Minimal padding
    lv_obj_set_style_pad_top(temp_container, 3, 0);
    lv_obj_set_style_border_width(temp_container, 0, 0);  // No border
    
    // Title label - smaller font, minimal margin
    lv_obj_t * label = lv_label_create(temp_container);
    lv_label_set_text(label, "Temperature");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_pad_bottom(label, 0, 0);
    
    // Create horizontal container for chart and y-axis scale
    lv_obj_t * chart_container = lv_obj_create(temp_container);
    lv_obj_set_size(chart_container, 230, 215);
    lv_obj_set_flex_flow(chart_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(chart_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(chart_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(chart_container, 0, 0);  // No padding
    lv_obj_set_style_border_width(chart_container, 0, 0);  // No border
    
    // Create the chart
    temp_chart = lv_chart_create(chart_container);
    lv_obj_set_size(temp_chart, 185, 200);
    lv_chart_set_point_count(temp_chart, 1);  // Start with 1, will update dynamically
    lv_obj_add_event_cb(temp_chart, temp_chart_draw_label_cb, LV_EVENT_ALL, NULL);
    lv_obj_refresh_ext_draw_size(temp_chart);
    lv_chart_set_type(temp_chart, LV_CHART_TYPE_LINE);
    lv_obj_set_style_pad_all(temp_chart, 0, 0);  // No padding inside chart
    
    // Add data series
    temp_chart_series = lv_chart_add_series(temp_chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    
    // Create Y-axis scale
    temp_scale_y = lv_scale_create(chart_container);
    lv_obj_set_size(temp_scale_y, 38, 200);  // Match chart height
    lv_scale_set_mode(temp_scale_y, LV_SCALE_MODE_VERTICAL_RIGHT);
    lv_scale_set_label_show(temp_scale_y, true);
    lv_obj_set_style_length(temp_scale_y, 5, LV_PART_ITEMS);
    lv_obj_set_style_length(temp_scale_y, 10, LV_PART_INDICATOR);
    lv_obj_set_style_text_font(temp_scale_y, &lv_font_montserrat_10, 0);
    lv_obj_set_style_pad_all(temp_scale_y, 0, 0);
    
    // Create X-axis container for time labels - minimal padding
    lv_obj_t * x_axis_container = lv_obj_create(temp_container);
    lv_obj_set_size(x_axis_container, 185, 20);  // Reduced height
    lv_obj_set_flex_flow(x_axis_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(x_axis_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(x_axis_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(x_axis_container, 0, 0);  // No padding
    lv_obj_set_style_pad_left(x_axis_container, 3, 0);  // Tiny left padding
    lv_obj_set_style_pad_right(x_axis_container, 3, 0);  // Tiny right padding
    lv_obj_set_style_border_width(x_axis_container, 0, 0);  // No border
    
    // Create time labels for X-axis
    for (int i = 0; i < 5; i++) {
      lv_obj_t * time_label = lv_label_create(x_axis_container);
      lv_label_set_text(time_label, "--:--");
      lv_obj_set_style_text_font(time_label, &lv_font_montserrat_8, 0);
      lv_obj_set_style_pad_all(time_label, 0, 0);
    }
  }
  
  // IMPORTANT: Update point count to match actual data (this makes points fill the width!)
  int displayPoints = tempHistoryCount;
  if (displayPoints > 100) displayPoints = 100;  // Cap at 100 for performance
  if (displayPoints < 2) displayPoints = 2;  // Minimum 2 points
  
  Serial.printf("Updating chart: tempHistoryCount=%d, displayPoints=%d\n", tempHistoryCount, displayPoints);
  lv_chart_set_point_count(temp_chart, displayPoints);
  
  // Find min/max temperature in history
  float minTemp = 999.0;
  float maxTemp = -999.0;
  int startIdx = (tempHistoryIndex - tempHistoryCount + tempHistoryCapacity) % tempHistoryCapacity;
  
  for (int i = 0; i < tempHistoryCount; i++) {
    int idx = (startIdx + i) % tempHistoryCapacity;
    float temp = tempHistory[idx].temp;
    if (temp < minTemp) minTemp = temp;
    if (temp > maxTemp) maxTemp = temp;
  }
  
  // Add padding to range
  float range = maxTemp - minTemp;
  if (range < 5.0) {
    float center = (minTemp + maxTemp) / 2.0;
    minTemp = center - 2.5;
    maxTemp = center + 2.5;
  } else {
    minTemp -= 1.0;
    maxTemp += 1.0;
  }
  
  // Update chart data - map tempHistoryCount to displayPoints
  for (int i = 0; i < displayPoints; i++) {
    // Map display point to history index
    int historyIdx;
    if (tempHistoryCount <= displayPoints) {
      // Show all data points
      historyIdx = i;
    } else {
      // Sample data evenly across history
      historyIdx = (i * (tempHistoryCount - 1)) / (displayPoints - 1);
    }
    
    if (historyIdx < tempHistoryCount) {
      int idx = (startIdx + historyIdx) % tempHistoryCapacity;
      int32_t temp_value = (int32_t)(tempHistory[idx].temp * 100);
      lv_chart_set_value_by_id(temp_chart, temp_chart_series, i, temp_value);
    } else {
      lv_chart_set_value_by_id(temp_chart, temp_chart_series, i, LV_CHART_POINT_NONE);
    }
  }
  
  // Update chart range
  lv_chart_set_range(temp_chart, LV_CHART_AXIS_PRIMARY_Y, (int32_t)(minTemp * 100), (int32_t)(maxTemp * 100));
  
  // Update Y-axis scale
  int totalTicks = (int)(maxTemp - minTemp) + 1;
  lv_scale_set_total_tick_count(temp_scale_y, totalTicks);
  if (totalTicks < 10) {
    lv_scale_set_major_tick_every(temp_scale_y, 1);
  } else {
    lv_scale_set_major_tick_every(temp_scale_y, totalTicks / 5);
  }
  lv_scale_set_range(temp_scale_y, (int32_t)minTemp, (int32_t)maxTemp);
  
  // Update X-axis time labels using ACTUAL timestamps
  if (temp_container != NULL) {
    lv_obj_t * x_axis_container = lv_obj_get_child(temp_container, -1);
    
    if (x_axis_container != NULL && tempHistoryCount > 0) {
      int numLabels = lv_obj_get_child_count(x_axis_container);
      
      for (int i = 0; i < numLabels; i++) {
        lv_obj_t * time_label = lv_obj_get_child(x_axis_container, i);
        
        int dataPoint;
        if (numLabels <= 1) {
          dataPoint = 0;
        } else {
          dataPoint = (i * (tempHistoryCount - 1)) / (numLabels - 1);
        }
        
        if (dataPoint < tempHistoryCount) {
          int idx = (startIdx + dataPoint) % tempHistoryCapacity;
          
          if (idx >= 0 && idx < tempHistoryCapacity) {
            time_t timestamp = (time_t)tempHistory[idx].timestamp;
            
            if (timestamp > 0 && timestamp >= 1000000000) {
              struct tm* timeinfo = localtime(&timestamp);
              
              if (timeinfo != NULL) {
                int hour = timeinfo->tm_hour;
                int min = timeinfo->tm_min;
                const char* ampm = (hour >= 12) ? "PM" : "AM";
                if (hour > 12) hour -= 12;
                else if (hour == 0) hour = 12;
                
                char time_buf[16];
                lv_snprintf(time_buf, sizeof(time_buf), "%d:%02d%s", hour, min, ampm);
                lv_label_set_text(time_label, time_buf);
              }
            }
          }
        }
      }
    }
  }
  
  lv_chart_refresh(temp_chart);
}

void onESPNowReceive(const esp_now_recv_info_t * info, const uint8_t *data, int len)
{
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
  
  // Allocate PSRAM for temperature history (with SRAM fallbacks)
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
    
    // Allocate PSRAM for wind history
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
    
    // Allocate wind rose data arrays (with fallback)
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
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFiManager wm;
  bool res;
  res = wm.autoConnect("MrWeather");
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  initSNTP();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(onESPNowReceive);
  espNowQueue = xQueueCreate(32, sizeof(EspNowMsg));
  if (espNowQueue == NULL) {
    Serial.println("Failed to create espNowQueue!");
  } else {
    Serial.println("espNowQueue created");
  }
  Serial.println("ESP-NOW initialized and listening...");
  Blynk.config(auth, IPAddress(216, 110, 224, 105), 8080);
  Blynk.connect();
}

bool connected = false;
unsigned long reconnectTime = 0;

// Replace your message processing code in loop() with this:
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
  }
  
  // Process queued ESP-NOW messages with deduplication
  EspNowMsg msg;
  while (espNowQueue != NULL && xQueueReceive(espNowQueue, &msg, 0) == pdTRUE) {
    const uint8_t sensor1mac[6] = {0x8C, 0xD0, 0xB2, 0xA9, 0x45, 0x45};
    const uint8_t sensor2mac[6] = {0xD0, 0xCF, 0x13, 0x0B, 0xA0, 0x70};
    
    unsigned long currentTime = millis();
    
    if (memcmp(msg.mac, sensor1mac, 6) == 0) {
      // Check if enough time has passed since last message from sensor 1
      if (currentTime - s1LastMsgTime < MSG_MIN_INTERVAL) {
        Serial.printf("Sensor1: Duplicate rejected (only %lums since last)\n", currentTime - s1LastMsgTime);
        sensor1MsgCount++;  // Still count it as received
        continue;  // Skip processing this duplicate
      }
      
      if (msg.len == sizeof(esp_now_payload_t)) {
        esp_now_payload_t payload;
        memcpy(&payload, msg.payload, sizeof(payload));
        s1LastUpdate = millis();
        s1LastMsgTime = currentTime;  // Update last message time
        s1temp = payload.temp;
        s1hum = payload.hum;
        s1pres = payload.presread;
        s1vBat = payload.volts0;
        addTempToHistory(s1temp);
        sensor1MsgCount++;
        Serial.printf("Sensor1: Message processed (temp=%.1f)\n", s1temp);
      }
    } else if (memcmp(msg.mac, sensor2mac, 6) == 0) {
      // Check if enough time has passed since last message from sensor 2
      if (currentTime - s2LastMsgTime < MSG_MIN_INTERVAL) {
        Serial.printf("Sensor2: Duplicate rejected (only %lums since last)\n", currentTime - s2LastMsgTime);
        sensor2MsgCount++;  // Still count it as received
        continue;  // Skip processing this duplicate
      }
      
      if (msg.len == sizeof(esp_now_payload_wind_t)) {
        esp_now_payload_wind_t payload;
        memcpy(&payload, msg.payload, sizeof(payload));
        s2LastUpdate = millis();
        s2LastMsgTime = currentTime;  // Update last message time
        s2temp = payload.temp;
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
    } else {
      // unknown MAC - ignore
    }
  }
  
  unsigned long secondsSinceUpdate1 = (millis() - s1LastUpdate) / 1000;
  unsigned long secondsSinceUpdate2 = (millis() - s2LastUpdate) / 1000;
  
  // Update UI labels (every 500ms)
  every(500) {
    char buf[64];
    
    // Update temperature label (ui_LabelTemp shows s1temp)
    if (s1temp < 200) {
      snprintf(buf, sizeof(buf), "%.2f°C", s1temp);
      lv_label_set_text(ui_LabelTemp, buf);
    }
    
    // Update wind speed label (ui_Label4 shows s2avgwind)
    if (s2avgwind != 999) {
      snprintf(buf, sizeof(buf), "%.1fkph", s2avgwind);
      lv_label_set_text(ui_Label4, buf);
    }

    if (s2winddirV != 999) {
      float windDir = getWindDirection();
      const char* cardinal = getCardinalDirection(windDir);
      lv_label_set_text(ui_Label5, cardinal);
    }
    
    // Update last update labels (LU1 = temp, LU2 = wind)
    snprintf(buf, sizeof(buf), "%lus", secondsSinceUpdate1);
    lv_label_set_text(ui_LabelLU1, buf);
    
    snprintf(buf, sizeof(buf), "%lus", secondsSinceUpdate2);
    lv_label_set_text(ui_LabelLU2, buf);
    
    // Update IP address label (ui_LabelIP)
    if (WiFi.status() == WL_CONNECTED) {
      snprintf(buf, sizeof(buf), "%s", WiFi.localIP().toString().c_str());
      lv_label_set_text(ui_LabelIP, buf);
    }
    
    // Update RSSI label (ui_Label3 shows WiFi signal strength)
    if (WiFi.status() == WL_CONNECTED) {
      snprintf(buf, sizeof(buf), "RSSI: %ddB", WiFi.RSSI());
      lv_label_set_text(ui_Label3, buf);
    }
    
    // Update time label (ui_LabelTime shows current local time in 12hr AMPM format)
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    int hour = timeinfo->tm_hour;
    int min = timeinfo->tm_min;
    int sec = timeinfo->tm_sec;
    const char* ampm = (hour >= 12) ? "PM" : "AM";
    if (hour > 12) hour -= 12;
    else if (hour == 0) hour = 12;  // Convert 0 to 12 for midnight
    snprintf(buf, sizeof(buf), "%d:%02d:%02d %s", hour, min, sec, ampm);
    lv_label_set_text(ui_LabelTime, buf);
  }
  every(2000) {
    drawTempGraph();
    drawWindRose();

  }
  
  every(30000) {
    // Send data to Blynk every 30 seconds
    if (s2instwind != 999) {
      Blynk.virtualWrite(V11, s2instwind);
      Blynk.virtualWrite(V12, s2avgwind);
      Blynk.virtualWrite(V13, s2windgust);
      Blynk.virtualWrite(V15, getWindDirection());
    }
    Blynk.virtualWrite(V14, WiFi.RSSI());

    if (s1temp < 200) {
      Blynk.virtualWrite(V16, s1temp);
      Blynk.virtualWrite(V17, s1hum);
      Blynk.virtualWrite(V18, s1pres);
    }

    // Diagnostic: print ESP-NOW queue stats every 30s
    Serial.printf("ESPNow: C1=%lu C2=%lu QDropped=%lu\n", sensor1MsgCount, sensor2MsgCount, espNowQueueDroppedCount);
  }
}