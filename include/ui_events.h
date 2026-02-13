// ui_events.h

#ifndef _UI_EVENTS_H
#define _UI_EVENTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

// Global variables for slider values
extern float g_sliderAvgValue;
extern float g_sliderGustValue;
extern volatile bool g_sliderValuesChanged;

// Event handler declarations
void ui_event_ButtonReturn(lv_event_t * e);
void ui_event_SliderAvg(lv_event_t * e);
void ui_event_SliderGust(lv_event_t * e);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif