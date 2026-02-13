// Add to ui_events.c (or create it if it doesn't exist)

#include "ui.h"

// Global variables for slider values and change flag
float g_sliderAvgValue = 60.0;
float g_sliderGustValue = 1.0;
volatile bool g_sliderValuesChanged = false;

void ui_event_SliderAvg(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target_obj(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
        // Update the label
        _ui_slider_set_text_value(ui_LabelAvgSlider, target, "", "s");
        
        // Update global variable and set flag
        g_sliderAvgValue = (float)lv_slider_get_value(target);
        g_sliderValuesChanged = true;
    }
	
}

void ui_event_SliderGust(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target_obj(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
        // Update the label
        _ui_slider_set_text_value(ui_LabelGustSlider, target, "", "s");
        
        // Update global variable and set flag
        g_sliderGustValue = (float)lv_slider_get_value(target);
        g_sliderValuesChanged = true;
    }
}