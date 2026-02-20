SET(SOURCES screens/ui_ScreenWifi.c
    screens/ui_ScreenMain.c
    screens/ui_ScreenSettings.c
    screens/ui_ScreenChart.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_windrose_png.c
    fonts/ui_font_Font1.c
    fonts/ui_font_Font10.c
    fonts/ui_font_Font12.c
    fonts/ui_font_Font16.c
    fonts/ui_font_Font20.c
    fonts/ui_font_Font24.c
    fonts/ui_font_Font32.c)

add_library(ui ${SOURCES})
