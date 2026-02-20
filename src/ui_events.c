
#include "ui.h"

// External declarations from main file
extern struct ChartState {
    int dataType;  // 0=TEMP, 1=HUM, 2=PRES, 3=WIND
    int sensorIndex;
    const char* title;
    const char* units;
} currentChart;

// WiFi functions implemented in main .cpp file
extern void populateWifiList();
extern void SaveWifiFromUI(lv_event_t * e);

void my_chart_event_cb(lv_event_t * e)
{
	// Your code here
}

void my_chart_event_cb2(lv_event_t * e)
{
	// Your code here
}

void ClickTemp1(lv_event_t * e)
{
	// Sensor 0 (first sensor) - Temperature
	currentChart.dataType = 0;  // CHART_TEMP
	currentChart.sensorIndex = 0;
	currentChart.title = "Temperature";
	currentChart.units = "°C";
}

void ClickWindSpeed(lv_event_t * e)
{
	// Wind speed chart - handled separately in main code
	currentChart.dataType = 3;  // CHART_WIND_SPEED
	currentChart.sensorIndex = -1;
	currentChart.title = "Wind Speed";
	currentChart.units = "kph";
}

void ClickTemp2(lv_event_t * e)
{
	// Sensor 1 (second sensor) - Temperature
	currentChart.dataType = 0;  // CHART_TEMP
	currentChart.sensorIndex = 1;
	currentChart.title = "Temperature";
	currentChart.units = "°C";
}

void ClickHum1(lv_event_t * e)
{
	// Sensor 0 - Humidity
	currentChart.dataType = 1;  // CHART_HUM
	currentChart.sensorIndex = 0;
	currentChart.title = "Humidity";
	currentChart.units = "%";
}

void ClickHum3(lv_event_t * e)
{
	// Sensor 2 - Humidity
	currentChart.dataType = 1;  // CHART_HUM
	currentChart.sensorIndex = 2;
	currentChart.title = "Humidity";
	currentChart.units = "%";
}

void ClickHum2(lv_event_t * e)
{
	// Sensor 1 - Humidity
	currentChart.dataType = 1;  // CHART_HUM
	currentChart.sensorIndex = 1;
	currentChart.title = "Humidity";
	currentChart.units = "%";
}

void ClickPres1(lv_event_t * e)
{
	// Sensor 0 - Pressure
	currentChart.dataType = 2;  // CHART_PRES
	currentChart.sensorIndex = 0;
	currentChart.title = "Pressure";
	currentChart.units = "mbar";
}

void ClickHum4(lv_event_t * e)
{
	// Sensor 3 - Humidity
	currentChart.dataType = 1;  // CHART_HUM
	currentChart.sensorIndex = 3;
	currentChart.title = "Humidity";
	currentChart.units = "%";
}

void ClickTemp3(lv_event_t * e)
{
	// Sensor 2 - Temperature
	currentChart.dataType = 0;  // CHART_TEMP
	currentChart.sensorIndex = 2;
	currentChart.title = "Temperature";
	currentChart.units = "°C";
}

void ClickTemp4(lv_event_t * e)
{
	// Sensor 3 - Temperature
	currentChart.dataType = 0;  // CHART_TEMP
	currentChart.sensorIndex = 3;
	currentChart.title = "Temperature";
	currentChart.units = "°C";
}

void ClickPres4(lv_event_t * e)
{
	// Sensor 3 - Pressure
	currentChart.dataType = 2;  // CHART_PRES
	currentChart.sensorIndex = 3;
	currentChart.title = "Pressure";
	currentChart.units = "mbar";
}

void ClickPres3(lv_event_t * e)
{
	// Sensor 2 - Pressure
	currentChart.dataType = 2;  // CHART_PRES
	currentChart.sensorIndex = 2;
	currentChart.title = "Pressure";
	currentChart.units = "mbar";
}

void ClickPres2(lv_event_t * e)
{
	// Sensor 1 - Pressure
	currentChart.dataType = 2;  // CHART_PRES
	currentChart.sensorIndex = 1;
	currentChart.title = "Pressure";
	currentChart.units = "mbar";
}