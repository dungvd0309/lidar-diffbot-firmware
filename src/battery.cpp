#include "battery.h"

float g_battery_voltage_filtered = 0.0f; 
BatteryData g_battery_data = {0.0f, 0};

int voltage_to_percentage(float voltage);
float apply_ema_filter(float new_voltage, float alpha);

void battery_init() 
{
    analogReadResolution(CONFIG::adc_resolution); 
    analogSetAttenuation(ADC_11db); // Full range (0-3.3V) for 12-bit ADC 
    g_battery_voltage_filtered = get_battery_voltage(); 
    g_battery_data.voltage = g_battery_voltage_filtered;
    g_battery_data.percentage = voltage_to_percentage(g_battery_voltage_filtered);
}

uint16_t get_battery_adc_raw()
{
    return analogRead(CONFIG::battery_voltage_pin);
}

// Calculate battery voltage 
float get_battery_voltage()
{
    uint16_t adc_value = get_battery_adc_raw();
    return (adc_value / CONFIG::adc_max_value) * CONFIG::battery_adc_reference_voltage / CONFIG::battery_voltage_divider_ratio;
}

// Calculate percentage from battery voltage
int voltage_to_percentage(float voltage) 
{
    int percentage = ((voltage - CONFIG::battery_min_voltage) / 
                        (CONFIG::battery_max_voltage - CONFIG::battery_min_voltage)) * 100;
    if (percentage < 0)         percentage = 0;
    else if (percentage > 100)  percentage = 100;
    return percentage;
}

// Apply EMA filter to smooth voltage readings
float apply_ema_filter(float new_voltage, float alpha) 
{
    g_battery_voltage_filtered = alpha * new_voltage + (1 - alpha) * g_battery_voltage_filtered;
    return g_battery_voltage_filtered;
}

void battery_update()
{  
    float battery_voltage = get_battery_voltage();
    battery_voltage = apply_ema_filter(battery_voltage, CONFIG::battery_ema_alpha);
    g_battery_data.voltage = battery_voltage;
    g_battery_data.percentage = voltage_to_percentage(battery_voltage);
}

BatteryData battery_read()
{
    return g_battery_data;
}