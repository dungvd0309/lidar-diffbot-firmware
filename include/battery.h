#pragma once
#include <Arduino.h>

#include "config.h"

struct BatteryData
{
    float voltage;
    int percentage;
};

void battery_init();
uint16_t get_battery_adc_raw();
float get_battery_voltage();
void battery_update();
BatteryData battery_read();