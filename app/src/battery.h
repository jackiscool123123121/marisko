#pragma once

#include <stdbool.h>

/* Configure the BQ24232 charger pins and enable charging. Call once at boot,
 * before anything reads usb_power_present()/battery_charging(). */
void charger_init(void);

bool usb_power_present(void);
bool battery_charging(void);

/* 0..100, clamped; -1 if the ADC read failed. */
int battery_percent(void);

/* 1..4 quarters for a 4-LED gauge; 0 if the ADC read failed. */
int battery_quarters(void);
