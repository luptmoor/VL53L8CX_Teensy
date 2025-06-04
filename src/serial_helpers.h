#ifndef SERIAL_HELPERS_H
#define SERIAL_HELPERS_H

#include <stdint.h>
#include <Arduino.h>
#include <vl53l8cx.h>

void serialPrint(const String &msg);
void serialPrintln(const String &msg);
void serialPrintf(const char *fmt, ...);
void display_commands_banner(void);
void serialPrintMeasurementResult(VL53L8CX_ResultsData *Result, uint8_t res, bool EnableSignal, bool EnableAmbient);
void handle_cmd(uint8_t cmd);
void toggle_resolution(void);
void toggle_signal_and_ambient(void);

#endif // SERIAL_HELPERS_H
