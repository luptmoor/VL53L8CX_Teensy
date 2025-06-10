#ifndef SENSOR_HELPERS_H
#define SENSOR_HELPERS_H

#include <vl53l8cx.h>
#include <stdint.h>

// extern VL53L8CX sensor_vl53l8cx_top;
// extern uint8_t res;
// extern uint8_t status;

void sensor_init_and_report(VL53L8CX &sensor_vl53l8cx, uint8_t res);
VL53L8CX_ResultsData get_sensor_data(VL53L8CX &sensor_vl53l8cx);

#endif // SENSOR_HELPERS_H
