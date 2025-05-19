#include "display_helpers.h"
#include "serial_helpers.h"
#include <vl53l8cx.h>

extern VL53L8CX sensor_vl53l8cx_top;
extern bool EnableAmbient;
extern bool EnableSignal;
extern uint8_t res;
extern uint8_t status;

void toggle_resolution(void)
{
    status = sensor_vl53l8cx_top.stop_ranging();
    switch (res)
    {
    case VL53L8CX_RESOLUTION_4X4:
        res = VL53L8CX_RESOLUTION_8X8;
        break;
    case VL53L8CX_RESOLUTION_8X8:
        res = VL53L8CX_RESOLUTION_4X4;
        break;
    default:
        break;
    }
    status = sensor_vl53l8cx_top.set_resolution(res);
    status = sensor_vl53l8cx_top.start_ranging();
}

void toggle_signal_and_ambient(void)
{
    EnableAmbient = !EnableAmbient;
    EnableSignal = !EnableSignal;
}

void handle_cmd(uint8_t cmd)
{
    switch (cmd)
    {
    case 'r':
        toggle_resolution();
        break;
    case 's':
        toggle_signal_and_ambient();
        break;
    case 'c':
        break;
    default:
        break;
    }
}
