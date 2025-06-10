#include "sensor_helpers.h"
#include "serial_helpers.h"
#include <Wire.h>

void sensor_init_and_report(VL53L8CX &sensor_vl53l8cx, uint8_t res)
{
    serialPrintln("Configuring component");
    sensor_vl53l8cx.begin();

    uint8_t p_alive = 0;
    uint8_t status;
    status = sensor_vl53l8cx.is_alive(&p_alive);
    serialPrint("Sensor alive: ");
    serialPrint(String(p_alive));
    serialPrint(" with status: ");
    serialPrintln(String(status));

    // Wire.beginTransmission(0x29);
    // uint8_t err = Wire.endTransmission();
    // serialPrintf("error = %u (should be 0)\n", err);

    if (status)
    {
        serialPrint("Error checking sensor alive with status: ");
        serialPrintln(String(status));
    }

    serialPrintln("Calling init");
    status = sensor_vl53l8cx.init();
    if (status)
    {
        serialPrint("Error initializing sensor with status: ");
        serialPrintln(String(status));
    }
    status = sensor_vl53l8cx.set_resolution(res);
    if (status)
    {
        serialPrint("Error setting resolution with status: ");
        serialPrintln(String(status));
    }
    status = sensor_vl53l8cx.set_ranging_frequency_hz(60);
    if (status)
    {
        serialPrint("Error setting frequency with status: ");
        serialPrintln(String(status));
    }

    serialPrintln("Start measurements");
    status = sensor_vl53l8cx.start_ranging();
    if (status)
    {
        serialPrint("Error starting measurements with status: ");
        serialPrintln(String(status));
    }
}

VL53L8CX_ResultsData get_sensor_data(VL53L8CX &sensor_vl53l8cx)
{

    VL53L8CX_ResultsData Results;
    uint8_t NewDataReady = 0;
    uint8_t status;

    do
    {
        status = sensor_vl53l8cx.check_data_ready(&NewDataReady);
        delay(5);
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0))
    {
        status = sensor_vl53l8cx.get_ranging_data(&Results);
        return Results;
    }
    else
    {
        serialPrint("Error getting sensor data with status: ");
        serialPrintln(String(status));
        VL53L8CX_ResultsData emptyResults = {};
        return emptyResults; // Return an empty results structure on error
    }
}