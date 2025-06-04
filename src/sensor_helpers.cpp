#include "sensor_helpers.h"
#include "serial_helpers.h"
#include <Wire.h>

extern VL53L8CX sensor_vl53l8cx_top;
extern uint8_t res;
extern uint8_t status;

void sensor_init_and_report()
{
    serialPrintln("Configuring component");
    sensor_vl53l8cx_top.begin();

    uint8_t p_alive = 0;
    status = sensor_vl53l8cx_top.is_alive(&p_alive);
    serialPrint("Sensor alive: ");
    serialPrint(String(p_alive));
    serialPrint(" with status: ");
    serialPrintln(String(status));

    Wire.beginTransmission(0x29);
    uint8_t err = Wire.endTransmission();
    serialPrintf("error = %u (should be 0)\n", err);

    if (status)
    {
        serialPrint("Error checking sensor alive with status: ");
        serialPrintln(String(status));
    }

    serialPrintln("Calling init");
    status = sensor_vl53l8cx_top.init();
    if (status)
    {
        serialPrint("Error initializing sensor with status: ");
        serialPrintln(String(status));
    }
    status = sensor_vl53l8cx_top.set_resolution(res);
    if (status)
    {
        serialPrint("Error setting resolution with status: ");
        serialPrintln(String(status));
    }
    status = sensor_vl53l8cx_top.set_ranging_frequency_hz(60);
    if (status)
    {
        serialPrint("Error setting frequency with status: ");
        serialPrintln(String(status));
    }

    serialPrintln("Start measurements");
    status = sensor_vl53l8cx_top.start_ranging();
    if (status)
    {
        serialPrint("Error starting measurements with status: ");
        serialPrintln(String(status));
    }
}

VL53L8CX_ResultsData get_sensor_data()
{

    VL53L8CX_ResultsData Results;
    uint8_t NewDataReady = 0;

    do
    {
        status = sensor_vl53l8cx_top.check_data_ready(&NewDataReady);
        delay(20);
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0))
    {
        status = sensor_vl53l8cx_top.get_ranging_data(&Results);

        // Log data to SD card
        // writeResultsToSD(dataFile, dataFileName, Results, res);

        // serialPrintMeasurementResult(&Results, res, EnableSignal, EnableAmbient);

        // if (SerialPort.available() > 0)
        // {
        //     handle_cmd(SerialPort.read());
        // }
        return Results;
    }
}