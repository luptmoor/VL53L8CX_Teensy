/* Includes ------------------------------------------------------------------*/
#include <SD.h>

#include "vl53l8cx.h"

#include "serial_helpers.h"
#include "sd_helpers.h"
#include "i2c_helpers.h"
#include "sensor_helpers.h"
#include "cf_helpers.hpp"

#define SerialPort Serial

#define LPN_PIN -1
#define PWREN_PIN -1

#define WRITE_TO_SD

// Components.
VL53L8CX sensor_vl53l8cx_top(&Wire, LPN_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L8CX_RESOLUTION_4X4;
uint8_t status;

// File for logging data
File dataFile;
char dataFileName[20] = "tof_data_01.csv";

/* Setup ---------------------------------------------------------------------*/
void setup()
{
    // Initialize serial for output.
    SerialPort.begin(9600);

    serialPrintln("Starting Crazyflie communication");
    COMMUNICATION_SERIAL.begin(COMMUNICATION_SERIAL_BAUD);

#ifdef WRITE_TO_SD
    // Initialize SD card
    serialPrintln("Initializing SD card...");
    if (!SD.begin(BUILTIN_SDCARD))
    {
        serialPrintln("Card failed, or not present.");
        while (1)
            ;
    }
    serialPrintln("Card initialized.");
    
    // Find next available data file name
    setNextDataFileName();
    serialPrint("Logging to file: ");
    serialPrintln(String(dataFileName));
    dataFile = SD.open(dataFileName, FILE_WRITE);
    writeSDHeader(dataFile);
#endif

    serialPrintln("Initializing I2C bus");
    // Initialize I2C bus.
    Wire.begin();
    Wire.setClock(400000); // Set I2C clock speed to 400kHz

    scan_i2c();
    // Sensor init and status reporting
    serialPrintln("Initializing sensor");
    sensor_init_and_report();
}

void loop()
{
    VL53L8CX_ResultsData Results = get_sensor_data();

#ifdef WRITE_TO_SD
    // Log results to SD card
    writeResultsToSD(dataFile, Results, res);
#endif

    // Store output message to be sent back to CF
    setOutputMessage(Results, res);

    // Send message via UART to CF
    sendCrazyflie();
}