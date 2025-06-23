/* Includes ------------------------------------------------------------------*/
#include <SD.h>
#include "vl53l8cx.h"

#include "serial_helpers.h"
#include "i2c_helpers.h"
#include "sensor_helpers.h"
#include "cf_helpers.hpp"
#include "sd_helpers.hpp"

#define SerialPort Serial

#define LPN_PIN_FORWARD 22
#define LPN_PIN_BOTTOM -1
#define PWREN_PIN -1

// Components.
VL53L8CX sensor_vl53l8cx_forward(&Wire, LPN_PIN_FORWARD);
VL53L8CX sensor_vl53l8cx_bottom(&Wire, LPN_PIN_BOTTOM);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L8CX_RESOLUTION_8X8;
uint8_t status;

#define WRITE_TO_SD

#ifdef WRITE_TO_SD
// File for logging data
FsFile dataFile;
// SdFat sd;
char dataFileName[20] = "tof_data_01.csv";
int bufferWriteCount = 0;
#endif // WRITE_TO_SD
/* Setup ---------------------------------------------------------------------*/
void setup()    
{
    // Set LPN pin to open drain, otherwise it doesn't work?
    pinMode(LPN_PIN_FORWARD, OUTPUT_OPENDRAIN);
    sensor_vl53l8cx_forward.on();

    // Initialize serial for output.
    SerialPort.begin(9600);

    #ifdef WRITE_TO_SD
    // Initialize SD card
    serialPrintln("Initializing SD card...");
    if (!SD.sdfs.begin(SdioConfig(FIFO_SDIO)))
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
    // dataFile = sd.open(dataFileName, FILE_WRITE);
    dataFile = SD.sdfs.open(dataFileName, O_RDWR | O_CREAT | O_AT_END);
    // dataFile.preAllocate(10*1024*1024); // Preallocate 10MB for the file
    writeSDHeader(dataFile);
    #endif

    serialPrintln("Starting Crazyflie communication");
    COMMUNICATION_SERIAL.begin(COMMUNICATION_SERIAL_BAUD);

    serialPrintln("Initializing I2C bus");
    // Initialize I2C bus.
    Wire.begin();
    Wire.setClock(400000); // Set I2C clock speed to 400kHz

    // Sensor init and status reporting
    serialPrintln("Initializing sensor");


    scan_i2c();
    delay(3);


    // Set bottom sensor LPN pin low so we can set the i2c address of forward sensor
    sensor_vl53l8cx_forward.off();
    sensor_vl53l8cx_bottom.set_i2c_address(0x54); // Set I2C address for forward sensor
    serialPrintln("Set address");
    delay(50);
    sensor_vl53l8cx_forward.on();
    scan_i2c();
    delay(50);
    sensor_init_and_report(sensor_vl53l8cx_bottom, res);
    sensor_init_and_report(sensor_vl53l8cx_forward, res);
    serialPrintln("Finished setting up sensors");

}

void loop()
{
    receiveCrazyflie();

    VL53L8CX_ResultsData results_bottom = get_sensor_data(sensor_vl53l8cx_bottom);
    uint16_t resultsBottom[4] = {0};
    process_sensor_data(resultsBottom, results_bottom, res);

    serialPrint("Bottom Sensor Data:  ");
    serialPrint("Dist 1: " + String(results_bottom.distance_mm[0]) + " mm, ");
    serialPrint("Status 1: " + String(results_bottom.target_status[0]) + " || ");
    
    VL53L8CX_ResultsData results_forward = get_sensor_data(sensor_vl53l8cx_forward);
    uint16_t resultsForward[4] = {0};
    process_sensor_data(resultsForward, results_forward, res);


    serialPrint("Forward Sensor Data:  ");
    serialPrint("Dist 1: " + String(results_forward.distance_mm[0]) + " mm, ");
    serialPrintln("Status 1: " + String(results_forward.target_status[0]));

    // Store output message to be sent back to CF
    setOutputMessage(resultsBottom, resultsForward, res);

    #ifdef WRITE_TO_SD
    // Log results to SD card
    writeResultsToSD(dataFile, results_bottom, 0, res);
    writeResultsToSD(dataFile, results_forward, 1, res);
    writeFlapperDataToSD(dataFile, myserial_control_in, 2, res);
    bufferWriteCount++;
    if (bufferWriteCount >= 30) // sync every 50 writes
    {
        dataFile.sync(); // Ensure data is written to SD card
        bufferWriteCount = 0;
    }
    #endif

    // Send message via UART to CF
    sendCrazyflie();
}