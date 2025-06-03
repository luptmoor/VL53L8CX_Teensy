/* Includes ------------------------------------------------------------------*/
#include <SD.h>

#include "vl53l8cx.h"

#include "serial_helpers.h"
#include "sd_helpers.h"
#include "i2c_helpers.h"
#include "display_helpers.h"
#include "sensor_helpers.h"
#include "cf_helpers.hpp"

#define DEV_I2C Wire
#define SerialPort Serial

#define LPN_PIN -1
#define PWREN_PIN -1

// Components.
VL53L8CX sensor_vl53l8cx_top(&DEV_I2C, LPN_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L8CX_RESOLUTION_4X4;
// char report[256];
uint8_t status;

// File for logging data
File dataFile;
char dataFileName[20] = "tof_data_01.csv";

/* Setup ---------------------------------------------------------------------*/
void setup()
{

    // Enable PWREN pin if present
    if (PWREN_PIN >= 0)
    {
        pinMode(PWREN_PIN, OUTPUT);
        digitalWrite(PWREN_PIN, HIGH);
        delay(10);
    }

    // Initialize serial for output.
    SerialPort.begin(9600);
    unsigned long serialTimeout = millis() + 2000; // 2s timeout
    while (!SerialPort && millis() < serialTimeout) {
        // Wait for serial port to connect, but only up to timeout
    }

    serialPrintln("Initializing I2C bus");
    // Initialize I2C bus.
    DEV_I2C.begin();
    DEV_I2C.setClock(400000); // Set I2C clock speed to 400kHz

    scan_i2c();

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

    writeSDHeader(dataFile, dataFileName);

    // Sensor init and status reporting
    sensor_init_and_report();

    COMMUNICATION_SERIAL.begin(COMMUNICATION_SERIAL_BAUD);
    serialPrintln("Starting Crazyflie communication");

}

void loop()
{    
    if (receiving) {
        // SerialPort.printf("Receiving messages...\n");
        receiveCrazyflie();
    } else if (sending) {
        // Timer for debugging
        if (timer_count_main > 1000000) {
          SerialPort.printf("Received %i packets over last second\n", serial_cf_received_packets);
          SerialPort.printf("Receiving took %i ms\n", timer_receive_outer / 1000);
          SerialPort.printf("Sending took %i ms\n", timer_send_outer / 1000);
          SerialPort.printf("Last control output x:%d, y:%d, z:%d\n", myserial_control_out.torque_x, myserial_control_out.torque_y, myserial_control_out.torque_z);
          // DEBUG_serial.printf("CPU temp is %f\n", tempmonGetTemp());
          serial_cf_received_packets = 0;
          timer_count_main = 0;
          timer_receive_outer = 0;
          timer_send_outer = 0;
        }

        VL53L8CX_ResultsData Results;
        uint8_t NewDataReady = 0;

        do
        {
            status = sensor_vl53l8cx_top.check_data_ready(&NewDataReady);
        } while (!NewDataReady);

        if ((!status) && (NewDataReady != 0))
        {
            status = sensor_vl53l8cx_top.get_ranging_data(&Results);

            // Log data to SD card
            writeResultsToSD(dataFile, dataFileName, Results, res);

            // serialPrintMeasurementResult(&Results, res, EnableSignal, EnableAmbient);

            if (SerialPort.available() > 0)
            {
                handle_cmd(SerialPort.read());
            }
            // delay(50);
        }

        
        // Store output message to be sent back to CF
        setOutputMessage(&Results, res);

        // Send message via UART to CF
        timer_send = 0;
        sendCrazyflie();
        timer_send_outer = timer_send_outer + timer_send;
        timer_send = 0;
    }
}
