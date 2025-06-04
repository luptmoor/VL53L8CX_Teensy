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

// Components.
VL53L8CX sensor_vl53l8cx_top(&Wire, LPN_PIN);

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
    // Initialize serial for output.
    SerialPort.begin(9600);
    
    serialPrintln("Starting Crazyflie communication");
    COMMUNICATION_SERIAL.begin(COMMUNICATION_SERIAL_BAUD);
    // while (!SerialPort) ; // wait for serial monitor open
    // if (CrashReport) {
    //     SerialPort.print(CrashReport);
    //     delay(5000);
    // }

    // unsigned long serialTimeout = millis() + 2000; // 2s timeout
    // while (!SerialPort && millis() < serialTimeout) {
    //     // Wait for serial port to connect, but only up to timeout
    // }

    serialPrintln("Initializing I2C bus");
    // Initialize I2C bus.
    Wire.begin();
    Wire.setClock(400000); // Set I2C clock speed to 400kHz

    scan_i2c();

    // Initialize SD card
    // serialPrintln("Initializing SD card...");
    // if (!SD.begin(BUILTIN_SDCARD))
    // {
    //     serialPrintln("Card failed, or not present.");
    //     while (1)
    //         ;
    // }
    // serialPrintln("Card initialized.");

    // Find next available data file name
    // setNextDataFileName();
    // serialPrint("Logging to file: ");
    // serialPrintln(String(dataFileName));

    // writeSDHeader(dataFile, dataFileName);

    // Sensor init and status reporting
    serialPrintln("Initializing sensor");
    sensor_init_and_report();
    // delay(6000);
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
          SerialPort.printf("Last control output ll:%d, ml:%d, mr:%d, rr:%d\n", myserial_control_out.dist_ll, myserial_control_out.dist_ml, myserial_control_out.dist_mr, myserial_control_out.dist_rr);
          // DEBUG_serial.printf("CPU temp is %f\n", tempmonGetTemp());
          serial_cf_received_packets = 0;
          timer_count_main = 0;
          timer_receive_outer = 0;
          timer_send_outer = 0;
        }
        VL53L8CX_ResultsData Results = get_sensor_data();

        // Store output message to be sent back to CF
        setOutputMessage(Results, res);

        // Send message via UART to CF
        timer_send = 0;
        sendCrazyflie();
        timer_send_outer = timer_send_outer + timer_send;
        timer_send = 0;
    }
}
