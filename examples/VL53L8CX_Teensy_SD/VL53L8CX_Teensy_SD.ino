/* Includes ------------------------------------------------------------------*/
#include <SD.h>

#include <vl53l8cx.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

#define LPN_PIN -1
#define PWREN_PIN -1

// Helper for safe serial logging
void serialPrint(const String &msg) {
    if (SerialPort) SerialPort.print(msg);
}
void serialPrintln(const String &msg) {
    if (SerialPort) SerialPort.println(msg);
}
void serialPrintf(const char *fmt, ...) {
    if (SerialPort) {
        va_list args;
        va_start(args, fmt);
        SerialPort.vprintf(fmt, args);
        va_end(args);
    }
}

void print_result(VL53L8CX_ResultsData *Result);
void clear_screen(void);
void handle_cmd(uint8_t cmd);
void display_commands_banner(void);

// Components.
VL53L8CX sensor_vl53l8cx_top(&DEV_I2C, LPN_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L8CX_RESOLUTION_8X8;
char report[256];
uint8_t status;

// File for logging data
File dataFile;
char dataFileName[20] = "tof_data_01.csv";

// Helper to find next available index for data file
void setNextDataFileName() {
    int maxIdx = 0;
    for (int idx = 1; idx <= 99; idx++) {
        char fname[20];
        snprintf(fname, sizeof(fname), "tof_data_%02d.csv", idx);
        if (SD.exists(fname)) {
            maxIdx = idx;
        }
    }
    int nextIdx = maxIdx + 1;
    if (nextIdx > 99) nextIdx = 99; // Clamp to 99
    snprintf(dataFileName, sizeof(dataFileName), "tof_data_%02d.csv", nextIdx);
}

void scan_i2c()
{
    serialPrintln("Scanning I2C bus...");
    byte error, address;
    int deviceCount = 0;

    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            serialPrint("I2C device found at address 0x");
            if (address < 16)
                serialPrint("0");
            serialPrint(String(address, HEX));
            serialPrintln("");
            deviceCount++;

            if (address == 0x52)
            {
                serialPrintln(" - VL53L8CX found!");
            }
        }
    }

    if (deviceCount == 0)
    {
        serialPrintln("No I2C devices found");
    }
}

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

    serialPrintln("Configuring component");
    // Configure VL53L8CX component.
    sensor_vl53l8cx_top.begin();
    scan_i2c();

    uint8_t p_alive = 0;
    status = sensor_vl53l8cx_top.is_alive(&p_alive);
    serialPrint("Sensor alive: ");
    serialPrint(String(p_alive));
    serialPrint(" with status: ");
    serialPrintln(String(status));

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
    if (dataFile) {
        serialPrintln("Writing CSV header to SD card...");
        // Write header: Time,Zone0_Distance_mm,Zone0_Status,Zone1_Distance_mm,Zone1_Status,...
        dataFile.print("Time");
        for (int i = 0; i < VL53L8CX_RESOLUTION_8X8; i++) { // Header for max zones (64 for 8x8)
            dataFile.print(",Zone");
            dataFile.print(i);
            dataFile.print("_Distance_mm,Zone");
            dataFile.print(i);
            dataFile.print("_Status");
        }
        dataFile.println();
        dataFile.close();
        serialPrintln("Header written and file closed.");
    } else {
        serialPrintln("Error opening data file to write header!");
        // Consider adding error handling, e.g., halt
    }

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
    status = sensor_vl53l8cx_top.set_ranging_frequency_hz(15);
    if (status)
    {
        serialPrint("Error setting frequency with status: ");
        serialPrintln(String(status));
    }

    serialPrintln("Start measurements");
    // Start Measurements
    status = sensor_vl53l8cx_top.start_ranging();
    if (status)
    {
        serialPrint("Error starting measurements with status: ");
        serialPrintln(String(status));
    }
}

void loop()
{
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
        dataFile = SD.open(dataFileName, FILE_WRITE); // Opens in append mode
        if (dataFile)
        {
            unsigned long timestamp = millis();
            dataFile.print(timestamp);

            // Iterate through zones based on current resolution
            // 'res' holds the number of active zones (16 for 4x4, 64 for 8x8)
            for (int i = 0; i < res; i++) 
            {
                dataFile.print(","); // Separator for the next data pair
                // Check if at least one target is detected in this zone
                if (Results.nb_target_detected[i] > 0) 
                {
                    // Distance of the first target in this zone
                    dataFile.print(Results.distance_mm[(VL53L8CX_NB_TARGET_PER_ZONE * i) + 0]);
                    dataFile.print(",");
                    // Status of the first target in this zone
                    dataFile.print(Results.target_status[(VL53L8CX_NB_TARGET_PER_ZONE * i) + 0]);
                }
                else
                {
                    // No target detected in this zone, print empty placeholders
                    dataFile.print(","); // One comma for empty distance and status
                }
            }
            // If current resolution is 4x4 (16 zones), fill remaining columns for 8x8 header with empty values
            if (res == VL53L8CX_RESOLUTION_4X4) {
                for (int i = VL53L8CX_RESOLUTION_4X4; i < VL53L8CX_RESOLUTION_8X8; i++) {
                    dataFile.print(",,,"); // Two commas for empty distance and status for unused zone columns
                }
            }

            dataFile.println(); // End of line for this measurement
            dataFile.close();
        }
        else // Add diagnostic for loop open failure
        {
            serialPrint("Loop Timestamp: "); 
            serialPrint(String(millis()));
            serialPrintln(" - Error opening data file in loop to write data!");
        }

        print_result(&Results); // Moved outside the if(dataFile) block

        if (SerialPort.available() > 0)
        {
            handle_cmd(SerialPort.read());
        }
        delay(50);
    }
}

void print_result(VL53L8CX_ResultsData * Result)
{
    int8_t i, j, k, l;
    uint8_t zones_per_line;
    uint8_t number_of_zones = res;

    zones_per_line = (number_of_zones == 16) ? 4 : 8;

    display_commands_banner();

    // Print header for the data
    serialPrintln("VL53L8CX Measurement Results:");
    serialPrintln("----------------------------");

    // Show current configuration
    serialPrint("Resolution: ");
    serialPrint((res == VL53L8CX_RESOLUTION_4X4) ? "4x4" : "8x8");
    serialPrint(" | Signal: ");
    serialPrint(EnableSignal ? "ON" : "OFF");
    serialPrint(" | Ambient: ");
    serialPrintln(EnableAmbient ? "ON" : "OFF");
    serialPrintln("");

    // Process each row of zones
    for (j = 0; j < number_of_zones; j += zones_per_line)
    {
        // Print separator line
        for (i = 0; i <= zones_per_line; i++)
        {
            serialPrint("--------");
        }
        serialPrintln("");

        // Print distance values for each target
        for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++)
        {
            if (l == 0)
            {
                serialPrint("Dist  | ");
            }
            else
            {
                serialPrint("Dist" + String(l) + " | ");
            }

            // Print distance for each zone in this row
            for (k = 0; k < zones_per_line; k++)
            {
                if (j + k < number_of_zones)
                { // Make sure we don't go out of bounds
                    if (Result->nb_target_detected[j + k] > l)
                    {
                        serialPrint(String(Result->distance_mm[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]));
                        serialPrint("\t| ");
                    }
                    else
                    {
                        serialPrint("-\t| ");
                    }
                }
            }
            serialPrintln("");

            // Print status values
            if (l == 0)
            {
                serialPrint("State | ");
            }
            else
            {
                serialPrint("State" + String(l) + "| ");
            }

            for (k = 0; k < zones_per_line; k++)
            {
                if (j + k < number_of_zones)
                {
                    if (Result->nb_target_detected[j + k] > l)
                    {
                        serialPrint(String(Result->target_status[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]));
                        serialPrint("\t| ");
                    }
                    else
                    {
                        serialPrint("-\t| ");
                    }
                }
            }
            // SerialPort.println();

            // Print signal and ambient if enabled
            if (EnableSignal)
            {
                if (l == 0)
                {
                    serialPrint("Signal| ");
                }
                else
                {
                    serialPrint("Sig" + String(l) + "  | ");
                }

                for (k = 0; k < zones_per_line; k++)
                {
                    if (j + k < number_of_zones)
                    {
                        if (Result->nb_target_detected[j + k] > l)
                        {
                            serialPrint(String(Result->signal_per_spad[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]));
                            serialPrint("\t| ");
                        }
                        else
                        {
                            serialPrint("-\t| ");
                        }
                    }
                }
                serialPrintln("");
            }

            if (EnableAmbient)
            {
                if (l == 0)
                {
                    serialPrint("Ambnt | ");
                }
                else
                {
                    serialPrint("Amb" + String(l) + "  | ");
                }

                for (k = 0; k < zones_per_line; k++)
                {
                    if (j + k < number_of_zones)
                    {
                        if (Result->nb_target_detected[j + k] > 0)
                        { // Ambient is per zone, not per target
                            serialPrint(String(Result->ambient_per_spad[j + k]));
                            serialPrint("\t| ");
                        }
                        else
                        {
                            serialPrint("-\t| ");
                        }
                    }
                }
                serialPrintln("");
            }

            // Add a separator line between targets
            if (l < VL53L8CX_NB_TARGET_PER_ZONE - 1 &&
                (EnableSignal || EnableAmbient || Result->nb_target_detected[j] > l + 1))
            {
                serialPrintln("--------+--------+--------+--------+--------+--------+--------+--------+");
            }
        }
        serialPrintln("");
    }
}

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
    EnableAmbient = (EnableAmbient) ? false : true;
    EnableSignal = (EnableSignal) ? false : true;
}

void clear_screen(void)
{
    snprintf(report, sizeof(report), "%c[2J", 27); /* 27 is ESC command */
    serialPrint(report);
}

void display_commands_banner(void)
{
    snprintf(report, sizeof(report), "%c[2H", 27); /* 27 is ESC command */
    serialPrint(report);

    serialPrint("53L8A1 Simple Ranging demo application\n");
    serialPrint("--------------------------------------\n\n");

    serialPrint("Use the following keys to control application\n");
    serialPrint(" 'r' : change resolution\n");
    serialPrint(" 's' : enable signal and ambient\n");
    serialPrint(" 'c' : clear screen\n");
    serialPrint("\n");
}

void handle_cmd(uint8_t cmd)
{
    switch (cmd)
    {
    case 'r':
        toggle_resolution();
        clear_screen();
        break;

    case 's':
        toggle_signal_and_ambient();
        clear_screen();
        break;

    case 'c':
        clear_screen();
        break;

    default:
        break;
    }
}
