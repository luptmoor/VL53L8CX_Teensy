#include "sd_helpers.h"
#include "serial_helpers.h"
#include <SD.h>

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
    if (nextIdx > 99) nextIdx = 99;
    snprintf(dataFileName, sizeof(dataFileName), "tof_data_%02d.csv", nextIdx);
}

void writeResultsToSD(File &dataFile, const char *dataFileName, const VL53L8CX_ResultsData &Results, uint8_t res) {
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
                dataFile.print(",,"); // Two commas for empty distance and status for unused zone columns
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
}

void writeSDHeader(File &dataFile, const char *dataFileName) {
    dataFile = SD.open(dataFileName, FILE_WRITE);
    if (dataFile) {
        serialPrintln("Writing CSV header to SD card...");
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
}
