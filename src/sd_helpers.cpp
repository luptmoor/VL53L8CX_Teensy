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

void writeResultsToSD(File &dataFile, const VL53L8CX_ResultsData &Results, uint8_t res) {
    char lineBuffer[2048]; // Large enough for a full 8x8 line
    int pos = 0;
    unsigned long timestamp = millis();
    pos += snprintf(lineBuffer + pos, sizeof(lineBuffer) - pos, "%lu", timestamp);

    // Iterate through zones based on current resolution
    for (int i = 0; i < res; i++) {
        pos += snprintf(lineBuffer + pos, sizeof(lineBuffer) - pos, ",");
        if (Results.nb_target_detected[i] > 0) {
            pos += snprintf(lineBuffer + pos, sizeof(lineBuffer) - pos, "%u,%u",
                Results.distance_mm[(VL53L8CX_NB_TARGET_PER_ZONE * i) + 0],
                Results.target_status[(VL53L8CX_NB_TARGET_PER_ZONE * i) + 0]);
        } else {
            pos += snprintf(lineBuffer + pos, sizeof(lineBuffer) - pos, ",");
        }
    }
    // If current resolution is 4x4 (16 zones), fill remaining columns for 8x8 header with empty values
    if (res == VL53L8CX_RESOLUTION_4X4) {
        for (int i = VL53L8CX_RESOLUTION_4X4; i < VL53L8CX_RESOLUTION_8X8; i++) {
            pos += snprintf(lineBuffer + pos, sizeof(lineBuffer) - pos, ",,");
        }
    }

    if (dataFile) {
        dataFile.println(lineBuffer);
    } else {
        serialPrint("Loop Timestamp: "); 
        serialPrint(String(millis()));
        serialPrintln(" - Data file not open for writing!");
    }
}

void writeSDHeader(File &dataFile) {
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
        serialPrintln("Header written.");
    } else {
        serialPrintln("Error: data file not open to write header!");
    }
}
