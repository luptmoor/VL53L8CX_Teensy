#ifndef SD_HELPERS_H
#define SD_HELPERS_H

#include <SD.h>
#include "vl53l8cx.h"
#include "cf_msgs.h"

// extern SdFat sd;

extern char dataFileName[20];
void setNextDataFileName();
void writeResultsToSD(FsFile &dataFile, const VL53L8CX_ResultsData &Results, uint8_t sensor, uint8_t res);
void writeFlapperDataToSD(FsFile &dataFile, const serial_control_in &results, uint8_t sensor, uint8_t res);
void writeSDHeader(FsFile &dataFile);

#endif // SD_HELPERS_H
