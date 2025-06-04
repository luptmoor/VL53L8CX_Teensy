#ifndef SD_HELPERS_H
#define SD_HELPERS_H

#include <SD.h>
#include <vl53l8cx.h>

extern char dataFileName[20];
void setNextDataFileName();
void writeResultsToSD(File &dataFile, const VL53L8CX_ResultsData &Results, uint8_t res);
void writeSDHeader(File &dataFile);

#endif // SD_HELPERS_H
