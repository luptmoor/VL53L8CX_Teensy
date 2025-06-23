#include <unordered_set>
// #include <Wire.h>

#include "sensor_helpers.h"
#include "serial_helpers.h"

// Sensor parameters
static std::unordered_set<uint8_t> valid_status { 5, 9, 10, 255 };

void sensor_init_and_report(VL53L8CX &sensor_vl53l8cx, uint8_t res)
{
    serialPrintln("Configuring component");
    sensor_vl53l8cx.begin();

    uint8_t p_alive = 0;
    uint8_t status;
    status = sensor_vl53l8cx.is_alive(&p_alive);
    serialPrint("Sensor alive: ");
    serialPrint(String(p_alive));
    serialPrint(" with status: ");
    serialPrintln(String(status));

    // Wire.beginTransmission(0x29);
    // uint8_t err = Wire.endTransmission();
    // serialPrintf("error = %u (should be 0)\n", err);

    if (status)
    {
        serialPrint("Error checking sensor alive with status: ");
        serialPrintln(String(status));
    }

    serialPrintln("Calling init");
    status = sensor_vl53l8cx.init();
    if (status)
    {
        serialPrint("Error initializing sensor with status: ");
        serialPrintln(String(status));
    }

    status = sensor_vl53l8cx.set_resolution(res);
    if (status)
    {
        serialPrint("Error setting resolution with status: ");
        serialPrintln(String(status));
    }
    status = sensor_vl53l8cx.set_ranging_frequency_hz(30);
    if (status)
    {
        serialPrint("Error setting frequency with status: ");
        serialPrintln(String(status));
    }

    sensor_vl53l8cx.set_ranging_mode(VL53L8CX_RANGING_MODE_CONTINUOUS);
    uint8_t p_ranging_mode = 0;
    sensor_vl53l8cx.get_ranging_mode(&p_ranging_mode);
    serialPrint("Ranging mode: ");
    serialPrintln(String(p_ranging_mode));

    serialPrintln("Start measurements");
    status = sensor_vl53l8cx.start_ranging();
    if (status)
    {
        serialPrint("Error starting measurements with status: ");
        serialPrintln(String(status));
    }
}

VL53L8CX_ResultsData get_sensor_data(VL53L8CX &sensor_vl53l8cx)
{

    VL53L8CX_ResultsData Results;
    uint8_t NewDataReady = 0;
    uint8_t status;

    do
    {
        status = sensor_vl53l8cx.check_data_ready(&NewDataReady);
        delay(5);
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0))
    {
        status = sensor_vl53l8cx.get_ranging_data(&Results);
        return Results;
    }
    else
    {
        serialPrint("Error getting sensor data with status: ");
        serialPrintln(String(status));
        VL53L8CX_ResultsData emptyResults = {};
        return emptyResults; // Return an empty results structure on error
    }
}

void process_sensor_data(uint16_t columnAverages[4], const VL53L8CX_ResultsData &results, uint8_t res)
{
    // Processes the results into 4 averaged columns for 4x4 or 8x8 resolution
    // Keep in mind that the results correspond to the numbering of the zones, 
    // which is 0 for bottom right and N for top left, so averaged columns for the 
    // forward sensor are now ordered from right to left for the forward sensor
    
    // Construct useful distance array
    uint8_t res_size = (res == VL53L8CX_RESOLUTION_4X4) ? 4 : 8; // 4x4 or 8x8 resolution
    uint16_t filteredDistances[res_size][res_size];

    for (int idx = 0; idx < res; ++idx)
    {
        int row = idx / res_size;           // 0 = bottom row ... 7 = top row
        int col = idx % res_size;
        filteredDistances[row][col] = valid_status.count(results.target_status[idx]) ? results.distance_mm[idx] : 0;
        filteredDistances[row][col] = max(filteredDistances[row][col], 0); // Ensure no negative distances
        filteredDistances[row][col] = (results.target_status[idx] == 255) ? 4000 : filteredDistances[row][col]; // Set 4000 for no target found
    }


    // Construct backup array from surrounding pixels
    uint16_t averagedDistances[res_size][res_size] = {}; // Only up to res_size x res_size used

    for (int row = 0; row < res_size; ++row) {
        for (int col = 0; col < res_size; ++col) {
            // Compute average of up to 8 neighbors (not including self)
            uint32_t sum = 0;
            uint8_t count = 0;
            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    if (dr == 0 && dc == 0) continue; // skip self
                    int nr = row + dr;
                    int nc = col + dc;
                    if (nr >= 0 && nr < res_size && nc >= 0 && nc < res_size) {
                        if (filteredDistances[nr][nc] > 0) {
                            sum += filteredDistances[nr][nc];
                            ++count;
                        }
                    }
                }
            }
            if (count > 0)
                averagedDistances[row][col] = sum / count;
            else
                averagedDistances[row][col] = 0;
        }
    }



    // --- Outlier Detection and Replacement ---

    // Determine number of rows and columns based on resolution
    int rowStart = (res_size == 8) ? 1 : 0; // For 8x8, skip first and last row; for 4x4, use all rows
    int rowEnd = (res_size == 8) ? 6 : (res_size - 1);

    for (int row = rowStart; row <= rowEnd; ++row) {
        for (int group = 0; group < 4; ++group) {
            int col0 = group * (res_size / 4);
            int col1 = col0 + 1;

            // Gather valid (non-zero) values for this group in this row
            uint16_t vals[2];
            int valCount = 0;
            if (filteredDistances[row][col0] > 0) vals[valCount++] = filteredDistances[row][col0];
            if (filteredDistances[row][col1] > 0) vals[valCount++] = filteredDistances[row][col1];

            // Compute mean and stddev if at least 2 values
            if (valCount > 1) {
                float mean = (vals[0] + vals[1]) / 2.0f;
                float var = ((vals[0] - mean) * (vals[0] - mean) + (vals[1] - mean) * (vals[1] - mean)) / 2.0f;
                float stddev = sqrt(var);

                // Check and replace outliers
                if (fabs(filteredDistances[row][col0] - mean) > 2.0f * stddev) {
                    filteredDistances[row][col0] = averagedDistances[row][col0];
                }
                if (fabs(filteredDistances[row][col1] - mean) > 2.0f * stddev) {
                    filteredDistances[row][col1] = averagedDistances[row][col1];
                }
            }
        }
    }

    // --- Column Averaging ---
    uint8_t counts[4] = {0};

    for (int row = rowStart; row <= rowEnd; ++row) {
        for (int group = 0; group < 4; ++group) {
            int col0 = group * (res_size / 4);
            int col1 = col0 + 1;

            // Only average non-zero values
            if (filteredDistances[row][col0] > 0) {
                columnAverages[group] += filteredDistances[row][col0];
                counts[group]++;
            }
            if (filteredDistances[row][col1] > 0) {
                columnAverages[group] += filteredDistances[row][col1];
                counts[group]++;
            }
        }
    }
    for (int group = 0; group < 4; ++group) {
        if (counts[group] > 0) {
            columnAverages[group] /= counts[group];
        } else {
            columnAverages[group] = 0;
        }
    }








}