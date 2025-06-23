#include "serial_helpers.h"

#define SerialPort Serial

extern VL53L8CX sensor_vl53l8cx_top;
extern bool EnableAmbient;
extern bool EnableSignal;
extern uint8_t res;
extern uint8_t status;


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

void display_commands_banner(void)
{
    // snprintf(report, sizeof(report), "%c[2H", 27); /* 27 is ESC command */
    // serialPrint(report);
    serialPrint("Simple Ranging demo application\n");
    serialPrint("--------------------------------------\n\n");
    serialPrint("Use the following keys to control application\n");
    serialPrint(" 'r' : change resolution\n");
    serialPrint(" 's' : enable signal and ambient\n");
    serialPrint(" 'c' : clear screen\n");
    serialPrint("\n");
}

void serialPrintMeasurementResult(VL53L8CX_ResultsData *Result, uint8_t res, bool EnableSignal, bool EnableAmbient) {
    int8_t i, j, k, l;
    uint8_t zones_per_line;
    uint8_t number_of_zones = res;

    zones_per_line = (number_of_zones == 16) ? 4 : 8;

    // display_commands_banner();

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
    EnableAmbient = !EnableAmbient;
    EnableSignal = !EnableSignal;
}

void handle_cmd(uint8_t cmd)
{
    switch (cmd)
    {
    case 'r':
        toggle_resolution();
        break;
    case 's':
        toggle_signal_and_ambient();
        break;
    case 'c':
        break;
    default:
        break;
    }
}
