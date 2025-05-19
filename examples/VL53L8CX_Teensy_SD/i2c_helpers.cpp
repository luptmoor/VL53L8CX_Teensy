#include <Wire.h>
#include "serial_helpers.h"
#include "i2c_helpers.h"

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
