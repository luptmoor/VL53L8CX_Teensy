/**
 ******************************************************************************
 * @file    VL53L8CX_HelloWorld_SPI.ino
 * @author  STMicroelectronics
 * @version V2.0.0
 * @date    27 June 2024
 * @brief   Arduino test application for the X-NUCLEO-53L8A1 based on VL53L8CX
 *          proximity sensor.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2024 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*
 * To use these examples you need to connect the VL53L8CX satellite sensor directly to the Nucleo board with wires as explained below:
 * pin 1 (SPI_I2C_n) of the VL53L8CX satellite connected to 3V3 of the Nucleo board
 * pin 2 (LPn) of the VL53L8CX satellite connected to 3V3 of the Nucleo board
 * pin 3 (NCS) of the VL53L8CX satellite connected to pin D10 of the Nucleo board
 * pin 4 (MISO) of the VL53L8CX satellite connected to pin D5 of the Nucleo board
 * pin 5 (MOSI_SDA) of the VL53L8CX satellite connected to pin D4 (MOSI) of the Nucleo board
 * pin 6 (MCLK_SCL) of the VL53L8CX satellite connected to pin D3 (MCLK_SCL) of the Nucleo board
 * pin 7 (PWREN) of the VL53L8CX satellite connected to pin D11 of the Nucleo board
 * pin 8 (I0VDD) of the VL53L8CX satellite not connected
 * pin 9 (3V3) of the VL53L8CX satellite not connected
 * pin 10 (1V8) of the VL53L8CX satellite not connected
 * pin 11 (5V) of the VL53L8CX satellite connected to 5V of the Nucleo board
 * GPIO1 of VL53L8CX satellite connected to A2 pin of the Nucleo board (not used)
 * GND of the VL53L8CX satellite connected to GND of the Nucleo board
 */

/* Includes ------------------------------------------------------------------*/

#include <SPI.h>
#include <vl53l8cx.h>

#define SerialPort Serial

#define PWREN_PIN 11

#define SPI_CLK_PIN 3
#define SPI_MISO_PIN  5
#define SPI_MOSI_PIN 4
#define CS_PIN 10

SPIClass DEV_SPI(SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CLK_PIN);

void print_result(VL53L8CX_ResultsData *Result);
void clear_screen(void);
void handle_cmd(uint8_t cmd);
void display_commands_banner(void);

VL53L8CX sensor_vl53l8cx_top(&DEV_SPI, CS_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L8CX_RESOLUTION_4X4;
char report[256];
uint8_t status;
/* Setup ---------------------------------------------------------------------*/
void setup()
{
  // Enable PWREN pin if present
  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  // Initialize serial for output.
  SerialPort.begin(460800);

  // Initialize SPI bus.
  DEV_SPI.begin();

  // Configure VL53L8CX component.
  sensor_vl53l8cx_top.begin();
  status = sensor_vl53l8cx_top.init();

  // Start Measurements
  status = sensor_vl53l8cx_top.start_ranging();
}

void loop()
{
  VL53L8CX_ResultsData Results;
  uint8_t NewDataReady = 0;
  do {
    status = sensor_vl53l8cx_top.check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l8cx_top.get_ranging_data(&Results);
    print_result(&Results);
  }

  if (Serial.available() > 0) {
    handle_cmd(Serial.read());
  }
  delay(50);
}

void print_result(VL53L8CX_ResultsData *Result)
{
  int8_t i, j, k, l;
  uint8_t zones_per_line;
  uint8_t number_of_zones = res;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  display_commands_banner();

  // Print header for the data
  SerialPort.println("VL53L8CX Measurement Results:");
  SerialPort.println("----------------------------");
  
  // Show current configuration
  SerialPort.print("Resolution: ");
  SerialPort.print((res == VL53L8CX_RESOLUTION_4X4) ? "4x4" : "8x8");
  SerialPort.print(" | Signal: ");
  SerialPort.print(EnableSignal ? "ON" : "OFF");
  SerialPort.print(" | Ambient: ");
  SerialPort.println(EnableAmbient ? "ON" : "OFF");
  SerialPort.println();

  // Process each row of zones
  for (j = 0; j < number_of_zones; j += zones_per_line) {
    // Print row header
    SerialPort.print("Row ");
    SerialPort.print(j / zones_per_line);
    SerialPort.println(":");
    
    // Print column headers
    SerialPort.print("Zone | ");
    for (i = 0; i < zones_per_line; i++) {
      SerialPort.print(String(j + i) + "\t| ");
    }
    SerialPort.println();
    
    // Print separator line
    for (i = 0; i <= zones_per_line; i++) {
      SerialPort.print("--------");
    }
    SerialPort.println();

    // Print distance values for each target
    for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
      if (l == 0) {
        SerialPort.print("Dist  | ");
      } else {
        SerialPort.print("Dist" + String(l) + " | ");
      }
      
      // Print distance for each zone in this row
      for (k = 0; k < zones_per_line; k++) {
        if (j + k < number_of_zones) { // Make sure we don't go out of bounds
          if (Result->nb_target_detected[j + k] > l) {
            SerialPort.print(Result->distance_mm[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
            SerialPort.print("\t| ");
          } else {
            SerialPort.print("-\t| ");
          }
        }
      }
      SerialPort.println();
      
      // Print status values
      if (l == 0) {
        SerialPort.print("State | ");
      } else {
        SerialPort.print("State" + String(l) + "| ");
      }
      
      for (k = 0; k < zones_per_line; k++) {
        if (j + k < number_of_zones) {
          if (Result->nb_target_detected[j + k] > l) {
            SerialPort.print(Result->target_status[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
            SerialPort.print("\t| ");
          } else {
            SerialPort.print("-\t| ");
          }
        }
      }
      SerialPort.println();

      // Print signal and ambient if enabled
      if (EnableSignal) {
        if (l == 0) {
          SerialPort.print("Signal| ");
        } else {
          SerialPort.print("Sig" + String(l) + "  | ");
        }
        
        for (k = 0; k < zones_per_line; k++) {
          if (j + k < number_of_zones) {
            if (Result->nb_target_detected[j + k] > l) {
              SerialPort.print(Result->signal_per_spad[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
              SerialPort.print("\t| ");
            } else {
              SerialPort.print("-\t| ");
            }
          }
        }
        SerialPort.println();
      }
      
      if (EnableAmbient) {
        if (l == 0) {
          SerialPort.print("Ambnt | ");
        } else {
          SerialPort.print("Amb" + String(l) + "  | ");
        }
        
        for (k = 0; k < zones_per_line; k++) {
          if (j + k < number_of_zones) {
            if (Result->nb_target_detected[j + k] > 0) { // Ambient is per zone, not per target
              SerialPort.print(Result->ambient_per_spad[j + k]);
              SerialPort.print("\t| ");
            } else {
              SerialPort.print("-\t| ");
            }
          }
        }
        SerialPort.println();
      }
      
      // Add a separator line between targets
      if (l < VL53L8CX_NB_TARGET_PER_ZONE - 1 && 
          (EnableSignal || EnableAmbient || Result->nb_target_detected[j] > l + 1)) {
        SerialPort.println("--------+--------+--------+--------+--------+--------+--------+--------+");
      }
    }
    SerialPort.println();
  }
}

void toggle_resolution(void)
{
  status = sensor_vl53l8cx_top.stop_ranging();

  switch (res) {
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
  SerialPort.print(report);
}

void display_commands_banner(void)
{
  snprintf(report, sizeof(report), "%c[2H", 27); /* 27 is ESC command */
  SerialPort.print(report);

  Serial.print("53L8A1 Simple Ranging demo application\n");
  Serial.print("--------------------------------------\n\n");

  Serial.print("Use the following keys to control application\n");
  Serial.print(" 'r' : change resolution\n");
  Serial.print(" 's' : enable signal and ambient\n");
  Serial.print(" 'c' : clear screen\n");
  Serial.print("\n");
}

void handle_cmd(uint8_t cmd)
{
  switch (cmd) {
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
