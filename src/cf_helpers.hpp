#ifndef CF_HELPERS_H
#define CF_HELPERS_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include "cf_msgs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COMMUNICATION_SERIAL_BAUD 460800
#define COMMUNICATION_SERIAL Serial1
#define DEBUG_serial Serial

// Extern variables (assumed from usage in cf_helpers.c)
extern struct serial_control_in myserial_control_in;
extern volatile struct serial_control_out myserial_control_out;
extern uint8_t serial_cf_msg_buf_in[];
extern uint16_t serial_cf_buf_in_cnt;
extern float inputs[];
extern void *controller;
extern elapsedMicros last_time_write_to_cf;
extern bool sending;
extern bool receiving;
extern elapsedMicros timer_count_main;
extern uint32_t timer_send_outer;
extern elapsedMicros timer_send;
extern elapsedMicros timer_receive;
extern uint32_t timer_receive_outer;
extern uint32_t serial_cf_received_packets;
extern uint32_t serial_cf_missed_packets_in;
extern byte START_BYTE_SERIAL_CF;

// Function declarations
void serialParseMessageIn(void);
void setInputMessage(void);
void setOutputMessage(const VL53L8CX_ResultsData &Results, uint8_t res);
void sendCrazyflie(void);
void receiveCrazyflie(void);

#ifdef __cplusplus
}
#endif

#endif // CF_HELPERS_H
