// #include <Arduino.h>
#include "cf_helpers.hpp"
#include "serial_helpers.h"

// -------------------------- COMMUNICATION DEFINED VARIABLES-----------------------------
byte START_BYTE_SERIAL_CF = 0x9A;
elapsedMicros last_time_write_to_cf = 0;
int ack_comm_TX_cf = 0;

uint8_t serial_cf_msg_buf_in[2 * sizeof(struct serial_control_in)] = {0};
uint16_t serial_cf_buf_in_cnt = 0;
uint32_t serial_cf_received_packets = 0;
uint32_t serial_cf_missed_packets_in = 0;

bool sending = false;
bool receiving = true;

struct serial_control_in myserial_control_in;
volatile struct serial_control_out myserial_control_out;
float inputs[9];
void *controller = NULL;

elapsedMicros timer_receive = 0;
uint32_t timer_receive_outer = 0;
elapsedMicros timer_send = 0;
uint32_t timer_send_outer = 0;
elapsedMicros timer_count_main = 0;


void serialParseMessageIn(void)
{
    // Copy received buffer to structure
    memmove(&myserial_control_in, &serial_cf_msg_buf_in[1], sizeof(struct serial_control_in) - 1);
}

void setInputMessage(void)
{
    inputs[0] = myserial_control_in.roll_gyro * 0.01f;
    inputs[1] = myserial_control_in.pitch_gyro * 0.01f;
    inputs[2] = myserial_control_in.yaw_gyro * 0.01f;
    inputs[3] = myserial_control_in.x_acc;
    inputs[4] = myserial_control_in.y_acc;
    inputs[5] = myserial_control_in.z_acc * 0.3f;
    inputs[6] = myserial_control_in.roll * 0.03f;
    inputs[7] = myserial_control_in.pitch * 0.03f;
    inputs[8] = myserial_control_in.yaw_t * 0.03f;
}

void resetOutputMessage(void)
{
    myserial_control_out.dist_ll_forward = 0;
    myserial_control_out.dist_ml_forward = 0;
    myserial_control_out.dist_mr_forward = 0;
    myserial_control_out.dist_rr_forward = 0;

    myserial_control_out.dist_ll_bottom = 0;
    myserial_control_out.dist_ml_bottom = 0;
    myserial_control_out.dist_mr_bottom = 0;
    myserial_control_out.dist_rr_bottom = 0;
    myserial_control_out.checksum_out = 0;
}

// void setOutputMessage(const VL53L8CX_ResultsData &results_bottom, const VL53L8CX_ResultsData &results_forward, uint8_t res)
void setOutputMessage(uint16_t resultsBottom[4], uint16_t resultsForward[4], uint8_t res)
{
    // Reset the output message
    resetOutputMessage();

    myserial_control_out.dist_ll_bottom = resultsBottom[3];
    myserial_control_out.dist_ml_bottom = resultsBottom[2];
    myserial_control_out.dist_mr_bottom = resultsBottom[1];
    myserial_control_out.dist_rr_bottom = resultsBottom[0];
    myserial_control_out.dist_ll_forward = resultsForward[3];
    myserial_control_out.dist_ml_forward = resultsForward[2];
    myserial_control_out.dist_mr_forward = resultsForward[1];
    myserial_control_out.dist_rr_forward = resultsForward[0];
}

void sendCrazyflie(void)
{
    // SENDING PACKET

    // Calculate checksum for outbound packet:
    uint8_t *buf_send = (uint8_t *)&myserial_control_out;
    myserial_control_out.checksum_out = 0;
    for (uint16_t i = 0; i < sizeof(struct serial_control_out) - 1; i++)
    {
        myserial_control_out.checksum_out += buf_send[i];
    }

    // Send out packet to buffer:
    noInterrupts();
    COMMUNICATION_SERIAL.write(START_BYTE_SERIAL_CF);
    COMMUNICATION_SERIAL.write(buf_send, sizeof(struct serial_control_out));
    interrupts();

    last_time_write_to_cf = 0;

    sending = false;
    receiving = true;
}

void receiveCrazyflie(void)
{
    // RECEIVING PACKET
    // Collect packets on the buffer if available:
    while (COMMUNICATION_SERIAL.available())
    {
        timer_receive = 0;
        uint8_t serial_cf_byte_in;
        serial_cf_byte_in = COMMUNICATION_SERIAL.read();
        if ((serial_cf_byte_in == START_BYTE_SERIAL_CF) || (serial_cf_buf_in_cnt > 0))
        {
            serial_cf_msg_buf_in[serial_cf_buf_in_cnt] = serial_cf_byte_in;
            serial_cf_buf_in_cnt++;
        }
        if (serial_cf_buf_in_cnt > sizeof(struct serial_control_in))
        {
            serial_cf_buf_in_cnt = 0;
            uint8_t checksum_in_local = 0;
            for (uint16_t i = 1; i < sizeof(struct serial_control_in); i++)
            {
                checksum_in_local += serial_cf_msg_buf_in[i];
            }
            if (checksum_in_local == serial_cf_msg_buf_in[sizeof(struct serial_control_in)])
            {
                serialParseMessageIn();
                serial_cf_received_packets++;
            }
            else
            {
                serial_cf_missed_packets_in++;
                DEBUG_serial.write("Incorrect message\n");
            }
            receiving = false;
            sending = true;
        }
        timer_receive_outer = timer_receive_outer + timer_receive;
        timer_receive = 0;
    }
}