// #include <Arduino.h>
#include "cf_helpers.hpp"

// -------------------------- COMMUNICATION DEFINED VARIABLES-----------------------------
const uint8_t START_BYTE_SERIAL_CF = 0x9A;
elapsedMicros last_time_write_to_cf = 0;
int ack_comm_TX_cf = 0; 

uint8_t serial_cf_msg_buf_in[ 2*sizeof(struct serial_control_in) ] = {0};
uint16_t serial_cf_buf_in_cnt = 0;
uint32_t serial_cf_received_packets = 0;
uint32_t serial_cf_missed_packets_in = 0;

volatile float extra_data_out[255]__attribute__((aligned));

bool sending;
bool receiving = true;

struct serial_control_in myserial_control_in;
volatile struct serial_control_out myserial_control_out;
float inputs[9];
void *controller = NULL;

uint32_t timer_receive = 0;
uint32_t timer_receive_outer = 0;
uint32_t timer_send = 0;
uint32_t timer_send_outer = 0;
uint32_t timer_count_main = 0;



static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}


void serialParseMessageIn(void)
{
  //Copy received buffer to structure
  memmove(&myserial_control_in,&serial_cf_msg_buf_in[1],sizeof(struct serial_control_in)-1);
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
    inputs[8] = myserial_control_in.yaw * 0.03f;
}

void setOutputMessage(void)
{
    myserial_control_out.torque_x = myserial_control_in.roll;
    myserial_control_out.torque_y = myserial_control_in.pitch;
    myserial_control_out.torque_z = myserial_control_in.yaw;
    myserial_control_out.x_integ = saturateSignedInt16(0);
    myserial_control_out.y_integ = saturateSignedInt16(0);
}


void sendCrazyflie(void)
{
    //SENDING PACKET

    //Calculate checksum for outbound packet: 
    uint8_t *buf_send = (uint8_t *)&myserial_control_out;
    myserial_control_out.checksum_out = 0;
    for(uint16_t i = 0; i < sizeof(struct serial_control_out) - 1; i++){
        myserial_control_out.checksum_out += buf_send [i];
    }
    
    //Send out packet to buffer:
    noInterrupts();
    COMMUNICATION_SERIAL.write(START_BYTE_SERIAL_CF);
    COMMUNICATION_SERIAL.write(buf_send,sizeof(struct serial_control_out));
    interrupts();
    
    last_time_write_to_cf = 0;

    sending = false;
    receiving = true;
}


void receiveCrazyflie(void)
{
  //RECEIVING PACKET
  //Collect packets on the buffer if available:
    while(COMMUNICATION_SERIAL.available()) {
        // DEBUG_serial.write("trying to read...\n");

        timer_receive = 0;
        uint8_t serial_cf_byte_in;
        serial_cf_byte_in = COMMUNICATION_SERIAL.read();
        if ((serial_cf_byte_in == START_BYTE_SERIAL_CF) || (serial_cf_buf_in_cnt > 0)) {
            serial_cf_msg_buf_in[serial_cf_buf_in_cnt] = serial_cf_byte_in;
            serial_cf_buf_in_cnt++;
        }
        if (serial_cf_buf_in_cnt > sizeof(struct serial_control_in)  ) {
            serial_cf_buf_in_cnt = 0;
            uint8_t checksum_in_local = 0;
            for(uint16_t i = 1; i < sizeof(struct serial_control_in) ; i++){
                checksum_in_local += serial_cf_msg_buf_in[i];
            }
            if(checksum_in_local == serial_cf_msg_buf_in[sizeof(struct serial_control_in)]){
                serialParseMessageIn();
                serial_cf_received_packets++;
                
            }
            else {
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