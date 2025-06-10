// #include <Arduino.h>
#include "cf_helpers.hpp"
#include "serial_helpers.h"

// -------------------------- COMMUNICATION DEFINED VARIABLES-----------------------------
byte START_BYTE_SERIAL_CF = 0x9A;
elapsedMicros last_time_write_to_cf = 0;
int ack_comm_TX_cf = 0; 

uint8_t serial_cf_msg_buf_in[ 2*sizeof(struct serial_control_in) ] = {0};
uint16_t serial_cf_buf_in_cnt = 0;
uint32_t serial_cf_received_packets = 0;
uint32_t serial_cf_missed_packets_in = 0;

volatile float extra_data_out[255]__attribute__((aligned));

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

void setOutputMessage(const VL53L8CX_ResultsData &results_bottom, const VL53L8CX_ResultsData &results_forward, uint8_t res)
{
    // Reset the output message
    resetOutputMessage();

    // first calculate the average distance from the 4x4 grid
    // this is done by summing the distances of the 4 zones and dividing by the number of targets detected
    // only the zones that have at least one target detected are considered
    for (uint16_t i = 0; i < 4 ; i++){
        myserial_control_out.dist_ll_forward += (results_forward.nb_target_detected[i*4 + 3] > 0) ? results_forward.distance_mm[i*4 + 3] : 0;
        myserial_control_out.dist_ml_forward += (results_forward.nb_target_detected[i*4 + 2] > 0) ? results_forward.distance_mm[i*4 + 2] : 0;
        myserial_control_out.dist_mr_forward += (results_forward.nb_target_detected[i*4 + 1] > 0) ? results_forward.distance_mm[i*4 + 1] : 0;
        myserial_control_out.dist_rr_forward += (results_forward.nb_target_detected[i*4 + 0] > 0) ? results_forward.distance_mm[i*4 + 0] : 0;
    }
    // divide by the number of targets detected in the 4 zones
    myserial_control_out.dist_ll_forward /= (results_forward.nb_target_detected[3] + results_forward.nb_target_detected[7] + results_forward.nb_target_detected[11] + results_forward.nb_target_detected[15]);
    myserial_control_out.dist_ml_forward /= (results_forward.nb_target_detected[2] + results_forward.nb_target_detected[6] + results_forward.nb_target_detected[10] + results_forward.nb_target_detected[14]);
    myserial_control_out.dist_mr_forward /= (results_forward.nb_target_detected[1] + results_forward.nb_target_detected[5] + results_forward.nb_target_detected[9] + results_forward.nb_target_detected[13]);
    myserial_control_out.dist_rr_forward /= (results_forward.nb_target_detected[0] + results_forward.nb_target_detected[4] + results_forward.nb_target_detected[8] + results_forward.nb_target_detected[12]);

    // same for bottom sensor TODO: this still assumes the left-to-right assumptions, which is not correct for bottom sensor of course. Left is now back and right is front.  
    for (uint16_t i = 0; i < 4 ; i++){
        myserial_control_out.dist_ll_bottom += (results_bottom.nb_target_detected[i*4 + 3] > 0) ? results_bottom.distance_mm[i*4 + 3] : 0;
        myserial_control_out.dist_ml_bottom += (results_bottom.nb_target_detected[i*4 + 2] > 0) ? results_bottom.distance_mm[i*4 + 2] : 0;
        myserial_control_out.dist_mr_bottom += (results_bottom.nb_target_detected[i*4 + 1] > 0) ? results_bottom.distance_mm[i*4 + 1] : 0;
        myserial_control_out.dist_rr_bottom += (results_bottom.nb_target_detected[i*4 + 0] > 0) ? results_bottom.distance_mm[i*4 + 0] : 0;
    }
    // divide by the number of targets detected in the 4 zones
    myserial_control_out.dist_ll_bottom /= (results_bottom.nb_target_detected[3] + results_bottom.nb_target_detected[7] + results_bottom.nb_target_detected[11] + results_bottom.nb_target_detected[15]);
    myserial_control_out.dist_ml_bottom /= (results_bottom.nb_target_detected[2] + results_bottom.nb_target_detected[6] + results_bottom.nb_target_detected[10] + results_bottom.nb_target_detected[14]);
    myserial_control_out.dist_mr_bottom /= (results_bottom.nb_target_detected[1] + results_bottom.nb_target_detected[5] + results_bottom.nb_target_detected[9] + results_bottom.nb_target_detected[13]);
    myserial_control_out.dist_rr_bottom /= (results_bottom.nb_target_detected[0] + results_bottom.nb_target_detected[4] + results_bottom.nb_target_detected[8] + results_bottom.nb_target_detected[12]);

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