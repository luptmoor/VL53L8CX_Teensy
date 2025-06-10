struct __attribute__((__packed__)) serial_control_in {
    //thrust (used for resetting network)
    float thrust;
    //state
    float roll; //roll target
    float pitch; //pitch target
    float yaw; //yaw rate target
    // gyro values
    float roll_gyro;
    float pitch_gyro;
    float yaw_gyro;
    // accelerometer values
    float x_acc;
    float y_acc;
    float z_acc;
    //CHECKSUM
    uint8_t checksum_in;
};

struct __attribute__((__packed__)) serial_control_out {
    // measured avg distance left-to-right
    int16_t dist_ll_bottom;
    int16_t dist_ml_bottom;
    int16_t dist_mr_bottom;
    int16_t dist_rr_bottom;
    int16_t dist_ll_forward;
    int16_t dist_ml_forward;
    int16_t dist_mr_forward;
    int16_t dist_rr_forward;

    //CHECKSUM
    uint8_t checksum_out;
};