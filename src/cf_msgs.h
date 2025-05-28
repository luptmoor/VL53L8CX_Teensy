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
    //torque commands
    int16_t torque_x; //torque x
    int16_t torque_y; //torque y
    int16_t torque_z; //torque z
    int16_t x_integ;
    int16_t y_integ;
    //CHECKSUM
    uint8_t checksum_out;
};