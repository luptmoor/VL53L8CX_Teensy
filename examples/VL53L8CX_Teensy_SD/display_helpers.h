#ifndef DISPLAY_HELPERS_H
#define DISPLAY_HELPERS_H

#include <stdint.h>

void handle_cmd(uint8_t cmd);
void toggle_resolution(void);
void toggle_signal_and_ambient(void);

#endif // DISPLAY_HELPERS_H
