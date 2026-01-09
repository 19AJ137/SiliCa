// Common definitions for SiliCa
// JIS X 6319-4 compatible card implementation

#pragma once
#include <stdint.h>

// Application layer packet type.
// The first element indicates the total length of the packet.
typedef const uint8_t *packet_t;

// Functions for serial output
// Similar to Arduino interface
void Serial_write(uint8_t data);
void Serial_print(const char *str);
void Serial_println(const char *str);

void save_error(packet_t);
