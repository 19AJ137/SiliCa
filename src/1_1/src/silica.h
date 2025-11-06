#pragma once
#include <stdint.h>

// Application layer packet type.
// The first element indicates the total length of the packet.
typedef const uint8_t *packet_t;

// Functions for serial output
// Similar to Arduino interface
void Serial_write(uint8_t);
void Serial_print(const char *);
void Serial_println(const char *);

// application layer functions
void initialize();
packet_t process(packet_t);
void save_error(packet_t);

// debug functions
void print_packet(packet_t);
