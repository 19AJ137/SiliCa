#pragma once
#include <stdint.h>

// Application layer packet type.
// The first element indicates the total length of the packet.
typedef const uint8_t *packet_t;

void Serial_write(uint8_t);
void Serial_print(const char *);
void Serial_println(const char *);

void initialize();
packet_t process(packet_t);
