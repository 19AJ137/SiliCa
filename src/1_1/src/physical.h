// Physical and data link layer for SiliCa
// JIS X 6319-4 compatible card implementation

#pragma once
#include "silica.h"

// System initialization
void setup();

// Main loop function
void loop();

// Receive command packet from the reader
// Returns null on error
packet_t receive_command();

// Send response packet to the reader
// Null response means no response
void send_response(packet_t response);
