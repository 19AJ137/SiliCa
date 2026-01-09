// Application layer for SiliCa
// JIS X 6319-4 compatible card implementation

#pragma once
#include "silica.h"

// Configuration constants
static constexpr int BLOCK_MAX = 12;
static constexpr int SYSTEM_MAX = 4;
static constexpr int SERVICE_MAX = 4;
static constexpr int LAST_ERROR_SIZE = 2;

// Special block numbers
static constexpr int BLOCK_ERROR = 0xE0;
static constexpr int BLOCK_D_ID = 0x83;
static constexpr int BLOCK_SER_C = 0x84;
static constexpr int BLOCK_SYS_C = 0x85;

// Application layer functions
void initialize();
packet_t process(packet_t command);

// Debug functions
void print_packet(packet_t packet);
