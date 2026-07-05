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

// FeliCa command codes (a response uses command code + 1).
enum FelicaCommand : uint8_t {
  CMD_POLLING = 0x00,
  CMD_REQUEST_SERVICE = 0x02,
  CMD_REQUEST_RESPONSE = 0x04,
  CMD_READ_WO_ENC = 0x06,
  CMD_WRITE_WO_ENC = 0x08,
  CMD_SEARCH_SERVICE_CODE = 0x0A,
  CMD_REQUEST_SYSTEM_CODE = 0x0C,
  CMD_AUTHENTICATION1 = 0x10, // unsupported
  CMD_ECHO = 0xF0,            // non-standard, used for testing
};

// Status flags returned to the reader.

// Status Flag 1.
static constexpr uint8_t SF1_NORMAL_COMPLETION = 0x00;
static constexpr uint8_t SF1_ERROR =
    0xFF; // error not associated with a list index

// Status Flag 2 (subset used by this implementation).
static constexpr uint8_t SF2_NORMAL_COMPLETION = 0x00;
static constexpr uint8_t SF2_SERVICE_OR_NODE_COUNT_OUT_OF_RANGE = 0xA1;
static constexpr uint8_t SF2_BLOCK_COUNT_OUT_OF_RANGE = 0xA2;
static constexpr uint8_t SF2_REFERENCED_NODE_DOES_NOT_EXIST = 0xA6;
static constexpr uint8_t SF2_BLOCK_NUMBER_OUT_OF_RANGE = 0xA8;

// Response frame layout (byte offsets into the response buffer).
static constexpr int RESP_LEN = 0;      // total length byte
static constexpr int RESP_CODE = 1;     // response code
static constexpr int RESP_IDM = 2;      // 8-byte IDm
static constexpr int RESP_PAYLOAD = 10; // command-specific payload start

// Length of a plain status response: LEN + CODE + IDm(8) + Status Flags(2).
static constexpr int STATUS_RESPONSE_LEN = 12;

// Application layer functions
void initialize();
packet_t process(packet_t command);

// Debug functions
void print_packet(packet_t packet);
