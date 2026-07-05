// Application layer implementation for SiliCa
// JIS X 6319-4 compatible card implementation
//
// Debug output is controlled by the DEBUG macro; see silica.h.

#include "application.h"
#include "entropy.h"
#include "physical.h"
#include <avr/eeprom.h>
#include <string.h>

// ============================================================================
// EEPROM Data Storage
// ============================================================================

static uint8_t idm[8];
static uint8_t pmm[8];
static uint8_t service_code[2 * SERVICE_MAX];
static uint8_t system_code[2 * SYSTEM_MAX];

static uint8_t EEMEM idm_eep[8];
static uint8_t EEMEM pmm_eep[8];
static uint8_t EEMEM service_code_eep[2 * SERVICE_MAX];
static uint8_t EEMEM system_code_eep[2 * SYSTEM_MAX];
static uint8_t EEMEM block_data_eep[16 * BLOCK_MAX];
static uint8_t EEMEM last_error_eep[16 * LAST_ERROR_SIZE];

// Response buffer
// Maximum size: Read response with 12 blocks = 13 + 16*12 = 205 bytes
static constexpr int RESPONSE_BUF_SIZE = 208;
static uint8_t response[RESPONSE_BUF_SIZE];

// ============================================================================
// Initialization
// ============================================================================

// "RANDOMID" in ASCII: 0x52, 0x41, 0x4E, 0x44, 0x4F, 0x4D, 0x49, 0x44
static const uint8_t RANDOM_ID_MARKER[8] = {'R', 'A', 'N', 'D',
                                            'O', 'M', 'I', 'D'};

// Check if IDm is "RANDOMID" marker
static bool is_random_id_marker(const uint8_t *data) {
  return memcmp(data, RANDOM_ID_MARKER, 8) == 0;
}

void initialize() {
  eeprom_read_block(idm, idm_eep, 8);
  eeprom_read_block(pmm, pmm_eep, 8);
  eeprom_read_block(service_code, service_code_eep, 2 * SERVICE_MAX);
  eeprom_read_block(system_code, system_code_eep, 2 * SYSTEM_MAX);

  // If IDm in EEPROM is "RANDOMID", generate random IDm
  if (is_random_id_marker(idm)) {
    generate_random_bytes(idm, 8);
    // IDm byte 0: upper nibble is system index (0), lower nibble is
    // manufacturer code Clear upper nibble to ensure system index 0
    idm[0] = (idm[0] & 0x0F);
  }
}

// ============================================================================
// Helper Functions
// ============================================================================

// Set error response with status flags
static void set_error_response(uint8_t status1, uint8_t status2) {
  response[RESP_LEN] = STATUS_RESPONSE_LEN;
  response[RESP_PAYLOAD] = status1;
  response[RESP_PAYLOAD + 1] = status2;
}

// Fast 16-bit little-endian read from byte array
static uint16_t read_le16(const uint8_t *ptr) {
  return ptr[0] | (static_cast<uint16_t>(ptr[1]) << 8);
}

// Parse block list and extract block numbers
// Returns size of block list on success, 0 on error
static int parse_block_list(int n, const uint8_t *block_list,
                            uint8_t *block_nums) {
  int j = 0;
  for (int i = 0; i < n; i++) {
    const uint8_t header = block_list[j];
    if (header == 0x80) {
      // 2-byte block list element (most common case first)
      block_nums[i] = block_list[j + 1];
      j += 2;
    } else if (header == 0x00) {
      // 3-byte block list element
      if (block_list[j + 2] != 0x00)
        return 0;
      block_nums[i] = block_list[j + 1];
      j += 3;
    } else {
      return 0;
    }
  }
  return j;
}

// Find system index by system code (0xFF matches any).
// An all-zero entry marks the end of the populated list.
// Returns -1 if not found.
static int find_system_index(uint8_t sc1, uint8_t sc2) {
  for (int i = 0; i < SYSTEM_MAX; i++) {
    const uint8_t sys1 = system_code[2 * i];
    const uint8_t sys2 = system_code[2 * i + 1];
    if (sys1 == 0 && sys2 == 0)
      return -1;
    if ((sc1 == sys1 || sc1 == 0xFF) && (sc2 == sys2 || sc2 == 0xFF))
      return i;
  }
  return -1;
}

// Check if a service code is present (0xFFFF matches any).
// An all-zero entry marks the end of the populated list.
static bool find_service_code(uint16_t target_service_code) {
  if (target_service_code == 0xFFFF)
    return true;

  for (int i = 0; i < SERVICE_MAX; i++) {
    const uint16_t sc = read_le16(service_code + 2 * i);
    if (sc == 0)
      return false;
    if (target_service_code == sc)
      return true;
  }
  return false;
}

// ============================================================================
// Command Handlers
// ============================================================================

static bool polling(packet_t command) {
  const uint8_t cmd2 = command[2];
  const uint8_t cmd3 = command[3];

  int system_index = find_system_index(cmd2, cmd3);

  // Avoid bricking cards - respond to wildcard polling
  if (cmd2 == 0xFF && cmd3 == 0xFF)
    system_index = 0;

  if (system_index == -1)
    return false;

  const uint8_t request_code = command[4];
  if (request_code > 0x02)
    return false;

  // Response = LEN + CODE + IDm(8) + PMm(8), plus 2 bytes when requested.
  response[RESP_LEN] =
      (request_code == 0x00) ? (RESP_PAYLOAD + 8) : (RESP_PAYLOAD + 10);
  response[RESP_CODE] = 0x01; // Polling response code

  // Copy IDm and PMm - use memcpy (compiler optimizes for known size)
  memcpy(response + RESP_IDM, idm, 8);
  memcpy(response + RESP_PAYLOAD, pmm, 8);

  // Update top nibble of IDm with system index
  if (system_index > 0)
    response[RESP_IDM] = (system_index << 4) | (response[RESP_IDM] & 0x0F);

  // System code request
  if (request_code == 0x01) {
    const int offset = 2 * system_index;
    response[RESP_PAYLOAD + 8] = system_code[offset];
    response[RESP_PAYLOAD + 9] = system_code[offset + 1];
  }
  // Communication performance request
  else if (request_code == 0x02) {
    response[RESP_PAYLOAD + 8] = 0x00; // Reserved
    response[RESP_PAYLOAD + 9] = 0x01; // Only 212kbps supported
  }

  return true;
}

static bool request_service(packet_t command) {
  if (command[0] < 11)
    return false;

  const int n = command[10]; // Number of nodes
  if (!(1 <= n && n <= 32))
    return false;

  response[RESP_LEN] = RESP_PAYLOAD + 1 + 2 * n;
  response[RESP_PAYLOAD] = n;

  // Return key version 0 for all nodes
  memset(response + RESP_PAYLOAD + 1, 0x00, 2 * n);

  return true;
}

// Read a special (metadata) block into a 16-byte destination.
// These pseudo-blocks are only accessible in single-block reads.
// Returns true if block_num is a recognized special block.
static bool read_special_block(int block_num, uint8_t *dest) {
  switch (block_num) {
  case BLOCK_D_ID: // IDm + PMm
    memcpy(dest, idm, 8);
    memcpy(dest + 8, pmm, 8);
    return true;

  case BLOCK_SER_C: // Service codes
    memcpy(dest, service_code, 2 * SERVICE_MAX);
    memset(dest + 2 * SERVICE_MAX, 0x00, 16 - 2 * SERVICE_MAX);
    return true;

  case BLOCK_SYS_C: // System codes
    memcpy(dest, system_code, 2 * SYSTEM_MAX);
    memset(dest + 2 * SYSTEM_MAX, 0x00, 16 - 2 * SYSTEM_MAX);
    return true;
  }
  return false;
}

// Write a special (metadata) block from a 16-byte source and persist it.
// These pseudo-blocks are only accessible in single-block writes.
// Returns true if block_num is a recognized special block.
static bool write_special_block(int block_num, const uint8_t *src) {
  switch (block_num) {
  case BLOCK_D_ID: // IDm + PMm
    memcpy(idm, src, 8);
    eeprom_update_block(idm, idm_eep, 8);
    memcpy(pmm, src + 8, 8);
    eeprom_update_block(pmm, pmm_eep, 8);
    return true;

  case BLOCK_SER_C: // Service codes
    memcpy(service_code, src, 2 * SERVICE_MAX);
    eeprom_update_block(service_code, service_code_eep, 2 * SERVICE_MAX);
    return true;

  case BLOCK_SYS_C: // System codes
    memcpy(system_code, src, 2 * SYSTEM_MAX);
    eeprom_update_block(system_code, system_code_eep, 2 * SYSTEM_MAX);
    return true;
  }
  return false;
}

static bool read_without_encryption(packet_t command) {
  if (command[0] < 16)
    return false;

  const int m = command[10]; // Number of services
  if (m != 1) {
    set_error_response(SF1_ERROR, SF2_SERVICE_OR_NODE_COUNT_OUT_OF_RANGE);
    return true;
  }

  const uint16_t target_service_code = read_le16(command + 11);
  const int n = command[13]; // Number of blocks

  if (!find_service_code(target_service_code)) {
    set_error_response(SF1_ERROR, SF2_REFERENCED_NODE_DOES_NOT_EXIST);
    return true;
  }

  if (!(1 <= n && n <= BLOCK_MAX)) {
    set_error_response(SF1_ERROR, SF2_BLOCK_COUNT_OUT_OF_RANGE);
    return true;
  }

  uint8_t block_nums[BLOCK_MAX];
  if (parse_block_list(n, command + 14, block_nums) == 0) {
    set_error_response(SF1_ERROR, SF2_REFERENCED_NODE_DOES_NOT_EXIST);
    return true;
  }

  // Pre-check for single block special cases
  const bool single_block = (n == 1);

  // Load block data from EEPROM
  for (int i = 0; i < n; i++) {
    const int block_num = block_nums[i];
    uint8_t *const dest = response + RESP_PAYLOAD + 3 + 16 * i;

    // Normal data blocks (most common case first)
    if (block_num < BLOCK_MAX) {
      eeprom_read_block(dest, block_data_eep + 16 * block_num, 16);
      continue;
    }

    // Error log blocks
    if (block_num >= BLOCK_ERROR && block_num < BLOCK_ERROR + LAST_ERROR_SIZE) {
      eeprom_read_block(dest, last_error_eep + (block_num - BLOCK_ERROR) * 16,
                        16);
      continue;
    }

    // Special (metadata) blocks - single-block reads only
    if (single_block && read_special_block(block_num, dest))
      continue;

    // Invalid block
    set_error_response(SF1_ERROR, SF2_BLOCK_NUMBER_OUT_OF_RANGE);
    return true;
  }

  response[RESP_LEN] = RESP_PAYLOAD + 3 + 16 * n;
  response[RESP_PAYLOAD] = SF1_NORMAL_COMPLETION;
  response[RESP_PAYLOAD + 1] = SF2_NORMAL_COMPLETION;
  response[RESP_PAYLOAD + 2] = n; // Number of blocks returned

  return true;
}

static bool write_without_encryption(packet_t command) {
  const int len = command[0];
  const int m = command[10]; // Number of services
  const int n = command[13]; // Number of blocks

  if (len < 32)
    return false;

  if (m != 1) {
    set_error_response(SF1_ERROR, SF2_SERVICE_OR_NODE_COUNT_OUT_OF_RANGE);
    return true;
  }

  if (!(1 <= n && n <= BLOCK_MAX)) {
    set_error_response(SF1_ERROR, SF2_BLOCK_COUNT_OUT_OF_RANGE);
    return true;
  }

  uint8_t block_nums[BLOCK_MAX];
  const int block_list_len = parse_block_list(n, command + 14, block_nums);

  if (block_list_len == 0) {
    set_error_response(SF1_ERROR, SF2_REFERENCED_NODE_DOES_NOT_EXIST);
    return true;
  }

  if (len != 14 + block_list_len + 16 * n)
    return false;

  // Pre-check for single block special cases
  const bool single_block = (n == 1);
  const uint8_t *const data_start = command + 14 + block_list_len;

  // Write block data to EEPROM
  for (int i = 0; i < n; i++) {
    const int block_num = block_nums[i];
    const uint8_t *const src = data_start + 16 * i;

    // Normal data blocks (most common case first)
    if (block_num < BLOCK_MAX) {
      eeprom_update_block(src, block_data_eep + 16 * block_num, 16);
      continue;
    }

    // Special (metadata) blocks - single-block writes only
    if (single_block && write_special_block(block_num, src))
      continue;

    // Invalid block
    set_error_response(SF1_ERROR, SF2_BLOCK_NUMBER_OUT_OF_RANGE);
    return true;
  }

  response[RESP_LEN] = STATUS_RESPONSE_LEN;
  response[RESP_PAYLOAD] = SF1_NORMAL_COMPLETION;
  response[RESP_PAYLOAD + 1] = SF2_NORMAL_COMPLETION;

  return true;
}

static bool search_service_code(int index) {
  response[RESP_LEN] = STATUS_RESPONSE_LEN;

  if (index < 0 || index >= SERVICE_MAX) {
    response[RESP_PAYLOAD] = 0xFF;
    response[RESP_PAYLOAD + 1] = 0xFF;
    return true;
  }

  const int offset = 2 * index;
  const uint8_t sc1 = service_code[offset];
  const uint8_t sc2 = service_code[offset + 1];

  if (sc1 == 0x00 && sc2 == 0x00) {
    response[RESP_PAYLOAD] = 0xFF;
    response[RESP_PAYLOAD + 1] = 0xFF;
  } else {
    response[RESP_PAYLOAD] = sc1;
    response[RESP_PAYLOAD + 1] = sc2;
  }

  return true;
}

static bool request_system_code() {
  // Emit each populated system code; an all-zero entry ends the list.
  int n = 0;
  for (int i = 0; i < SYSTEM_MAX; i++) {
    const uint8_t sc1 = system_code[2 * i];
    const uint8_t sc2 = system_code[2 * i + 1];
    if (sc1 == 0x00 && sc2 == 0x00)
      break;
    response[RESP_PAYLOAD + 1 + 2 * n] = sc1;
    response[RESP_PAYLOAD + 2 + 2 * n] = sc2;
    n++;
  }

  response[RESP_LEN] = RESP_PAYLOAD + 1 + 2 * n;
  response[RESP_PAYLOAD] = n;
  return n != 0;
}

// ============================================================================
// Error Handling
// ============================================================================

void save_error(packet_t command) {
  int len = command[0];
  if (len > static_cast<int>(sizeof(last_error_eep)))
    len = sizeof(last_error_eep);

  eeprom_update_block(command, last_error_eep, len);
}

// ============================================================================
// Main Command Processor
// ============================================================================

packet_t process(packet_t command) {
  if (command == nullptr)
    return nullptr;

  const int len = command[0];
  const uint8_t command_code = command[1];

  // Polling command (no IDm verification)
  if (command_code == CMD_POLLING)
    return polling(command) ? response : nullptr;

  // Echo command (for testing)
  if (command_code == CMD_ECHO && command[2] == 0x00) {
    memcpy(response, command, len);
    return response;
  }

  // Verify IDm matches - optimized order (nibble check first is faster)
  if ((command[2] & 0x0F) != (idm[0] & 0x0F))
    return nullptr;
  if (memcmp(command + 3, idm + 1, 7) != 0)
    return nullptr;

  // Command code must be even
  if (command_code & 0x01)
    return nullptr;

  // Set response code and copy IDm
  response[RESP_CODE] = command_code + 1;
  memcpy(response + RESP_IDM, command + 2, 8);

  // Use computed goto or switch - switch is usually optimized well by compiler
  switch (command_code) {
  case CMD_REQUEST_SERVICE:
    if (!request_service(command))
      return nullptr;
    break;

  case CMD_REQUEST_RESPONSE:
    if (len != 10)
      return nullptr;
    response[RESP_LEN] = RESP_PAYLOAD + 1;
    response[RESP_PAYLOAD] = 0x00; // Mode
    break;

  case CMD_READ_WO_ENC:
    if (!read_without_encryption(command))
      return nullptr;
    if (response[RESP_PAYLOAD] != SF1_NORMAL_COMPLETION) {
      save_error(command);
#ifdef DEBUG
      Serial_println("Read failed");
      print_packet(command);
#endif
    }
    break;

  case CMD_WRITE_WO_ENC:
    if (!write_without_encryption(command))
      return nullptr;
    break;

  case CMD_SEARCH_SERVICE_CODE: {
    if (len != 12)
      return nullptr;
    const int index = read_le16(command + 10);
    search_service_code(index);
    break;
  }

  case CMD_REQUEST_SYSTEM_CODE:
    if (len != 10)
      return nullptr;
    if (!request_system_code())
      return nullptr;
    break;

  case CMD_AUTHENTICATION1: // unsupported
  default:
    return nullptr;
  }

  return response;
}

// ============================================================================
// Debug Functions
// ============================================================================

#ifdef DEBUG
// Fast hex character conversion using lookup table
static const char hex_table[] = "0123456789ABCDEF";

void print_packet(packet_t packet) {
  int len = packet[0];
  if (len == 0) {
    Serial_println("<empty>");
    return;
  }

  for (int i = 1; i < len; i++) {
    const uint8_t byte = packet[i];
    Serial_write(hex_table[byte >> 4]);
    Serial_write(hex_table[byte & 0x0F]);
    if (i != len - 1)
      Serial_write(' ');
  }
  Serial_println("");
}
#else
// No-op in release build
void print_packet(packet_t) {}
#endif
