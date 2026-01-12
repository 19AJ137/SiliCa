// Application layer implementation for SiliCa
// JIS X 6319-4 compatible card implementation
// Optimized for speed

// Define DEBUG to enable debug output (comment out for release build)
// #define DEBUG

#include <string.h>
#include <avr/eeprom.h>
#include "application.h"

// ============================================================================
// Compiler Optimization Hints
// ============================================================================

#define FORCE_INLINE __attribute__((always_inline)) inline
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

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

void initialize()
{
    eeprom_read_block(idm, idm_eep, 8);
    eeprom_read_block(pmm, pmm_eep, 8);
    eeprom_read_block(service_code, service_code_eep, 2 * SERVICE_MAX);
    eeprom_read_block(system_code, system_code_eep, 2 * SYSTEM_MAX);
}

// ============================================================================
// Optimized Helper Functions
// ============================================================================

// Set error response with status flags - inlined for speed
static FORCE_INLINE void set_error_response(uint8_t status1, uint8_t status2)
{
    response[0] = 12;
    response[10] = status1;
    response[11] = status2;
}

// Fast 16-bit little-endian read from byte array
static FORCE_INLINE uint16_t read_le16(const uint8_t *ptr)
{
    return ptr[0] | (static_cast<uint16_t>(ptr[1]) << 8);
}

// Parse block list and extract block numbers
// Returns size of block list on success, 0 on error
// Optimized with register hints and early exit
static int parse_block_list(int n, const uint8_t *block_list, uint8_t *block_nums)
{
    register int j = 0;
    for (register int i = 0; i < n; i++)
    {
        const uint8_t header = block_list[j];
        if (header == 0x80)
        {
            // 2-byte block list element (most common case first)
            block_nums[i] = block_list[j + 1];
            j += 2;
        }
        else if (header == 0x00)
        {
            // 3-byte block list element
            if (UNLIKELY(block_list[j + 2] != 0x00))
                return 0;
            block_nums[i] = block_list[j + 1];
            j += 3;
        }
        else
        {
            return 0;
        }
    }
    return j;
}

// Find system index by system code - fully unrolled for SYSTEM_MAX=4
// Returns -1 if not found
static int find_system_index(uint8_t sc1, uint8_t sc2)
{
    // Unrolled loop for SYSTEM_MAX=4
    // Check index 0
    {
        const uint8_t sys1 = system_code[0];
        const uint8_t sys2 = system_code[1];
        if (sys1 == 0 && sys2 == 0)
            return -1;
        if ((sc1 == sys1 || sc1 == 0xFF) && (sc2 == sys2 || sc2 == 0xFF))
            return 0;
    }
    // Check index 1
    {
        const uint8_t sys1 = system_code[2];
        const uint8_t sys2 = system_code[3];
        if (sys1 == 0 && sys2 == 0)
            return -1;
        if ((sc1 == sys1 || sc1 == 0xFF) && (sc2 == sys2 || sc2 == 0xFF))
            return 1;
    }
    // Check index 2
    {
        const uint8_t sys1 = system_code[4];
        const uint8_t sys2 = system_code[5];
        if (sys1 == 0 && sys2 == 0)
            return -1;
        if ((sc1 == sys1 || sc1 == 0xFF) && (sc2 == sys2 || sc2 == 0xFF))
            return 2;
    }
    // Check index 3
    {
        const uint8_t sys1 = system_code[6];
        const uint8_t sys2 = system_code[7];
        if (sys1 == 0 && sys2 == 0)
            return -1;
        if ((sc1 == sys1 || sc1 == 0xFF) && (sc2 == sys2 || sc2 == 0xFF))
            return 3;
    }
    return -1;
}

// Check if service code exists - fully unrolled for SERVICE_MAX=4
// Uses 16-bit comparison for speed
static bool find_service_code(uint16_t target_service_code)
{
    if (target_service_code == 0xFFFF)
        return true;

    // Unrolled loop with 16-bit word comparison
    uint16_t sc;

    // Check index 0
    sc = read_le16(service_code);
    if (sc == 0)
        return false;
    if (target_service_code == sc)
        return true;

    // Check index 1
    sc = read_le16(service_code + 2);
    if (sc == 0)
        return false;
    if (target_service_code == sc)
        return true;

    // Check index 2
    sc = read_le16(service_code + 4);
    if (sc == 0)
        return false;
    if (target_service_code == sc)
        return true;

    // Check index 3
    sc = read_le16(service_code + 6);
    if (sc == 0)
        return false;
    if (target_service_code == sc)
        return true;

    return false;
}

// ============================================================================
// Command Handlers
// ============================================================================

static bool polling(packet_t command)
{
    const uint8_t cmd2 = command[2];
    const uint8_t cmd3 = command[3];

    int system_index = find_system_index(cmd2, cmd3);

    // Avoid bricking cards - respond to wildcard polling
    if (cmd2 == 0xFF && cmd3 == 0xFF)
        system_index = 0;

    if (UNLIKELY(system_index == -1))
        return false;

    const uint8_t request_code = command[4];
    if (UNLIKELY(request_code > 0x02))
        return false;

    // Set response length based on request code
    response[0] = (request_code == 0x00) ? 18 : 20;
    response[1] = 0x01; // Response code

    // Copy IDm and PMm - use memcpy (compiler optimizes for known size)
    memcpy(response + 2, idm, 8);
    memcpy(response + 10, pmm, 8);

    // Update top nibble of IDm with system index
    if (system_index > 0)
        response[2] = (system_index << 4) | (response[2] & 0x0F);

    // System code request
    if (request_code == 0x01)
    {
        const int offset = 2 * system_index;
        response[18] = system_code[offset];
        response[19] = system_code[offset + 1];
    }
    // Communication performance request
    else if (request_code == 0x02)
    {
        response[18] = 0x00; // Reserved
        response[19] = 0x01; // Only 212kbps supported
    }

    return true;
}

static bool request_service(packet_t command)
{
    if (UNLIKELY(command[0] < 11))
        return false;

    const int n = command[10]; // Number of nodes
    if (UNLIKELY(!(1 <= n && n <= 32)))
        return false;

    response[0] = 11 + 2 * n;
    response[10] = n;

    // Return key version 0 for all nodes - optimized with memset
    memset(response + 11, 0x00, 2 * n);

    return true;
}

static bool read_without_encryption(packet_t command)
{
    if (UNLIKELY(command[0] < 16))
        return false;

    const int m = command[10]; // Number of services
    if (UNLIKELY(m != 1))
    {
        set_error_response(0xFF, 0xA1);
        return true;
    }

    const uint16_t target_service_code = read_le16(command + 11);
    const int n = command[13]; // Number of blocks

    if (UNLIKELY(!find_service_code(target_service_code)))
    {
        set_error_response(0xFF, 0xA6);
        return true;
    }

    if (UNLIKELY(!(1 <= n && n <= BLOCK_MAX)))
    {
        set_error_response(0xFF, 0xA2);
        return true;
    }

    uint8_t block_nums[BLOCK_MAX];
    if (UNLIKELY(parse_block_list(n, command + 14, block_nums) == 0))
    {
        set_error_response(0xFF, 0xA6);
        return true;
    }

    // Pre-check for single block special cases
    const bool single_block = (n == 1);

    // Load block data from EEPROM
    for (register int i = 0; i < n; i++)
    {
        const int block_num = block_nums[i];
        uint8_t *const dest = response + 13 + 16 * i;

        // Normal data blocks (most common case first)
        if (LIKELY(block_num < BLOCK_MAX))
        {
            eeprom_read_block(dest, block_data_eep + 16 * block_num, 16);
            continue;
        }

        // Error log blocks
        if (block_num >= BLOCK_ERROR && block_num < BLOCK_ERROR + LAST_ERROR_SIZE)
        {
            eeprom_read_block(dest, last_error_eep + (block_num - BLOCK_ERROR) * 16, 16);
            continue;
        }

        // Single-block special cases only
        if (single_block)
        {
            switch (block_num)
            {
            case BLOCK_D_ID: // IDm + PMm
                memcpy(dest, idm, 8);
                memcpy(dest + 8, pmm, 8);
                continue;

            case BLOCK_SER_C: // Service codes
                memcpy(dest, service_code, 2 * SERVICE_MAX);
                memset(dest + 2 * SERVICE_MAX, 0x00, 16 - 2 * SERVICE_MAX);
                continue;

            case BLOCK_SYS_C: // System codes
                memcpy(dest, system_code, 2 * SYSTEM_MAX);
                memset(dest + 2 * SYSTEM_MAX, 0x00, 16 - 2 * SYSTEM_MAX);
                continue;
            }
        }

        // Invalid block
        set_error_response(0xFF, 0xA8);
        return true;
    }

    response[0] = 13 + 16 * n;
    response[10] = 0x00;
    response[11] = 0x00;
    response[12] = n;

    return true;
}

static bool write_without_encryption(packet_t command)
{
    const int len = command[0];
    const int m = command[10]; // Number of services
    const int n = command[13]; // Number of blocks

    if (UNLIKELY(len < 32))
        return false;

    if (UNLIKELY(m != 1))
    {
        set_error_response(0xFF, 0xA1);
        return true;
    }

    if (UNLIKELY(!(1 <= n && n <= BLOCK_MAX)))
    {
        set_error_response(0xFF, 0xA2);
        return true;
    }

    uint8_t block_nums[BLOCK_MAX];
    const int N = parse_block_list(n, command + 14, block_nums);

    if (UNLIKELY(N == 0))
    {
        set_error_response(0xFF, 0xA6);
        return true;
    }

    if (UNLIKELY(len != 14 + N + 16 * n))
        return false;

    // Pre-check for single block special cases
    const bool single_block = (n == 1);
    const uint8_t *const data_start = command + 14 + N;

    // Write block data to EEPROM
    for (register int i = 0; i < n; i++)
    {
        const int block_num = block_nums[i];
        const uint8_t *const src = data_start + 16 * i;

        // Normal data blocks (most common case first)
        if (LIKELY(block_num < BLOCK_MAX))
        {
            eeprom_update_block(src, block_data_eep + 16 * block_num, 16);
            continue;
        }

        // Single-block special cases only
        if (single_block)
        {
            switch (block_num)
            {
            case BLOCK_D_ID: // IDm + PMm
                memcpy(idm, src, 8);
                eeprom_update_block(idm, idm_eep, 8);
                memcpy(pmm, src + 8, 8);
                eeprom_update_block(pmm, pmm_eep, 8);
                continue;

            case BLOCK_SER_C: // Service codes
                memcpy(service_code, src, 2 * SERVICE_MAX);
                eeprom_update_block(service_code, service_code_eep, 2 * SERVICE_MAX);
                continue;

            case BLOCK_SYS_C: // System codes
                memcpy(system_code, src, 2 * SYSTEM_MAX);
                eeprom_update_block(system_code, system_code_eep, 2 * SYSTEM_MAX);
                continue;
            }
        }

        // Invalid block
        set_error_response(0xFF, 0xA8);
        return true;
    }

    response[0] = 12;
    response[10] = 0x00;
    response[11] = 0x00;

    return true;
}

static FORCE_INLINE bool search_service_code(int index)
{
    response[0] = 12;

    if (UNLIKELY(index < 0 || index >= SERVICE_MAX))
    {
        response[10] = 0xFF;
        response[11] = 0xFF;
        return true;
    }

    const int offset = 2 * index;
    const uint8_t sc1 = service_code[offset];
    const uint8_t sc2 = service_code[offset + 1];

    if (sc1 == 0x00 && sc2 == 0x00)
    {
        response[10] = 0xFF;
        response[11] = 0xFF;
    }
    else
    {
        response[10] = sc1;
        response[11] = sc2;
    }

    return true;
}

static bool request_system_code()
{
    int n = 0;

    // Unrolled loop for SYSTEM_MAX=4
    #define CHECK_SYSTEM(idx) \
        { \
            const uint8_t sc1 = system_code[2 * (idx)]; \
            const uint8_t sc2 = system_code[2 * (idx) + 1]; \
            if (sc1 == 0x00 && sc2 == 0x00) \
                goto done; \
            response[11 + 2 * n] = sc1; \
            response[12 + 2 * n] = sc2; \
            n++; \
        }

    CHECK_SYSTEM(0);
    CHECK_SYSTEM(1);
    CHECK_SYSTEM(2);
    CHECK_SYSTEM(3);

    #undef CHECK_SYSTEM

done:
    response[0] = 11 + 2 * n;
    response[10] = n;
    return n != 0;
}

// ============================================================================
// Error Handling
// ============================================================================

void save_error(packet_t command)
{
    int len = command[0];
    if (len > static_cast<int>(sizeof(last_error_eep)))
        len = sizeof(last_error_eep);

    eeprom_update_block(command, last_error_eep, len);
}

// ============================================================================
// Main Command Processor
// ============================================================================

packet_t process(packet_t command)
{
    if (UNLIKELY(command == nullptr))
        return nullptr;

    const int len = command[0];
    const uint8_t command_code = command[1];

    // Polling command (no IDm verification)
    if (command_code == 0x00)
        return polling(command) ? response : nullptr;

    // Echo command (for testing)
    if (command_code == 0xF0 && command[2] == 0x00)
    {
        memcpy(response, command, len);
        return response;
    }

    // Verify IDm matches - optimized order (nibble check first is faster)
    if (UNLIKELY((command[2] & 0x0F) != (idm[0] & 0x0F)))
        return nullptr;
    if (UNLIKELY(memcmp(command + 3, idm + 1, 7) != 0))
        return nullptr;

    // Command code must be even
    if (UNLIKELY(command_code & 0x01))
        return nullptr;

    // Set response code and copy IDm
    response[1] = command_code + 1;
    memcpy(response + 2, command + 2, 8);

    // Use computed goto or switch - switch is usually optimized well by compiler
    switch (command_code)
    {
    case 0x02: // Request Service
        if (UNLIKELY(!request_service(command)))
            return nullptr;
        break;

    case 0x04: // Request Response
        if (UNLIKELY(len != 10))
            return nullptr;
        response[0] = 11;
        response[10] = 0x00;
        break;

    case 0x06: // Read Without Encryption
        if (UNLIKELY(!read_without_encryption(command)))
            return nullptr;
        if (UNLIKELY(response[10] != 0x00))
        {
            save_error(command);
#ifdef DEBUG
            Serial_println("Read failed");
            print_packet(command);
#endif
        }
        break;

    case 0x08: // Write Without Encryption
        if (UNLIKELY(!write_without_encryption(command)))
            return nullptr;
        break;

    case 0x0A: // Search Service Code
    {
        if (UNLIKELY(len != 12))
            return nullptr;
        const int index = read_le16(command + 10);
        search_service_code(index);
        break;
    }

    case 0x0C: // Request System Code
        if (UNLIKELY(len != 10))
            return nullptr;
        if (UNLIKELY(!request_system_code()))
            return nullptr;
        break;

    case 0x10: // Authentication1 (unsupported)
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

void print_packet(packet_t packet)
{
    int len = packet[0];
    if (UNLIKELY(len == 0))
    {
        Serial_println("<empty>");
        return;
    }

    for (int i = 1; i < len; i++)
    {
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
