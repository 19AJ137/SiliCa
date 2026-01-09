// Application layer implementation for SiliCa
// JIS X 6319-4 compatible card implementation

#include <stdio.h>
#include <string.h>
#include <avr/eeprom.h>
#include "application.h"

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
static uint8_t response[0xFF] = {};

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
// Helper Functions
// ============================================================================

// Set error response with status flags
static void set_error_response(uint8_t status1, uint8_t status2)
{
    response[0] = 12;
    response[10] = status1;
    response[11] = status2;
}

// Parse block list and extract block numbers
// Returns size of block list on success, 0 on error
static int parse_block_list(int n, const uint8_t *block_list, uint8_t *block_nums)
{
    int j = 0;
    for (int i = 0; i < n; i++)
    {
        if (block_list[j] == 0x80)
        {
            // 2-byte block list element
            block_nums[i] = block_list[j + 1];
            j += 2;
        }
        else if (block_list[j] == 0x00)
        {
            // 3-byte block list element
            if (block_list[j + 2] != 0x00)
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

// Find system index by system code
// Returns -1 if not found
static int find_system_index(uint8_t sc1, uint8_t sc2)
{
    for (int i = 0; i < SYSTEM_MAX; i++)
    {
        uint8_t sys1 = system_code[2 * i];
        uint8_t sys2 = system_code[2 * i + 1];

        if (sys1 == 0 && sys2 == 0)
            break;

        if ((sc1 == sys1 || sc1 == 0xFF) && (sc2 == sys2 || sc2 == 0xFF))
            return i;
    }
    return -1;
}

// Check if service code exists
static bool find_service_code(uint16_t target_service_code)
{
    if (target_service_code == 0xFFFF)
        return true;

    for (int i = 0; i < SERVICE_MAX; i++)
    {
        uint16_t sc = service_code[2 * i] | (service_code[2 * i + 1] << 8);
        if (sc == 0)
            break;
        if (target_service_code == sc)
            return true;
    }
    return false;
}

// ============================================================================
// Command Handlers
// ============================================================================

static bool polling(packet_t command)
{
    int system_index = find_system_index(command[2], command[3]);

    // Avoid bricking cards - respond to wildcard polling
    if (command[2] == 0xFF && command[3] == 0xFF)
        system_index = 0;

    if (system_index == -1)
        return false;

    uint8_t request_code = command[4];
    if (request_code > 0x02)
        return false;

    // Set response length based on request code
    response[0] = (request_code == 0x00) ? 18 : 20;
    response[1] = 0x01; // Response code

    // Copy IDm and PMm
    memcpy(response + 2, idm, 8);
    memcpy(response + 10, pmm, 8);

    // Update top nibble of IDm with system index
    if (system_index > 0)
        response[2] = (system_index << 4) | (response[2] & 0x0F);

    // System code request
    if (request_code == 0x01)
        memcpy(response + 18, system_code + 2 * system_index, 2);

    // Communication performance request
    if (request_code == 0x02)
    {
        response[18] = 0x00; // Reserved
        response[19] = 0x01; // Only 212kbps supported
    }

    return true;
}

static bool request_service(packet_t command)
{
    if (command[0] < 11)
        return false;

    int n = command[10]; // Number of nodes
    if (!(1 <= n && n <= 32))
        return false;

    response[0] = 11 + 2 * n;
    response[10] = n;

    // Return key version 0 for all nodes
    for (int i = 0; i < n; i++)
    {
        response[11 + 2 * i] = 0x00;
        response[12 + 2 * i] = 0x00;
    }

    return true;
}

static bool read_without_encryption(packet_t command)
{
    if (command[0] < 16)
        return false;

    int m = command[10]; // Number of services
    if (m != 1)
    {
        set_error_response(0xFF, 0xA1);
        return true;
    }

    uint16_t target_service_code = command[11] | (command[12] << 8);
    int n = command[13]; // Number of blocks

    if (!find_service_code(target_service_code))
    {
        set_error_response(0xFF, 0xA6);
        return true;
    }

    if (!(1 <= n && n <= BLOCK_MAX))
    {
        set_error_response(0xFF, 0xA2);
        return true;
    }

    uint8_t block_nums[BLOCK_MAX];
    if (parse_block_list(n, command + 14, block_nums) == 0)
    {
        set_error_response(0xFF, 0xA6);
        return true;
    }

    // Load block data from EEPROM
    for (int i = 0; i < n; i++)
    {
        int block_num = block_nums[i];
        bool valid_block = false;

        // Normal data blocks
        if (block_num < BLOCK_MAX)
        {
            valid_block = true;
            eeprom_read_block(response + 13 + 16 * i, block_data_eep + 16 * block_num, 16);
        }

        // Error log blocks
        if (BLOCK_ERROR <= block_num && block_num < BLOCK_ERROR + LAST_ERROR_SIZE)
        {
            valid_block = true;
            eeprom_read_block(response + 13 + 16 * i, last_error_eep + (block_num - BLOCK_ERROR) * 16, 16);
        }

        // D_ID block (IDm + PMm)
        if (n == 1 && block_num == BLOCK_D_ID)
        {
            valid_block = true;
            memcpy(response + 13, idm, 8);
            memcpy(response + 21, pmm, 8);
        }

        // SER_C block (Service codes)
        if (n == 1 && block_num == BLOCK_SER_C)
        {
            valid_block = true;
            memcpy(response + 13, service_code, 2 * SERVICE_MAX);
            memset(response + 13 + 2 * SERVICE_MAX, 0x00, 16 - 2 * SERVICE_MAX);
        }

        // SYS_C block (System codes)
        if (n == 1 && block_num == BLOCK_SYS_C)
        {
            valid_block = true;
            memcpy(response + 13, system_code, 2 * SYSTEM_MAX);
            memset(response + 13 + 2 * SYSTEM_MAX, 0x00, 16 - 2 * SYSTEM_MAX);
        }

        if (!valid_block)
        {
            set_error_response(0xFF, 0xA8);
            return true;
        }
    }

    response[0] = 13 + 16 * n;
    response[10] = 0x00;
    response[11] = 0x00;
    response[12] = n;

    return true;
}

static bool write_without_encryption(packet_t command)
{
    int len = command[0];
    int m = command[10]; // Number of services
    uint16_t target_service_code = command[11] | (command[12] << 8);
    int n = command[13]; // Number of blocks

    if (len < 32)
        return false;

    if (m != 1)
    {
        set_error_response(0xFF, 0xA1);
        return true;
    }

    if (!(1 <= n && n <= BLOCK_MAX))
    {
        set_error_response(0xFF, 0xA2);
        return true;
    }

    uint8_t block_nums[BLOCK_MAX];
    int N = parse_block_list(n, command + 14, block_nums);

    if (N == 0)
    {
        set_error_response(0xFF, 0xA6);
        return true;
    }

    if (len != 14 + N + 16 * n)
        return false;

    // Write block data to EEPROM
    for (int i = 0; i < n; i++)
    {
        int block_num = block_nums[i];
        bool valid_block = false;

        // Normal data blocks
        if (block_num < BLOCK_MAX)
        {
            valid_block = true;
            eeprom_update_block(command + 14 + N + 16 * i, block_data_eep + 16 * block_num, 16);
        }

        // D_ID block (IDm + PMm)
        if (n == 1 && block_num == BLOCK_D_ID)
        {
            valid_block = true;
            memcpy(idm, command + 16, 8);
            eeprom_update_block(idm, idm_eep, 8);
            memcpy(pmm, command + 24, 8);
            eeprom_update_block(pmm, pmm_eep, 8);
        }

        // SER_C block (Service codes)
        if (n == 1 && block_num == BLOCK_SER_C)
        {
            valid_block = true;
            memcpy(service_code, command + 16, 2 * SERVICE_MAX);
            eeprom_update_block(service_code, service_code_eep, 2 * SERVICE_MAX);
        }

        // SYS_C block (System codes)
        if (n == 1 && block_num == BLOCK_SYS_C)
        {
            valid_block = true;
            memcpy(system_code, command + 16, 2 * SYSTEM_MAX);
            eeprom_update_block(system_code, system_code_eep, 2 * SYSTEM_MAX);
        }

        if (!valid_block)
        {
            set_error_response(0xFF, 0xA8);
            return true;
        }
    }

    response[0] = 12;
    response[10] = 0x00;
    response[11] = 0x00;

    return true;
}

static bool search_service_code(int index)
{
    response[0] = 12;

    if (index < 0 || index >= SERVICE_MAX)
    {
        response[10] = 0xFF;
        response[11] = 0xFF;
        return true;
    }

    uint8_t sc1 = service_code[2 * index];
    uint8_t sc2 = service_code[2 * index + 1];

    if (sc1 == 0x00 && sc2 == 0x00)
    {
        response[10] = 0xFF;
        response[11] = 0xFF;
        return true;
    }

    response[10] = sc1;
    response[11] = sc2;

    return true;
}

static bool request_system_code()
{
    int n = 0;

    for (int i = 0; i < SYSTEM_MAX; i++)
    {
        uint8_t sc1 = system_code[2 * i];
        uint8_t sc2 = system_code[2 * i + 1];

        if (sc1 == 0x00 && sc2 == 0x00)
            break;

        response[11 + 2 * i] = sc1;
        response[12 + 2 * i] = sc2;
        n++;
    }

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
    if (len > sizeof(last_error_eep))
        len = sizeof(last_error_eep);

    eeprom_update_block(command, last_error_eep, len);
}

// ============================================================================
// Main Command Processor
// ============================================================================

packet_t process(packet_t command)
{
    if (command == nullptr)
        return nullptr;

    const int len = command[0];
    const uint8_t command_code = command[1];

    // Polling command (no IDm verification)
    if (command_code == 0x00)
        return polling(command) ? response : nullptr;

    // Echo command (for testing)
    if (command[1] == 0xF0 && command[2] == 0x00)
    {
        memcpy(response, command, len);
        return response;
    }

    // Verify IDm matches
    if ((command[2] & 0x0F) != (idm[0] & 0x0F))
        return nullptr;
    if (memcmp(command + 3, idm + 1, 7) != 0)
        return nullptr;

    // Command code must be even
    if (command_code % 2 != 0)
        return nullptr;

    // Set response code and copy IDm
    response[1] = command_code + 1;
    memcpy(response + 2, command + 2, 8);

    switch (command_code)
    {
    case 0x02: // Request Service
        if (!request_service(command))
            return nullptr;
        break;

    case 0x04: // Request Response
        if (len != 10)
            return nullptr;
        response[0] = 11;
        response[10] = 0x00;
        break;

    case 0x06: // Read Without Encryption
        if (!read_without_encryption(command))
            return nullptr;
        if (response[10] != 0x00)
        {
            save_error(command);
            Serial_println("Read failed");
            print_packet(command);
        }
        break;

    case 0x08: // Write Without Encryption
        if (!write_without_encryption(command))
            return nullptr;
        break;

    case 0x0A: // Search Service Code
    {
        if (len != 12)
            return nullptr;
        int index = command[10] | (command[11] << 8);
        search_service_code(index);
        break;
    }

    case 0x0C: // Request System Code
        if (len != 10)
            return nullptr;
        if (!request_system_code())
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

void print_packet(packet_t packet)
{
    int len = packet[0];
    if (len == 0)
    {
        Serial_println("<empty>");
        return;
    }

    for (int i = 1; i < len; i++)
    {
        char hex_str[5];
        sprintf(hex_str, "%02X", packet[i]);
        Serial_print(hex_str);
        if (i != len - 1)
            Serial_print(" ");
    }
    Serial_println("");
}
