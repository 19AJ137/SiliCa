// Implementation of the application layer for
// JIS X 6319-4 compatible card "SiliCa"

#include <stdio.h>
#include <string.h>
#include <avr/eeprom.h>
#include "silica.h"

static uint8_t idm[8];
static uint8_t pmm[8];
static uint8_t sys[2];

static uint8_t EEMEM idm_eep[8];
static uint8_t EEMEM pmm_eep[8];
static uint8_t EEMEM sys_eep[2];

static uint16_t service_code;
static uint16_t EEMEM service_code_eep;

static constexpr int BLOCK_MAX = 12;
static uint8_t EEMEM block_data_eep[16 * BLOCK_MAX];

static uint8_t response[0xFF] = {};

void initialize()
{
    // read IDm, PMm, System Code from EEPROM
    eeprom_busy_wait();
    eeprom_read_block(idm, idm_eep, 8);
    eeprom_read_block(pmm, pmm_eep, 8);
    eeprom_read_block(sys, sys_eep, 2);

    service_code = eeprom_read_word(&service_code_eep);
}

bool polling(packet_t command)
{
    // check target system code
    if (!((command[2] == sys[0] || command[2] == 0xFF) && (command[3] == sys[1] || command[3] == 0xFF)))
        return false;

    // request code
    uint8_t req_code = command[4];
    if (req_code == 0x00)
        response[0] = 18;
    else
        response[0] = 20;

    if (req_code > 0x02)
        return false;

    // response code
    response[1] = 0x01;

    // time slot
    int n = command[5];

    memcpy(response + 2, idm, 8);
    memcpy(response + 10, pmm, 8);

    // system code request
    if (req_code == 0x01)
    {
        memcpy(response + 18, sys, 2);
    }
    // communication performance request
    if (req_code == 0x02)
    {
        response[18] = 0x00; // reserved
        response[19] = 0x01; // only 212kbps communication is supported
    }

    return true;
}

bool read_without_encryption(packet_t command)
{
    int m = command[10];

    if (m != 1)
    {
        response[0] = 12;    // length
        response[10] = 0xFF; // status flag 1
        response[11] = 0xA1; // status flag 2

        return true;
    }

    uint16_t target_service_code = command[11] | (command[12] << 8);

    if (target_service_code != service_code)
    {
        response[0] = 12;    // length
        response[10] = 0xFF; // status flag 1
        response[11] = 0xA6; // status flag 2

        return true;
    }

    int n = command[13];

    if (!(1 <= n && n <= BLOCK_MAX))
    {
        response[0] = 12;    // length
        response[10] = 0xFF; // status flag 1
        response[11] = 0xA2; // status flag 2

        return true;
    }

    for (int i = 0; i < n; i++)
    {
        int block_num = command[14 + i * 2 + 1];

        if (command[14 + i * 2] != 0x80 || block_num >= BLOCK_MAX)
        {
            response[0] = 12;    // length
            response[10] = 0xFF; // status flag 1
            response[11] = 0xA8; // status flag 2
            return true;
        }
    }

    response[0] = 13 + 16 * n; // length

    response[10] = 0x00; // status flag 1
    response[11] = 0x00; // status flag 2

    response[12] = n; // number of blocks

    for (int i = 0; i < n; i++)
    {
        int block_num = command[14 + i * 2 + 1];

        eeprom_busy_wait();
        eeprom_read_block(response + 13 + 16 * i, block_data_eep + 16 * block_num, 16);
    }

    return true;
}

bool write_without_encryption(packet_t command)
{
    int len = command[0];
    int m = command[10];
    uint16_t target_service_code = command[11] | (command[12] << 8);
    int n = command[13];

    if (m != 1)
    {
        response[0] = 12;    // length
        response[10] = 0xFF; // status flag 1
        response[11] = 0xA1; // status flag 2

        return true;
    }

    // system configuration mode
    if (target_service_code == 0x0000)
    {
        if (!(n == 1 && command[14] == 0x80 && command[15] == 0x00))
            return false;

        if (len < 32)
            return false;

        // Update IDm
        memcpy(idm, command + 16, 8);
        eeprom_busy_wait();
        eeprom_update_block(idm, idm_eep, 8);

        // Update PMm
        memcpy(pmm, command + 24, 8);
        eeprom_busy_wait();
        eeprom_update_block(pmm, pmm_eep, 8);

        // Update System Code if present
        if (len >= 34)
        {
            memcpy(sys, command + 32, 2);
            eeprom_busy_wait();
            eeprom_update_block(sys, sys_eep, 2);
        }

        response[0] = 12; // length

        response[10] = 0x00; // status flag 1
        response[11] = 0x00; // status flag 2

        return true;
    }

    if (!(1 <= n && n <= BLOCK_MAX))
    {
        response[0] = 12;    // length
        response[10] = 0xFF; // status flag 1
        response[11] = 0xA2; // status flag 2
        return true;
    }

    for (int i = 0; i < n; i++)
    {
        int block_num = command[14 + i * 2 + 1];

        if (command[14 + i * 2] != 0x80 || block_num >= BLOCK_MAX)
        {
            response[0] = 12;    // length
            response[10] = 0xFF; // status flag 1
            response[11] = 0xA8; // status flag 2
            return true;
        }
    }

    for (int i = 0; i < n; i++)
    {
        int block_num = command[14 + i * 2 + 1];

        eeprom_busy_wait();
        eeprom_update_block(command + 14 + n * 2 + 16 * i, block_data_eep + 16 * block_num, 16);
    }

    service_code = target_service_code;
    eeprom_busy_wait();
    eeprom_update_word(&service_code_eep, service_code);

    response[0] = 12; // length

    response[10] = 0x00; // status flag 1
    response[11] = 0x00; // status flag 2

    return true;
}

packet_t process(packet_t command)
{
    if (command == nullptr)
        return nullptr;

    const int len = command[0];
    const int command_code = command[1];

    // Polling
    if (command_code == 0x00)
    {
        if (polling(command))
            return response;
        else
            return nullptr;
    }

    // Echo
    if (command[1] == 0xF0 && command[2] == 0x00)
    {
        memcpy(response, command, len);
        return response;
    }

    // check IDm
    if (memcmp(command + 2, idm, 8) != 0)
        return nullptr;

    // check command code
    if (command_code % 2 != 0)
        return nullptr;

    // set response code
    response[1] = command_code + 1;

    // copy IDm
    memcpy(response + 2, idm, 8);

    switch (command_code)
    {
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
            Serial_print("Read failed: ");
            print_packet(command);
        }
        break;
    case 0x08: // Write Without Encryption
        if (!write_without_encryption(command))
            return nullptr;
        break;
    case 0x0A: // Search Service Code
        if (len != 12)
            return nullptr;

        response[0] = 12;
        if (command[10] == 0x00 && command[11] == 0x00)
        {
            response[10] = service_code & 0xFF;
            response[11] = service_code >> 8;
        }
        else
        {
            response[10] = 0xFF;
            response[11] = 0xFF;
        }
        break;
    case 0x0C: // Request System Code
        if (len != 10)
            return nullptr;

        response[0] = 13;

        response[10] = 1;
        response[11] = sys[0];
        response[12] = sys[1];

        break;
    case 0x02: // Request Service
    case 0x10: // Authentication1
    case 0x12: // Authentication2
    // pass through
    default:
        return nullptr;
    }

    return response;
}

void print_packet(packet_t packet)
{
    int len = packet[0];

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
