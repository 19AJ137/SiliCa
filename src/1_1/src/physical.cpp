// Physical and data link layer implementation for SiliCa
// JIS X 6319-4 compatible card implementation

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <util/crc16.h>
#include <util/delay.h>
#include "physical.h"
#include "application.h"

// ============================================================================
// Constants
// ============================================================================

// Data link layer header (preamble + sync code)
static const uint8_t header[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x4D};

// Manchester encoding lookup table
static const uint8_t manchester_table[16] = {
    0x55, 0x56, 0x59, 0x5A, 0x65, 0x66, 0x69, 0x6A,
    0x95, 0x96, 0x99, 0x9A, 0xA5, 0xA6, 0xA9, 0xAA
};

// Bit extraction masks for each shift value
// Each entry contains 8 pairs of (byte_offset, bit_mask)
static const uint8_t bit_masks[8][8][2] = {
    // shift = 0
    {{0, 0x80}, {0, 0x20}, {0, 0x08}, {0, 0x02}, {1, 0x80}, {1, 0x20}, {1, 0x08}, {1, 0x02}},
    // shift = 1
    {{0, 0x40}, {0, 0x10}, {0, 0x04}, {0, 0x01}, {1, 0x40}, {1, 0x10}, {1, 0x04}, {1, 0x01}},
    // shift = 2
    {{0, 0x20}, {0, 0x08}, {0, 0x02}, {1, 0x80}, {1, 0x20}, {1, 0x08}, {1, 0x02}, {2, 0x80}},
    // shift = 3
    {{0, 0x10}, {0, 0x04}, {0, 0x01}, {1, 0x40}, {1, 0x10}, {1, 0x04}, {1, 0x01}, {2, 0x40}},
    // shift = 4
    {{0, 0x08}, {0, 0x02}, {1, 0x80}, {1, 0x20}, {1, 0x08}, {1, 0x02}, {2, 0x80}, {2, 0x20}},
    // shift = 5
    {{0, 0x04}, {0, 0x01}, {1, 0x40}, {1, 0x10}, {1, 0x04}, {1, 0x01}, {2, 0x40}, {2, 0x10}},
    // shift = 6
    {{0, 0x02}, {1, 0x80}, {1, 0x20}, {1, 0x08}, {1, 0x02}, {2, 0x80}, {2, 0x20}, {2, 0x08}},
    // shift = 7
    {{0, 0x01}, {1, 0x40}, {1, 0x10}, {1, 0x04}, {1, 0x01}, {2, 0x40}, {2, 0x10}, {2, 0x04}},
};

// ============================================================================
// Buffers
// ============================================================================

static uint8_t rx_buf[0x220] = {};
static uint8_t command[0x110] = {};

// ============================================================================
// Serial Output Functions
// ============================================================================

void Serial_write(uint8_t data)
{
    while (!(USART0.STATUS & USART_DREIF_bm))
    {
    }
    USART0.TXDATAL = data;
}

void Serial_print(const char *str)
{
    while (*str)
        Serial_write(*str++);
}

void Serial_println(const char *str)
{
    Serial_print(str);
    Serial_print("\r\n");
}

// ============================================================================
// SPI Functions
// ============================================================================

static uint8_t SPI_transfer(uint8_t data = 0)
{
    while (!(SPI0.INTFLAGS & SPI_DREIF_bm))
    {
    }
    SPI0.DATA = data;
    return SPI0.DATA;
}

// ============================================================================
// CRC Functions
// ============================================================================

static uint16_t crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = _crc_xmodem_update(crc, buf[i]);
    return crc;
}

// ============================================================================
// Frame Capture and Decoding
// ============================================================================

static int capture_frame()
{
    for (int i = 0; i < sizeof(rx_buf); i++)
    {
        uint8_t data = SPI_transfer();
        rx_buf[i] = data;

        // End of frame
        if (data == 0x00 || data == 0xFF)
        {
            // Frame too short
            if (i < sizeof(header) * 2)
            {
                i = -1;
                continue;
            }
            return i + 1;
        }
    }
    return 0; // Frame too long
}

static int get_shift_from_sync(uint8_t sync1, uint8_t sync2)
{
    uint8_t a1 = sync1 & 0xAA;
    uint8_t b1 = sync1 & 0x55;
    uint8_t a2 = sync2 & 0xAA;
    uint8_t b2 = sync2 & 0x55;

    if (a1 == 0x8A && a2 == 0x08) return 0;
    if (b1 == 0x45 && b2 == 0x04) return 1;
    if (a1 == 0x22 && a2 == 0x82) return 2;
    if (b1 == 0x11 && b2 == 0x41) return 3;
    if (a1 == 0x08 && a2 == 0xA0) return 4;
    if (b1 == 0x04 && b2 == 0x50) return 5;
    if (a1 == 0x02 && a2 == 0x28) return 6;
    if (b1 == 0x01 && b2 == 0x14) return 7;

    return -1;
}

static int find_sync_index(int rx_len, int &shift, bool &invert)
{
    for (int i = 0; i < rx_len - 1; i++)
    {
        int shift1 = get_shift_from_sync(rx_buf[i], rx_buf[i + 1]);
        int shift2 = get_shift_from_sync(~rx_buf[i], ~rx_buf[i + 1]);

        if (shift1 != -1 && shift1 > shift2)
        {
            shift = shift1;
            invert = false;
            return i;
        }
        if (shift2 != -1 && shift2 > shift1)
        {
            shift = shift2;
            invert = true;
            return i;
        }
    }
    return -1;
}

// Extract one byte from received data according to bit shift
// Optimized using lookup table
static uint8_t extract_byte(int shift, uint8_t data1, uint8_t data2, uint8_t data3)
{
    const uint8_t data[3] = {data1, data2, data3};
    uint8_t result = 0;

    for (int bit = 0; bit < 8; bit++)
    {
        uint8_t byte_idx = bit_masks[shift][bit][0];
        uint8_t mask = bit_masks[shift][bit][1];
        if (data[byte_idx] & mask)
            result |= (0x80 >> bit);
    }

    return result;
}

packet_t receive_command()
{
    int rx_len = capture_frame();
    if (rx_len == 0)
    {
        Serial_println("Frame capture error");
        return nullptr;
    }

    int shift = -1;
    bool invert;
    int rx_index = find_sync_index(rx_len, shift, invert);
    if (rx_index == -1)
    {
        Serial_println("Sync error");
        return nullptr;
    }

    // Skip sync pattern
    rx_index += 4;

    // Decode data
    int index = 0;
    for (int i = rx_index; i < rx_len - 2; i += 2)
    {
        uint8_t x = extract_byte(shift, rx_buf[i], rx_buf[i + 1], rx_buf[i + 2]);
        if (invert)
            x = ~x;
        command[index++] = x;
    }

    // Verify length
    int len = command[0];
    if (len + 2 > index)
    {
        Serial_println("Length error");
        return nullptr;
    }

    // Verify EDC (Error Detection Code)
    uint16_t calculated_edc = crc16(command, len);
    uint16_t received_edc = (command[len] << 8) | command[len + 1];

    // Allow last 1-bit error
    if ((calculated_edc ^ received_edc) > 1)
    {
        Serial_println("EDC error");
        return nullptr;
    }

    return command;
}

// ============================================================================
// Transmission Functions
// ============================================================================

static void enable_transmit(bool enable)
{
    SPI_transfer(0x00);
    SPI_transfer(0x00);

    if (enable)
        CCL.CTRLA = CCL_ENABLE_bm;
    else
        CCL.CTRLA = 0;
}

static void transmit_byte(uint8_t data)
{
    SPI_transfer(manchester_table[data >> 4]);
    SPI_transfer(manchester_table[data & 0xF]);
}

void send_response(packet_t response)
{
    if (response == nullptr)
        return;

    int len = response[0];
    uint16_t edc = crc16(response, len);

    enable_transmit(true);

    // Send header
    for (int i = 0; i < sizeof(header); i++)
        transmit_byte(header[i]);

    // Send body
    for (int i = 0; i < len; i++)
        transmit_byte(response[i]);

    // Send footer (EDC)
    transmit_byte(edc >> 8);
    transmit_byte(edc & 0xFF);

    enable_transmit(false);
}

// ============================================================================
// System Setup
// ============================================================================

void setup()
{
    // Configure system clock: set fclk to fc/4 (3.39MHz) using external clock
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_ENABLE_bm);

    // Set up analog comparator with 25mV hysteresis and output on PA5
    PORTA.DIRSET = PIN5_bm;
    AC0.CTRLA = AC_OUTEN_bm | AC_HYSMODE_25mV_gc | AC_ENABLE_bm;

    // Set up SPI in slave mode using alternate pins
    PORTMUX.CTRLB |= PORTMUX_SPI0_ALTERNATE_gc;
    SPI0.CTRLA = 0;
    SPI0.CTRLB = SPI_BUFEN_bm | SPI_BUFWR_bm;
    SPI0.CTRLA = SPI_ENABLE_bm;

    // Pull SS (Slave Select) pin low
    PORTA.DIRSET = PIN4_bm;
    PORTA.OUTCLR = PIN4_bm;

    // Configure SCK at fclk/8 = 423.75kHz, output on PB0
    // Configure WO2 with phase shift for CCL input
    PORTB.DIRSET = PIN0_bm;
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SPLIT.CTRLA = 0;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.PER = 7;
    TCA0.SINGLE.CMP0 = 3;
    TCA0.SINGLE.CMP2 = 5;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;

    // Configure CCL for modulation
    PORTMUX.CTRLA |= PORTMUX_LUT1_ALTERNATE_gc;

    // Link CCL_LUT0 output to CCL_LUT1EV0 via ASYNCCH0
    EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_CCL_LUT0_gc;
    EVSYS.ASYNCUSER3 = EVSYS_ASYNCUSER0_ASYNCCH0_gc;

    // Configure CCL for filtered modulation signal on PC1
    CCL.CTRLA = 0;
    CCL.LUT0CTRLA = 0;
    CCL.LUT0CTRLB = CCL_INSEL1_MASK_gc | CCL_INSEL0_MASK_gc;
    CCL.LUT0CTRLC = CCL_INSEL2_SPI0_gc;
    CCL.TRUTH0 = 0xF0;
    CCL.LUT0CTRLA = CCL_ENABLE_bm;
    CCL.LUT1CTRLA = 0;
    CCL.LUT1CTRLB = CCL_INSEL1_MASK_gc | CCL_INSEL0_EVENT0_gc;
    CCL.LUT1CTRLC = CCL_INSEL2_TCA0_gc;
    CCL.TRUTH1 = 0xAA;
    CCL.LUT1CTRLA = CCL_CLKSRC_bm | CCL_FILTSEL0_bm | CCL_OUTEN_bm | CCL_ENABLE_bm;

    // Set up USART for serial output
    PORTMUX.CTRLB |= PORTMUX_USART0_ALTERNATE_gc;
    PORTA.OUTSET = PIN1_bm;
    PORTA.DIRSET = PIN1_bm;
    USART0.BAUD = 118; // 115200bps
    USART0.CTRLB = USART_TXEN_bm;

    // Initialize application layer
    initialize();

    // Print version info
    Serial_println("SiliCa v1.1");
    Serial_print("Build on: ");
    Serial_println(__DATE__);
}

// ============================================================================
// Main Loop
// ============================================================================

void loop()
{
    packet_t cmd = receive_command();
    if (cmd == nullptr)
        return;

    packet_t resp = process(cmd);
    if (resp == nullptr)
    {
        Serial_println("Unsupported command");
        save_error(cmd);
        print_packet(cmd);
        return;
    }

    // Delay for Polling command (1000us + 1500us = 2.5ms)
    if (cmd[1] == 0x00)
        _delay_us(1500);

    send_response(resp);
}
