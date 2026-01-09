// Physical and data link layer implementation for SiliCa
// JIS X 6319-4 compatible card implementation

// Define DEBUG to enable debug output (comment out for release build)
// #define DEBUG

#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <util/delay.h>
#include "physical.h"
#include "application.h"

// ============================================================================
// Constants
// ============================================================================

// Data link layer header (preamble + sync code)
// Stored in RAM for fast access (no pgm_read_byte overhead)
static constexpr uint8_t header[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x4D};
static constexpr int HEADER_SIZE = 8;

// Manchester encoding lookup table
// Stored in RAM for fast access (no pgm_read_byte overhead)
static const uint8_t manchester_table[16] = {
    0x55, 0x56, 0x59, 0x5A, 0x65, 0x66, 0x69, 0x6A,
    0x95, 0x96, 0x99, 0x9A, 0xA5, 0xA6, 0xA9, 0xAA
};

// ============================================================================
// Buffer Size Configuration
// ============================================================================

// Maximum frame size: preamble(12) + sync(4) + length(2) + data(255) + EDC(4) = 277 bytes
// With 2x oversampling: 554 bytes, round up to 560
static constexpr int RX_BUF_SIZE = 560;

// Maximum command size: length(1) + data(254) + EDC(2) = 257 bytes
static constexpr int CMD_BUF_SIZE = 260;

// ============================================================================
// Buffers
// ============================================================================

// Buffer for receiving data
static uint8_t rx_buf[RX_BUF_SIZE];

// Buffer for command processing
static uint8_t command[CMD_BUF_SIZE];

// ============================================================================
// Serial Output Functions
// Functions for serial output.
// These functions perform blocking writes.
// ============================================================================

#ifdef DEBUG
// Serial write - blocking output
inline void Serial_write(uint8_t data)
{
    while (!(USART0.STATUS & USART_DREIF_bm))
    {
        // do nothing
    }
    USART0.TXDATAL = data;
}

// Print string to serial
void Serial_print(const char *str)
{
    while (*str)
        Serial_write(*str++);
}

// Print string to serial with newline
void Serial_println(const char *str)
{
    Serial_print(str);
    Serial_write('\r');
    Serial_write('\n');
}
#else
// No-op versions for release build
inline void Serial_write(uint8_t) {}
void Serial_print(const char *) {}
void Serial_println(const char *) {}
#endif

// ============================================================================
// SPI Functions
// ============================================================================

// Transfer one byte via SPI
// Arduino SPI.transfer() equivalent
// Inline for maximum speed
static inline __attribute__((always_inline)) uint8_t SPI_transfer(uint8_t data = 0)
{
    while (!(SPI0.INTFLAGS & SPI_DREIF_bm))
    {
        // do nothing
    }
    SPI0.DATA = data;
    return SPI0.DATA;
}

// ============================================================================
// CRC Functions
// ============================================================================

// Calculate CRC16-CCITT
// Inline for maximum speed
static inline __attribute__((always_inline)) uint16_t crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = _crc_xmodem_update(crc, buf[i]);
    return crc;
}

// ============================================================================
// Frame Capture and Decoding
// ============================================================================

// Capture frame from SPI
// Return length of captured data
static int capture_frame()
{
    // Wait for start of frame
    for (int i = 0; i < RX_BUF_SIZE; i++)
    {
        uint8_t data = SPI_transfer();
        rx_buf[i] = data;

        // End of frame
        if (data == 0x00 || data == 0xFF)
        {
            // Frame too short
            if (i < HEADER_SIZE * 2)
            {
                i = -1;
                continue;
            }
            return i + 1;
        }
    }
    // Frame too long
    return 0;
}

// Determine bit shift from sync pattern
// Return -1 if not a valid sync pattern
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

// Find sync pattern in received data
// Return index of first sync byte
static int find_sync_index(int rx_len, int &shift, bool &invert)
{
    for (int i = 0; i < rx_len - 1; i++)
    {
        uint8_t b1 = rx_buf[i];
        uint8_t b2 = rx_buf[i + 1];

        int shift1 = get_shift_from_sync(b1, b2);
        int shift2 = get_shift_from_sync(~b1, ~b2);

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

// Extract one byte from 3 bytes of received data
// according to the specified bit shift
// Fully unrolled for maximum speed (no loops)
static uint8_t extract_byte(int shift, uint8_t d0, uint8_t d1, uint8_t d2)
{
    uint8_t r = 0;
    switch (shift)
    {
    case 0:
        if (d0 & 0x80) r |= 0x80;
        if (d0 & 0x20) r |= 0x40;
        if (d0 & 0x08) r |= 0x20;
        if (d0 & 0x02) r |= 0x10;
        if (d1 & 0x80) r |= 0x08;
        if (d1 & 0x20) r |= 0x04;
        if (d1 & 0x08) r |= 0x02;
        if (d1 & 0x02) r |= 0x01;
        break;
    case 1:
        if (d0 & 0x40) r |= 0x80;
        if (d0 & 0x10) r |= 0x40;
        if (d0 & 0x04) r |= 0x20;
        if (d0 & 0x01) r |= 0x10;
        if (d1 & 0x40) r |= 0x08;
        if (d1 & 0x10) r |= 0x04;
        if (d1 & 0x04) r |= 0x02;
        if (d1 & 0x01) r |= 0x01;
        break;
    case 2:
        if (d0 & 0x20) r |= 0x80;
        if (d0 & 0x08) r |= 0x40;
        if (d0 & 0x02) r |= 0x20;
        if (d1 & 0x80) r |= 0x10;
        if (d1 & 0x20) r |= 0x08;
        if (d1 & 0x08) r |= 0x04;
        if (d1 & 0x02) r |= 0x02;
        if (d2 & 0x80) r |= 0x01;
        break;
    case 3:
        if (d0 & 0x10) r |= 0x80;
        if (d0 & 0x04) r |= 0x40;
        if (d0 & 0x01) r |= 0x20;
        if (d1 & 0x40) r |= 0x10;
        if (d1 & 0x10) r |= 0x08;
        if (d1 & 0x04) r |= 0x04;
        if (d1 & 0x01) r |= 0x02;
        if (d2 & 0x40) r |= 0x01;
        break;
    case 4:
        if (d0 & 0x08) r |= 0x80;
        if (d0 & 0x02) r |= 0x40;
        if (d1 & 0x80) r |= 0x20;
        if (d1 & 0x20) r |= 0x10;
        if (d1 & 0x08) r |= 0x08;
        if (d1 & 0x02) r |= 0x04;
        if (d2 & 0x80) r |= 0x02;
        if (d2 & 0x20) r |= 0x01;
        break;
    case 5:
        if (d0 & 0x04) r |= 0x80;
        if (d0 & 0x01) r |= 0x40;
        if (d1 & 0x40) r |= 0x20;
        if (d1 & 0x10) r |= 0x10;
        if (d1 & 0x04) r |= 0x08;
        if (d1 & 0x01) r |= 0x04;
        if (d2 & 0x40) r |= 0x02;
        if (d2 & 0x10) r |= 0x01;
        break;
    case 6:
        if (d0 & 0x02) r |= 0x80;
        if (d1 & 0x80) r |= 0x40;
        if (d1 & 0x20) r |= 0x20;
        if (d1 & 0x08) r |= 0x10;
        if (d1 & 0x02) r |= 0x08;
        if (d2 & 0x80) r |= 0x04;
        if (d2 & 0x20) r |= 0x02;
        if (d2 & 0x08) r |= 0x01;
        break;
    case 7:
        if (d0 & 0x01) r |= 0x80;
        if (d1 & 0x40) r |= 0x40;
        if (d1 & 0x10) r |= 0x20;
        if (d1 & 0x04) r |= 0x10;
        if (d1 & 0x01) r |= 0x08;
        if (d2 & 0x40) r |= 0x04;
        if (d2 & 0x10) r |= 0x02;
        if (d2 & 0x04) r |= 0x01;
        break;
    }
    return r;
}

// Receive command packet from the reader
// Return null if error
packet_t receive_command()
{
    // Capture frame
    int rx_len = capture_frame();
    if (rx_len == 0)
    {
        return nullptr;
    }

    // Find sync pattern
    int shift = -1;
    bool invert;
    int rx_index = find_sync_index(rx_len, shift, invert);
    if (rx_index == -1)
    {
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
        return nullptr;
    }

    // Verify EDC (Error Detection Code)
    uint16_t calculated_edc = crc16(command, len);
    uint16_t received_edc = (command[len] << 8) | command[len + 1];

    if ((calculated_edc ^ received_edc) <= 1)
    {
        // Allow last 1-bit error
    }
    else
    {
        return nullptr;
    }

    return command;
}

// ============================================================================
// Transmission Functions
// ============================================================================

// Enable or disable transmission
// Inline for maximum speed
static inline __attribute__((always_inline)) void enable_transmit(bool enable)
{
    // Flush buffer
    SPI_transfer(0x00);
    SPI_transfer(0x00);

    if (enable)
        CCL.CTRLA = CCL_ENABLE_bm;
    else
        CCL.CTRLA = 0;
}

// Transmit one byte with Manchester encoding
// Inline for maximum speed
static inline __attribute__((always_inline)) void transmit_byte(uint8_t data)
{
    SPI_transfer(manchester_table[data >> 4]);
    SPI_transfer(manchester_table[data & 0xF]);
}

// Send response packet to the reader
// Null response means no response
void send_response(packet_t response)
{
    if (response == nullptr)
        return;

    int len = response[0];

    // Calculate EDC (Error Detection Code) in advance
    uint16_t edc = crc16(response, len);

    enable_transmit(true);

    // Send header (unrolled for speed)
    transmit_byte(header[0]);
    transmit_byte(header[1]);
    transmit_byte(header[2]);
    transmit_byte(header[3]);
    transmit_byte(header[4]);
    transmit_byte(header[5]);
    transmit_byte(header[6]);
    transmit_byte(header[7]);

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

// System initialization
void setup()
{
    // Configure system clock: set fclk to fc/4 (3.39MHz) using an external clock source
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_ENABLE_bm);

    // Set up the analog comparator with a 25mV hysteresis and enable output on PA5
    PORTA.DIRSET = PIN5_bm;
    AC0.CTRLA = AC_OUTEN_bm | AC_HYSMODE_25mV_gc | AC_ENABLE_bm;

    // Set up SPI in slave mode using alternate pins
    PORTMUX.CTRLB |= PORTMUX_SPI0_ALTERNATE_gc;
    SPI0.CTRLA = 0;
    SPI0.CTRLB = SPI_BUFEN_bm | SPI_BUFWR_bm;
    SPI0.CTRLA = SPI_ENABLE_bm;

    // Pull the SS (Slave Select) pin low
    PORTA.DIRSET = PIN4_bm;
    PORTA.OUTCLR = PIN4_bm;

    // Configure SCK to operate at fclk/8 = 423.75kHz and output on PB0
    // Configure WO2 with a phase shift for CCL input
    PORTB.DIRSET = PIN0_bm;
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SPLIT.CTRLA = 0;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.PER = 7; // Set the period to achieve a frequency of fclk/8
    TCA0.SINGLE.CMP0 = 3;
    TCA0.SINGLE.CMP2 = 5; // Adjust phase shift
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;

    // Adjust CCL (Configurable Custom Logic) for modulation
    PORTMUX.CTRLA |= PORTMUX_LUT1_ALTERNATE_gc;

    // Link CCL_LUT0 output to CCL_LUT1EV0 via ASYNCCH0
    EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_CCL_LUT0_gc;
    EVSYS.ASYNCUSER3 = EVSYS_ASYNCUSER0_ASYNCCH0_gc;

    // Configure CCL to generate a filtered modulation signal on PC1
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

    // Application layer initialization
    initialize();

#ifdef DEBUG
    // Print version info
    Serial_println("SiliCa v1.1 DEBUG");
#endif
}

// ============================================================================
// Main Loop
// ============================================================================

// Main loop
// Process commands continuously
void loop()
{
    packet_t cmd = receive_command();
    if (cmd == nullptr)
        return;

    packet_t resp = process(cmd);
    if (resp == nullptr)
    {
        save_error(cmd);
        return;
    }

    // Delay for Polling command
    // 1000us + 1500us = 2.5ms
    if (cmd[1] == 0x00)
        _delay_us(1500);

    send_response(resp);
}
