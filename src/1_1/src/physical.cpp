// Physical and data link layer implementation for SiliCa
// JIS X 6319-4 compatible card implementation
// Optimized for speed

// Define DEBUG to enable debug output (comment out for release build)
// #define DEBUG

#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>
#include <util/delay.h>
#include "physical.h"
#include "application.h"

// ============================================================================
// Compiler Optimization Hints
// ============================================================================

#define FORCE_INLINE __attribute__((always_inline)) inline
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define RESTRICT __restrict__

// ============================================================================
// Constants
// ============================================================================

// Data link layer header (preamble + sync code)
// Stored in RAM for fast access (no pgm_read_byte overhead)
static constexpr uint8_t header[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x4D};
static constexpr int HEADER_SIZE = 8;

// Manchester encoding lookup table
// Stored in RAM for fast access (no pgm_read_byte overhead)
// Aligned for faster memory access
static const uint8_t manchester_table[16] __attribute__((aligned(16))) = {
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

// Buffer for receiving data - aligned for faster access
static uint8_t rx_buf[RX_BUF_SIZE] __attribute__((aligned(4)));

// Buffer for command processing - aligned for faster access
static uint8_t command[CMD_BUF_SIZE] __attribute__((aligned(4)));

// ============================================================================
// Delayed Response for Polling Command
// ============================================================================

// Pending response to be sent after timer expires
static volatile packet_t pending_response = nullptr;

// Flag indicating timer has expired
static volatile bool timer_expired = false;

// Start delay timer using TCB0
// Timer starts counting from capture_frame completion
static FORCE_INLINE void start_delay_timer()
{
    // Reset state
    pending_response = nullptr;
    timer_expired = false;

    // Reset counter and start timer
    TCB0.CNT = 0;
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

// Stop delay timer
static FORCE_INLINE void stop_delay_timer()
{
    TCB0.CTRLA = 0;
}

// TCB0 interrupt handler for delayed response
ISR(TCB0_INT_vect)
{
    // Disable timer and clear interrupt flag
    TCB0.CTRLA = 0;
    TCB0.INTFLAGS = TCB_CAPT_bm;

    // If response is already available, send it
    // Otherwise, set flag to indicate timer has expired
    packet_t resp = pending_response;
    if (resp != nullptr)
    {
        pending_response = nullptr;
        timer_expired = false;
        send_response(resp);
    }
    else
    {
        timer_expired = true;
    }
}

// ============================================================================
// Serial Output Functions
// Functions for serial output.
// These functions perform blocking writes.
// ============================================================================

#ifdef DEBUG
// Serial write - blocking output
FORCE_INLINE void Serial_write(uint8_t data)
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
FORCE_INLINE void Serial_write(uint8_t) {}
void Serial_print(const char *) {}
void Serial_println(const char *) {}
#endif

// ============================================================================
// SPI Functions
// ============================================================================

// Transfer one byte via SPI
// Arduino SPI.transfer() equivalent
// Inline for maximum speed
static FORCE_INLINE uint8_t SPI_transfer(uint8_t data = 0)
{
    while (UNLIKELY(!(SPI0.INTFLAGS & SPI_DREIF_bm)))
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
// Optimized with register hints and restrict pointer
static FORCE_INLINE uint16_t crc16(const uint8_t * RESTRICT buf, int len)
{
    register uint16_t crc = 0;
    const uint8_t * RESTRICT end = buf + len;
    while (buf < end)
    {
        crc = _crc_xmodem_update(crc, *buf++);
    }
    return crc;
}

// ============================================================================
// Frame Capture and Decoding
// ============================================================================

// Capture frame from SPI
// Return length of captured data
// Optimized with register hints
static int capture_frame()
{
    // Wait for start of frame
    for (register int i = 0; i < RX_BUF_SIZE; i++)
    {
        const uint8_t data = SPI_transfer();
        rx_buf[i] = data;

        // End of frame - check for idle line
        if (UNLIKELY(data == 0x00 || data == 0xFF))
        {
            // Frame too short - restart
            if (UNLIKELY(i < HEADER_SIZE * 2))
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
// Also checks inverted pattern and returns shift via pointer
// Returns: 0 = normal, 1 = inverted, -1 = invalid
// Fully unrolled for maximum speed - no loops, no table lookups
static FORCE_INLINE int get_shift_from_sync_ex(uint8_t sync1, uint8_t sync2, int &shift)
{
    // Pre-compute masks for both normal and inverted
    const uint8_t a1 = sync1 & 0xAA;
    const uint8_t b1 = sync1 & 0x55;
    const uint8_t a2 = sync2 & 0xAA;
    const uint8_t b2 = sync2 & 0x55;

    // Check normal patterns (ordered by likely frequency - shift 0 first)
    if (a1 == 0x8A && a2 == 0x08) { shift = 0; return 0; }
    if (b1 == 0x45 && b2 == 0x04) { shift = 1; return 0; }
    if (a1 == 0x22 && a2 == 0x82) { shift = 2; return 0; }
    if (b1 == 0x11 && b2 == 0x41) { shift = 3; return 0; }
    if (a1 == 0x08 && a2 == 0xA0) { shift = 4; return 0; }
    if (b1 == 0x04 && b2 == 0x50) { shift = 5; return 0; }
    if (a1 == 0x02 && a2 == 0x28) { shift = 6; return 0; }
    if (b1 == 0x01 && b2 == 0x14) { shift = 7; return 0; }

    // Check inverted patterns
    const uint8_t inv_a1 = (~sync1) & 0xAA;
    const uint8_t inv_b1 = (~sync1) & 0x55;
    const uint8_t inv_a2 = (~sync2) & 0xAA;
    const uint8_t inv_b2 = (~sync2) & 0x55;

    if (inv_a1 == 0x8A && inv_a2 == 0x08) { shift = 0; return 1; }
    if (inv_b1 == 0x45 && inv_b2 == 0x04) { shift = 1; return 1; }
    if (inv_a1 == 0x22 && inv_a2 == 0x82) { shift = 2; return 1; }
    if (inv_b1 == 0x11 && inv_b2 == 0x41) { shift = 3; return 1; }
    if (inv_a1 == 0x08 && inv_a2 == 0xA0) { shift = 4; return 1; }
    if (inv_b1 == 0x04 && inv_b2 == 0x50) { shift = 5; return 1; }
    if (inv_a1 == 0x02 && inv_a2 == 0x28) { shift = 6; return 1; }
    if (inv_b1 == 0x01 && inv_b2 == 0x14) { shift = 7; return 1; }

    shift = -1;
    return -1; // invalid
}

// Find sync pattern in received data
// Return index of first sync byte
// Optimized: single function call checks both normal and inverted patterns
static FORCE_INLINE int find_sync_index(int rx_len, int &shift, bool &invert)
{
    const int limit = rx_len - 1;
    for (register int i = 0; i < limit; i++)
    {
        const int result = get_shift_from_sync_ex(rx_buf[i], rx_buf[i + 1], shift);
        if (LIKELY(result >= 0))
        {
            invert = (result == 1);
            return i;
        }
    }
    return -1;
}

// Extract one byte from 3 bytes of received data
// according to the specified bit shift
// Optimized using bitwise operations - fully inlined for speed
static FORCE_INLINE uint8_t extract_byte(int shift, uint8_t d0, uint8_t d1, uint8_t d2)
{
    // Use switch for compile-time optimization
    // The compiler will generate efficient code for this pattern
    switch (shift)
    {
    case 0:
        // d0[7,5,3,1] -> r[7,6,5,4], d1[7,5,3,1] -> r[3,2,1,0]
        return ((d0 & 0x80) ? 0x80 : 0) | ((d0 & 0x20) ? 0x40 : 0) |
               ((d0 & 0x08) ? 0x20 : 0) | ((d0 & 0x02) ? 0x10 : 0) |
               ((d1 & 0x80) ? 0x08 : 0) | ((d1 & 0x20) ? 0x04 : 0) |
               ((d1 & 0x08) ? 0x02 : 0) | ((d1 & 0x02) ? 0x01 : 0);
    case 1:
        return ((d0 & 0x40) ? 0x80 : 0) | ((d0 & 0x10) ? 0x40 : 0) |
               ((d0 & 0x04) ? 0x20 : 0) | ((d0 & 0x01) ? 0x10 : 0) |
               ((d1 & 0x40) ? 0x08 : 0) | ((d1 & 0x10) ? 0x04 : 0) |
               ((d1 & 0x04) ? 0x02 : 0) | ((d1 & 0x01) ? 0x01 : 0);
    case 2:
        return ((d0 & 0x20) ? 0x80 : 0) | ((d0 & 0x08) ? 0x40 : 0) |
               ((d0 & 0x02) ? 0x20 : 0) | ((d1 & 0x80) ? 0x10 : 0) |
               ((d1 & 0x20) ? 0x08 : 0) | ((d1 & 0x08) ? 0x04 : 0) |
               ((d1 & 0x02) ? 0x02 : 0) | ((d2 & 0x80) ? 0x01 : 0);
    case 3:
        return ((d0 & 0x10) ? 0x80 : 0) | ((d0 & 0x04) ? 0x40 : 0) |
               ((d0 & 0x01) ? 0x20 : 0) | ((d1 & 0x40) ? 0x10 : 0) |
               ((d1 & 0x10) ? 0x08 : 0) | ((d1 & 0x04) ? 0x04 : 0) |
               ((d1 & 0x01) ? 0x02 : 0) | ((d2 & 0x40) ? 0x01 : 0);
    case 4:
        return ((d0 & 0x08) ? 0x80 : 0) | ((d0 & 0x02) ? 0x40 : 0) |
               ((d1 & 0x80) ? 0x20 : 0) | ((d1 & 0x20) ? 0x10 : 0) |
               ((d1 & 0x08) ? 0x08 : 0) | ((d1 & 0x02) ? 0x04 : 0) |
               ((d2 & 0x80) ? 0x02 : 0) | ((d2 & 0x20) ? 0x01 : 0);
    case 5:
        return ((d0 & 0x04) ? 0x80 : 0) | ((d0 & 0x01) ? 0x40 : 0) |
               ((d1 & 0x40) ? 0x20 : 0) | ((d1 & 0x10) ? 0x10 : 0) |
               ((d1 & 0x04) ? 0x08 : 0) | ((d1 & 0x01) ? 0x04 : 0) |
               ((d2 & 0x40) ? 0x02 : 0) | ((d2 & 0x10) ? 0x01 : 0);
    case 6:
        return ((d0 & 0x02) ? 0x80 : 0) | ((d1 & 0x80) ? 0x40 : 0) |
               ((d1 & 0x20) ? 0x20 : 0) | ((d1 & 0x08) ? 0x10 : 0) |
               ((d1 & 0x02) ? 0x08 : 0) | ((d2 & 0x80) ? 0x04 : 0) |
               ((d2 & 0x20) ? 0x02 : 0) | ((d2 & 0x08) ? 0x01 : 0);
    case 7:
    default:
        return ((d0 & 0x01) ? 0x80 : 0) | ((d1 & 0x40) ? 0x40 : 0) |
               ((d1 & 0x10) ? 0x20 : 0) | ((d1 & 0x04) ? 0x10 : 0) |
               ((d1 & 0x01) ? 0x08 : 0) | ((d2 & 0x40) ? 0x04 : 0) |
               ((d2 & 0x10) ? 0x02 : 0) | ((d2 & 0x04) ? 0x01 : 0);
    }
}

// Receive command packet from the reader
// Return null if error
// Starts delay timer immediately after capture_frame completes
packet_t receive_command()
{
    // Capture frame
    const int rx_len = capture_frame();

    // Start delay timer immediately after capture_frame completes
    // Timer starts counting from this point for Polling command
    // Will be stopped later if not a Polling command or if error occurs
    start_delay_timer();

    if (UNLIKELY(rx_len == 0))
    {
        return nullptr;
    }

    // Find sync pattern
    int shift = -1;
    bool invert;
    int rx_index = find_sync_index(rx_len, shift, invert);
    if (UNLIKELY(rx_index == -1))
    {
        return nullptr;
    }

    // Skip sync pattern (4 bytes in oversampled data = 2 bytes original)
    rx_index += 4;

    // Decode data - optimized with register hints and pointer arithmetic
    register int index = 0;
    const int end_index = rx_len - 2;
    const uint8_t * RESTRICT src = rx_buf + rx_index;
    uint8_t * RESTRICT dst = command;

    if (invert)
    {
        // Inverted path - XOR with 0xFF
        for (register int i = rx_index; i < end_index; i += 2, src += 2)
        {
            *dst++ = ~extract_byte(shift, src[0], src[1], src[2]);
            index++;
        }
    }
    else
    {
        // Normal path - no XOR needed
        for (register int i = rx_index; i < end_index; i += 2, src += 2)
        {
            *dst++ = extract_byte(shift, src[0], src[1], src[2]);
            index++;
        }
    }

    // Verify length
    const int len = command[0];
    if (UNLIKELY(len + 2 > index))
    {
        return nullptr;
    }

    // Verify EDC (Error Detection Code)
    const uint16_t calculated_edc = crc16(command, len);
    const uint16_t received_edc = (static_cast<uint16_t>(command[len]) << 8) | command[len + 1];

    // Allow last 1-bit error
    if (UNLIKELY((calculated_edc ^ received_edc) > 1))
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
static FORCE_INLINE void enable_transmit(bool enable)
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
// Inline for maximum speed - uses direct table indexing
static FORCE_INLINE void transmit_byte(uint8_t data)
{
    SPI_transfer(manchester_table[data >> 4]);
    SPI_transfer(manchester_table[data & 0x0F]);
}

// Send response packet to the reader
// Null response means no response
void send_response(packet_t response)
{
    if (UNLIKELY(response == nullptr))
        return;

    const int len = response[0];

    // Calculate EDC (Error Detection Code) in advance
    const uint16_t edc = crc16(response, len);

    enable_transmit(true);

    // Send header (fully unrolled for maximum speed)
    // header[0-5] are all 0x00, header[6]=0xB2, header[7]=0x4D
    // Manchester encoded: 0x00 -> 0x55,0x55
    SPI_transfer(0x55); SPI_transfer(0x55);  // header[0] = 0x00
    SPI_transfer(0x55); SPI_transfer(0x55);  // header[1] = 0x00
    SPI_transfer(0x55); SPI_transfer(0x55);  // header[2] = 0x00
    SPI_transfer(0x55); SPI_transfer(0x55);  // header[3] = 0x00
    SPI_transfer(0x55); SPI_transfer(0x55);  // header[4] = 0x00
    SPI_transfer(0x55); SPI_transfer(0x55);  // header[5] = 0x00
    transmit_byte(0xB2);                      // header[6] = 0xB2
    transmit_byte(0x4D);                      // header[7] = 0x4D

    // Send body - use pointer for faster iteration
    const uint8_t * RESTRICT ptr = response;
    const uint8_t * RESTRICT end = response + len;
    while (ptr < end)
    {
        transmit_byte(*ptr++);
    }

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

    // Set up TCB0 for delayed response (Polling command)
    // Timer will be started in receive_command after capture_frame
    // 2.417ms - preamble+sync(64bit)@212kbps(302us) = 2.115ms delay at 3.39MHz = 7170 cycles
    TCB0.CCMP = 7170;
    TCB0.CTRLB = TCB_CNTMODE_INT_gc; // Periodic interrupt mode
    TCB0.INTCTRL = TCB_CAPT_bm;      // Enable capture interrupt

    // Enable global interrupts
    sei();

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

// Set response and send when timer expires (or immediately if already expired)
static void send_delayed_response(packet_t response)
{
    // Disable interrupts to safely check and update state
    cli();

    if (timer_expired)
    {
        // Timer already expired, send immediately
        timer_expired = false;
        sei();
        send_response(response);
    }
    else
    {
        // Timer not yet expired, ISR will send the response
        pending_response = response;
        sei();
    }
}

// Main loop
// Process commands continuously
// Note: Timer is started in receive_command() right after capture_frame()
void loop()
{
    // Timer is started in receive_command after capture_frame completes
    packet_t cmd = receive_command();
    if (UNLIKELY(cmd == nullptr))
        return;

    // Check if this is a Polling command
    // Timer was already started in receive_command, so we need to either:
    // - Keep it running for Polling command (cmd[1] == 0x00)
    // - Stop it for other commands
    const bool is_polling = (cmd[1] == 0x00);
    if (LIKELY(!is_polling))
    {
        // Stop timer for non-Polling commands (most common case)
        stop_delay_timer();
    }

    packet_t resp = process(cmd);
    if (UNLIKELY(resp == nullptr))
    {
        save_error(cmd);
        // Stop timer if still running
        stop_delay_timer();
        return;
    }

    if (UNLIKELY(is_polling))
    {
        // Use timer for delayed response (2.417ms from capture_frame completion)
        send_delayed_response(resp);
    }
    else
    {
        send_response(resp);
    }
}
