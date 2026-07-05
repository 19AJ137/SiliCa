// Physical and data link layer implementation for SiliCa
// JIS X 6319-4 compatible card implementation
//
// Debug output is controlled by the DEBUG macro; see silica.h.

#include "physical.h"
#include "application.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/crc16.h>

// ============================================================================
// Constants
// ============================================================================

// Data link layer header: preamble (6x 0x00) + sync code (0xB2, 0x4D).
static constexpr int HEADER_SIZE = 8;
static constexpr uint8_t header[HEADER_SIZE] = {0x00, 0x00, 0x00, 0x00,
                                                0x00, 0x00, 0xB2, 0x4D};

// Manchester encoding lookup table (one nibble -> one encoded byte).
static const uint8_t manchester_table[16] = {0x55, 0x56, 0x59, 0x5A, 0x65, 0x66,
                                             0x69, 0x6A, 0x95, 0x96, 0x99, 0x9A,
                                             0xA5, 0xA6, 0xA9, 0xAA};

// ============================================================================
// Buffer Size Configuration
// ============================================================================

// Maximum frame size: preamble(12) + sync(4) + length(2) + data(255) + EDC(4) =
// 277 bytes With 2x oversampling: 554 bytes, round up to 560
static constexpr int RX_BUF_SIZE = 560;

// Maximum command size: length(1) + data(254) + EDC(2) = 257 bytes
static constexpr int CMD_BUF_SIZE = 260;

// ============================================================================
// Buffers
// ============================================================================

// Buffer for received (oversampled) frame data
static uint8_t rx_buf[RX_BUF_SIZE];

// Buffer for the decoded command packet
static uint8_t command[CMD_BUF_SIZE];

// ============================================================================
// Delayed Response for Polling Command
// ============================================================================

// Pending response to be sent after timer expires
static volatile packet_t pending_response = nullptr;

// Flag indicating timer has expired
static volatile bool timer_expired = false;

// Start delay timer using TCB0
// Timer starts counting from capture_frame completion
static void start_delay_timer() {
  // Reset state
  pending_response = nullptr;
  timer_expired = false;

  // Reset counter and start timer
  TCB0.CNT = 0;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

// Stop delay timer
static void stop_delay_timer() { TCB0.CTRLA = 0; }

// TCB0 interrupt handler for delayed response
ISR(TCB0_INT_vect) {
  // Disable timer and clear interrupt flag
  TCB0.CTRLA = 0;
  TCB0.INTFLAGS = TCB_CAPT_bm;

  // If response is already available, send it
  // Otherwise, set flag to indicate timer has expired
  packet_t resp = pending_response;
  if (resp != nullptr) {
    pending_response = nullptr;
    timer_expired = false;
    send_response(resp);
  } else {
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
void Serial_write(uint8_t data) {
  while (!(USART0.STATUS & USART_DREIF_bm)) {
    // do nothing
  }
  USART0.TXDATAL = data;
}

// Print string to serial
void Serial_print(const char *str) {
  while (*str)
    Serial_write(*str++);
}

// Print string to serial with newline
void Serial_println(const char *str) {
  Serial_print(str);
  Serial_write('\r');
  Serial_write('\n');
}
#else
// No-op versions for release build
void Serial_write(uint8_t) {}
void Serial_print(const char *) {}
void Serial_println(const char *) {}
#endif

// ============================================================================
// SPI Functions
// ============================================================================

// Transfer one byte via SPI (Arduino SPI.transfer() equivalent).
static uint8_t SPI_transfer(uint8_t data = 0) {
  while (!(SPI0.INTFLAGS & SPI_DREIF_bm)) {
    // do nothing
  }
  SPI0.DATA = data;
  return SPI0.DATA;
}

// ============================================================================
// CRC Functions
// ============================================================================

// Calculate CRC16-CCITT (XMODEM variant)
static uint16_t crc16(const uint8_t *buf, int len) {
  uint16_t crc = 0;
  const uint8_t *end = buf + len;
  while (buf < end) {
    crc = _crc_xmodem_update(crc, *buf++);
  }
  return crc;
}

// ============================================================================
// Frame Capture and Decoding
// ============================================================================
//
// This is the RF-critical hot path: decoding runs against the Polling response
// deadline (the TCB0 delay timer is armed right after capture_frame()). If you
// change get_shift_from_sync_ex or extract_byte, validate timing on real
// hardware.

// Capture frame from SPI
// Return length of captured data
static int capture_frame() {
  // Wait for start of frame
  for (int i = 0; i < RX_BUF_SIZE; i++) {
    const uint8_t data = SPI_transfer();
    rx_buf[i] = data;

    // End of frame - check for idle line
    if (data == 0x00 || data == 0xFF) {
      // Frame too short - restart
      if (i < HEADER_SIZE * 2) {
        i = -1;
        continue;
      }
      return i + 1;
    }
  }
  // Frame too long
  return 0;
}

// Determine bit shift from a 2-byte sync pattern.
// Returns 0 = normal, 1 = inverted, -1 = invalid; the matched shift (0..7) is
// returned via the reference parameter.
static int get_shift_from_sync_ex(uint8_t sync1, uint8_t sync2, int &shift) {
  // One row per shift: {mask, expected sync1 & mask, expected sync2 & mask}.
  // Even shifts occupy the odd bit lanes (0xAA), odd shifts the even (0x55).
  static const uint8_t patterns[8][3] = {
      {0xAA, 0x8A, 0x08}, {0x55, 0x45, 0x04}, {0xAA, 0x22, 0x82},
      {0x55, 0x11, 0x41}, {0xAA, 0x08, 0xA0}, {0x55, 0x04, 0x50},
      {0xAA, 0x02, 0x28}, {0x55, 0x01, 0x14},
  };

  // Check normal patterns first, then inverted.
  for (int s = 0; s < 8; s++) {
    const uint8_t mask = patterns[s][0];
    if ((sync1 & mask) == patterns[s][1] && (sync2 & mask) == patterns[s][2]) {
      shift = s;
      return 0;
    }
  }
  const uint8_t inv1 = ~sync1;
  const uint8_t inv2 = ~sync2;
  for (int s = 0; s < 8; s++) {
    const uint8_t mask = patterns[s][0];
    if ((inv1 & mask) == patterns[s][1] && (inv2 & mask) == patterns[s][2]) {
      shift = s;
      return 1;
    }
  }

  shift = -1;
  return -1; // invalid
}

// Find sync pattern in received data.
// Returns index of first sync byte, or -1 if not found.
static int find_sync_index(int rx_len, int &shift, bool &invert) {
  const int limit = rx_len - 1;
  for (int i = 0; i < limit; i++) {
    const int result = get_shift_from_sync_ex(rx_buf[i], rx_buf[i + 1], shift);
    if (result >= 0) {
      invert = (result == 1);
      return i;
    }
  }
  return -1;
}

// Extract one data byte from 3 oversampled bytes at the given bit shift.
// The bit for output position k (MSB first) sits at bit (23 - shift - 2k) of
// the 24-bit window {d0, d1, d2}; align it to the top, then step 2 bits.
static uint8_t extract_byte(int shift, uint8_t d0, uint8_t d1, uint8_t d2) {
  uint32_t window = ((uint32_t)d0 << 16) | ((uint32_t)d1 << 8) | d2;
  window <<= shift;

  uint8_t result = 0;
  for (int k = 0; k < 8; k++) {
    result = (result << 1) | ((window >> 23) & 1);
    window <<= 2;
  }
  return result;
}

// Receive command packet from the reader
// Return null if error
// Starts delay timer immediately after capture_frame completes
packet_t receive_command() {
  // Capture frame
  const int rx_len = capture_frame();

  // Start delay timer immediately after capture_frame completes
  // Timer starts counting from this point for Polling command
  // Will be stopped later if not a Polling command or if error occurs
  start_delay_timer();

  if (rx_len == 0) {
    return nullptr;
  }

  // Find sync pattern
  int shift = -1;
  bool invert;
  int rx_index = find_sync_index(rx_len, shift, invert);
  if (rx_index == -1) {
    return nullptr;
  }

  // Skip sync pattern (4 bytes in oversampled data = 2 bytes original)
  rx_index += 4;

  // Decode data (the inverted path just XORs each byte with 0xFF)
  const uint8_t xor_mask = invert ? 0xFF : 0x00;
  const int end_index = rx_len - 2;
  int index = 0;
  for (int i = rx_index; i < end_index; i += 2) {
    const uint8_t *src = rx_buf + i;
    command[index++] = extract_byte(shift, src[0], src[1], src[2]) ^ xor_mask;
  }

  // Verify length
  const int len = command[0];
  if (len + 2 > index) {
    return nullptr;
  }

  // Verify EDC (Error Detection Code)
  const uint16_t calculated_edc = crc16(command, len);
  const uint16_t received_edc =
      (static_cast<uint16_t>(command[len]) << 8) | command[len + 1];

  // Allow last 1-bit error
  if ((calculated_edc ^ received_edc) > 1) {
    return nullptr;
  }

  return command;
}

// ============================================================================
// Transmission Functions
// ============================================================================

// Enable or disable transmission
static void enable_transmit(bool enable) {
  // Flush buffer
  SPI_transfer(0x00);
  SPI_transfer(0x00);

  if (enable)
    CCL.CTRLA = CCL_ENABLE_bm;
  else
    CCL.CTRLA = 0;
}

// Transmit one byte with Manchester encoding
static void transmit_byte(uint8_t data) {
  SPI_transfer(manchester_table[data >> 4]);
  SPI_transfer(manchester_table[data & 0x0F]);
}

// Send response packet to the reader
// Null response means no response
void send_response(packet_t response) {
  if (response == nullptr)
    return;

  const int len = response[0];

  // Calculate EDC (Error Detection Code) in advance
  const uint16_t edc = crc16(response, len);

  enable_transmit(true);

  // Send header. Transmission is SPI-clock-bound, so a loop is as fast as
  // unrolling (the SPI busy-wait dwarfs any loop overhead).
  for (int i = 0; i < HEADER_SIZE; i++) {
    transmit_byte(header[i]);
  }

  // Send body (the length byte is included in len)
  for (int i = 0; i < len; i++) {
    transmit_byte(response[i]);
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
void setup() {
  // Configure system clock: set fclk to fc/4 (3.39MHz) using an external clock
  // source
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_ENABLE_bm);

  // Set up the analog comparator with a 25mV hysteresis and enable output on
  // PA5
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
  CCL.LUT1CTRLA =
      CCL_CLKSRC_bm | CCL_FILTSEL0_bm | CCL_OUTEN_bm | CCL_ENABLE_bm;

  // Set up USART for serial output
  PORTMUX.CTRLB |= PORTMUX_USART0_ALTERNATE_gc;
  PORTA.OUTSET = PIN1_bm;
  PORTA.DIRSET = PIN1_bm;
  USART0.BAUD = 118; // 115200bps
  USART0.CTRLB = USART_TXEN_bm;

  // Set up TCB0 for delayed response (Polling command)
  // Timer will be started in receive_command after capture_frame
  // 2.417ms - preamble+sync(64bit)@212kbps(302us) = 2.115ms delay at 3.39MHz =
  // 7170 cycles
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
static void send_delayed_response(packet_t response) {
  // Disable interrupts to safely check and update state
  cli();

  if (timer_expired) {
    // Timer already expired, send immediately
    timer_expired = false;
    sei();
    send_response(response);
  } else {
    // Timer not yet expired, ISR will send the response
    pending_response = response;
    sei();
  }
}

// Main loop
// Process commands continuously
// Note: Timer is started in receive_command() right after capture_frame()
void loop() {
  // Timer is started in receive_command after capture_frame completes
  packet_t cmd = receive_command();
  if (cmd == nullptr)
    return;

  // Check if this is a Polling command
  // Timer was already started in receive_command, so we need to either:
  // - Keep it running for Polling command (cmd[1] == 0x00)
  // - Stop it for other commands
  const bool is_polling = (cmd[1] == 0x00);
  if (!is_polling) {
    // Stop timer for non-Polling commands (most common case)
    stop_delay_timer();
  }

  packet_t resp = process(cmd);
  if (resp == nullptr) {
    save_error(cmd);
    // Stop timer if still running
    stop_delay_timer();
    return;
  }

  if (is_polling) {
    // Use timer for delayed response (2.417ms from capture_frame completion)
    send_delayed_response(resp);
  } else {
    send_response(resp);
  }
}
