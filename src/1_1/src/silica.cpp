// Implementation of the physical and data link layers for
// JIS X 6319-4 compatible card "SiliCa"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <util/crc16.h>
#include <util/delay.h>
#include "silica.h"

static const uint8_t header[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x4D};

static uint8_t rx_buf[0x220] = {};
static uint8_t command[0x110] = {};

void Serial_write(uint8_t data)
{
    while (!(USART0.STATUS & USART_DREIF_bm))
    {
        // do nothing
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

uint8_t SPI_transfer(uint8_t data = 0)
{
    while (!(SPI0.INTFLAGS & SPI_DREIF_bm))
    {
        // do nothing
    }
    SPI0.DATA = data;
    return SPI0.DATA;
}

uint16_t crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = _crc_xmodem_update(crc, buf[i]);
    return crc;
}

int capture_frame()
{
    // wait for start of frame
    for (int i = 0; i < sizeof(rx_buf); i++)
    {
        uint8_t data = SPI_transfer();
        rx_buf[i] = data;

        // end of frame
        if (data == 0x00 || data == 0xFF)
        {
            if (i < sizeof(header) * 2)
            {
                i = -1;
                continue;
            }
            else
            {
                return i + 1;
            }
        }
    }
    // frame too long
    return 0;
}

void print_frame(int rx_len)
{
    for (int i = 0; i < rx_len; i++)
    {
        char hex_str[5];
        sprintf(hex_str, "%02X", rx_buf[i]);
        Serial_print(hex_str);
        if (i != rx_len - 1)
            Serial_print(" ");
    }
    Serial_println("");
}

int get_shift_from_sync(uint8_t sync1, uint8_t sync2)
{
    uint8_t a1 = sync1 & 0xAA;
    uint8_t b1 = sync1 & 0x55;

    uint8_t a2 = sync2 & 0xAA;
    uint8_t b2 = sync2 & 0x55;

    if (a1 == 0x8A && a2 == 0x08)
        return 0;
    if (b1 == 0x45 && b2 == 0x04)
        return 1;
    if (a1 == 0x22 && a2 == 0x82)
        return 2;
    if (b1 == 0x11 && b2 == 0x41)
        return 3;
    if (a1 == 0x08 && a2 == 0xA0)
        return 4;
    if (b1 == 0x04 && b2 == 0x50)
        return 5;
    if (a1 == 0x02 && a2 == 0x28)
        return 6;
    if (b1 == 0x01 && b2 == 0x14)
        return 7;

    return -1;
}

int get_sync_index(int rx_len, int &shift, bool &invert)
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

uint8_t extract_data(int shift, uint8_t data1, uint8_t data2, uint8_t data3)
{
    uint8_t x = 0;

    if (shift == 0)
    {
        if (data1 & 0x80)
            x |= 0x80;
        if (data1 & 0x20)
            x |= 0x40;
        if (data1 & 0x08)
            x |= 0x20;
        if (data1 & 0x02)
            x |= 0x10;
        if (data2 & 0x80)
            x |= 0x08;
        if (data2 & 0x20)
            x |= 0x04;
        if (data2 & 0x08)
            x |= 0x02;
        if (data2 & 0x02)
            x |= 0x01;
    }
    if (shift == 1)
    {
        if (data1 & 0x40)
            x |= 0x80;
        if (data1 & 0x10)
            x |= 0x40;
        if (data1 & 0x04)
            x |= 0x20;
        if (data1 & 0x01)
            x |= 0x10;
        if (data2 & 0x40)
            x |= 0x08;
        if (data2 & 0x10)
            x |= 0x04;
        if (data2 & 0x04)
            x |= 0x02;
        if (data2 & 0x01)
            x |= 0x01;
    }
    if (shift == 2)
    {
        if (data1 & 0x20)
            x |= 0x80;
        if (data1 & 0x08)
            x |= 0x40;
        if (data1 & 0x02)
            x |= 0x20;
        if (data2 & 0x80)
            x |= 0x10;
        if (data2 & 0x20)
            x |= 0x08;
        if (data2 & 0x08)
            x |= 0x04;
        if (data2 & 0x02)
            x |= 0x02;
        if (data3 & 0x80)
            x |= 0x01;
    }
    if (shift == 3)
    {
        if (data1 & 0x10)
            x |= 0x80;
        if (data1 & 0x04)
            x |= 0x40;
        if (data1 & 0x01)
            x |= 0x20;
        if (data2 & 0x40)
            x |= 0x10;
        if (data2 & 0x10)
            x |= 0x08;
        if (data2 & 0x04)
            x |= 0x04;
        if (data2 & 0x01)
            x |= 0x02;
        if (data3 & 0x40)
            x |= 0x01;
    }
    if (shift == 4)
    {
        if (data1 & 0x08)
            x |= 0x80;
        if (data1 & 0x02)
            x |= 0x40;
        if (data2 & 0x80)
            x |= 0x20;
        if (data2 & 0x20)
            x |= 0x10;
        if (data2 & 0x08)
            x |= 0x08;
        if (data2 & 0x02)
            x |= 0x04;
        if (data3 & 0x80)
            x |= 0x02;
        if (data3 & 0x20)
            x |= 0x01;
    }
    if (shift == 5)
    {
        if (data1 & 0x04)
            x |= 0x80;
        if (data1 & 0x01)
            x |= 0x40;
        if (data2 & 0x40)
            x |= 0x20;
        if (data2 & 0x10)
            x |= 0x10;
        if (data2 & 0x04)
            x |= 0x08;
        if (data2 & 0x01)
            x |= 0x04;
        if (data3 & 0x40)
            x |= 0x02;
        if (data3 & 0x10)
            x |= 0x01;
    }
    if (shift == 6)
    {
        if (data1 & 0x02)
            x |= 0x80;
        if (data2 & 0x80)
            x |= 0x40;
        if (data2 & 0x20)
            x |= 0x20;
        if (data2 & 0x08)
            x |= 0x10;
        if (data2 & 0x02)
            x |= 0x08;
        if (data3 & 0x80)
            x |= 0x04;
        if (data3 & 0x20)
            x |= 0x02;
        if (data3 & 0x08)
            x |= 0x01;
    }
    if (shift == 7)
    {
        if (data1 & 0x01)
            x |= 0x80;
        if (data2 & 0x40)
            x |= 0x40;
        if (data2 & 0x10)
            x |= 0x20;
        if (data2 & 0x04)
            x |= 0x10;
        if (data2 & 0x01)
            x |= 0x08;
        if (data3 & 0x40)
            x |= 0x04;
        if (data3 & 0x10)
            x |= 0x02;
        if (data3 & 0x04)
            x |= 0x01;
    }

    return x;
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
    int rx_index = get_sync_index(rx_len, shift, invert);
    if (rx_index == -1)
    {
        Serial_println("Sync error");
        return nullptr;
    }

    rx_index += 4;

    int index = 0;
    for (int i = rx_index; i < rx_len - 2; i += 2)
    {
        uint8_t x = extract_data(shift, rx_buf[i], rx_buf[i + 1], rx_buf[i + 2]);

        if (invert)
            x = ~x;

        command[index++] = x;
    }

    int len = command[0];
    if (len + 2 != index)
    {
        Serial_println("Length error");
        return nullptr;
    }

    uint16_t calculated_edc = crc16(command, len - 2);
    uint16_t received_edc = command[len - 2] << 8 | command[len - 1];

    if (calculated_edc ^ received_edc <= 1)
    {
        // allow last 1-bit error
    }
    else
    {
        Serial_println("EDC error");
        return nullptr;
    }

    return command;
}

void enable_transmit(bool enable)
{
    // flash buffer
    SPI_transfer(0x00);
    SPI_transfer(0x00);

    if (enable)
        CCL.CTRLA = CCL_ENABLE_bm;
    else
        CCL.CTRLA = 0;
}

void transmit_byte(uint8_t data)
{
    static const uint8_t table[16] = {0x55, 0x56, 0x59, 0x5A, 0x65, 0x66, 0x69, 0x6A, 0x95, 0x96, 0x99, 0x9A, 0xA5, 0xA6, 0xA9, 0xAA};

    SPI_transfer(table[data >> 4]);
    SPI_transfer(table[data & 0xF]);
}

void send_response(packet_t response)
{
    if (response == nullptr)
        return;

    int len = response[0];

    // calculate EDC (Error Detection Code) in advance
    uint16_t edc = crc16(response, len);

    enable_transmit(true);

    // send header
    for (int i = 0; i < sizeof(header); i++)
        transmit_byte(header[i]);

    // send body
    for (int i = 0; i < len; i++)
        transmit_byte(response[i]);

    // send footer (EDC)
    transmit_byte(edc >> 8);
    transmit_byte(edc & 0xFF);

    enable_transmit(false);
}

void setup()
{
    // configure system clock: set fclk to fc/4 (3.39MHz) using an external clock source
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_ENABLE_bm);

    // set up the analog comparator with a 25mV hysteresis and enable output on PA5
    PORTA.DIRSET = PIN5_bm;
    AC0.CTRLA = AC_OUTEN_bm | AC_HYSMODE_25mV_gc | AC_ENABLE_bm;

    // set up SPI in slave mode using alternate pins
    PORTMUX.CTRLB |= PORTMUX_SPI0_ALTERNATE_gc;
    SPI0.CTRLA = 0;
    SPI0.CTRLB = SPI_BUFEN_bm | SPI_BUFWR_bm;
    SPI0.CTRLA = SPI_ENABLE_bm;

    // pull the SS (Slave Select) pin low
    PORTA.DIRSET = PIN4_bm;
    PORTA.OUTCLR = PIN4_bm;

    // configure SCK to operate at fclk/8 = 423.75kHz and output on PB0
    // configure WO2 with a phase shift for CCL input
    PORTB.DIRSET = PIN0_bm;
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SPLIT.CTRLA = 0;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.PER = 7; // Set the period to achieve a frequency of fclk/8
    TCA0.SINGLE.CMP0 = 3;
    TCA0.SINGLE.CMP2 = 5; // adjust phase shift
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;

    // adjust CCL (Configurable Custom Logic) for modulation
    PORTMUX.CTRLA |= PORTMUX_LUT1_ALTERNATE_gc;

    // link CCL_LUT0 output to CCL_LUT1EV0 via ASYNCCH0
    EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_CCL_LUT0_gc;
    EVSYS.ASYNCUSER3 = EVSYS_ASYNCUSER0_ASYNCCH0_gc;

    // configure CCL to generate a filtered modulation signal on PC1
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

    // set up USART for serial output
    PORTMUX.CTRLB |= PORTMUX_USART0_ALTERNATE_gc;
    PORTA.OUTSET = PIN1_bm;
    PORTA.DIRSET = PIN1_bm;
    USART0.BAUD = 118; // 115200bps
    USART0.CTRLB = USART_TXEN_bm;

    initialize();

    Serial_println("SiliCa v1.1");
    Serial_print("Build on: ");
    Serial_println(__DATE__);
}

void test_response()
{
    static const uint8_t polling[20] = {20, 0x01, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xAB, 0xCD};
    send_response(polling);
    _delay_us(1000);
}

void loop()
{
    packet_t command = receive_command();
    if (command == nullptr)
        return;

    packet_t response = process(command);
    if (response == nullptr)
    {
        Serial_println("Unsupported command");
        print_packet(command);
        return;
    }

    // delay for Polling command
    // 1000us + 1500us = 2.5ms
    if (command[1] == 0x00)
        _delay_us(1500);

    send_response(response);
}

int main()
{
    setup();

    while (true)
    {
        loop();
    }
}
