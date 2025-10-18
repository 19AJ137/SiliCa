// JIS X 6319-4 compatible card "SiliCa"
// physical and data link layer implementation

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <util/crc16.h>
#include "silica.h"

static const uint8_t header[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x4D};
const int LEN_HEADER = sizeof(header);

uint8_t buf[0x110] = {};
uint8_t rx_buf[0x220] = {};

void Serial_write(uint8_t b)
{
    while ((USART0.STATUS & USART_DREIF_bm) == 0)
    {
        // do nothing
    }
    USART0.TXDATAL = b;
}

void Serial_print(const char *s)
{
    while (*s)
        Serial_write(*s++);
}

void Serial_println(const char *s)
{
    Serial_print(s);
    Serial_print("\r\n");
}

void enable_transmit(bool mode)
{
    if (mode)
    {
        PORTC.DIRSET = PIN1_bm;
        CCL.CTRLA = CCL_ENABLE_bm;
    }
    else
    {
        CCL.CTRLA = 0;
        PORTC.DIRCLR = PIN1_bm;
    }
}

uint16_t crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = _crc_xmodem_update(crc, data[i]);
    return crc;
}

uint8_t SPI_transfer(uint8_t data)
{
    while ((SPI0.INTFLAGS & SPI_DREIF_bm) == 0)
    {
        // do nothing
    }
    SPI0.DATA = data;
    return SPI0.DATA;
}

int get_shift(uint8_t sync)
{
    switch (sync)
    {
    case 0x9A:
        return 0;
    case 0x65:
    case 0XCD:
        return 1;
    case 0x32:
    case 0x66:
        return 2;
    case 0x99:
    case 0xB3:
        return 3;
    case 0x4C:
    case 0x59:
        return 4;
    case 0xA6:
    case 0xAC:
        return 5;
    case 0x53:
    case 0x56:
        return 6;
    case 0xA9:
    case 0xAB:
        return 7;
    case 0x54:
        return 8;
    default:
        return -1;
    }
}

packet_t receive_command()
{
    enable_transmit(false);

    int len_raw = -1;
    for (int i = 0; i < sizeof(rx_buf); i++)
    {
        uint8_t data = SPI_transfer(0);

        rx_buf[i] = data;

        // end of frame
        if (data == 0x00 || data == 0xFF)
        {
            // too short frame
            if (i < LEN_HEADER * 2)
            {
                i = -1;
                continue;
            }
            else
            {
                len_raw = i + 1;
                break;
            }
        }
    }
    if (len_raw < 0)
        return nullptr;

    if (false)
    {
        Serial_println("BEGIN");
        for (int i = 0; i < len_raw; i++)
        {
            char hex_str[5];
            sprintf(hex_str, "%02X", rx_buf[i]);
            Serial_print(hex_str);
            if (i != len_raw - 1)
                Serial_print(" ");
        }
        Serial_println("");
        Serial_println("END");

        return nullptr;
    }

    int index;
    // skip preambles
    for (index = 1; index < sizeof(rx_buf); index++)
    {
        if (rx_buf[index] != 0x55 && rx_buf[index] != 0xAA)
            break;
    }

    // preamble too short
    if (index < 10)
        return nullptr;

    // get shift from first sync byte
    int shift = get_shift(rx_buf[index]);
    if (shift < 0)
        return nullptr;

    // skip sync bytes
    index += 4;

    // original code
    if (false)
    {
        int k = 0;
        for (int j = index * 8 + shift; j < len_raw * 8; j += 2)
        {
            int l = j / 8;
            int m = 7 - j % 8;
            int n = (rx_buf[l] >> m) & 1;

            int p = k / 8;
            int q = 7 - k % 8;
            if (q == 7)
                buf[p] = 0;

            buf[p] |= n << q;
            k++;
        }
    }

    // AI generated code
    int src = index * 8 + shift;
    int end = len_raw * 8;
    int dst_byte = 0;
    int dst_bit = 7;
    int src_byte = src >> 3;
    int src_bit_pos = 7 - (src & 7);
    uint8_t src_cur = rx_buf[src_byte];

    // assemble bits two-at-a-time from rx_buf into buf, avoid divisions/mods each loop
    while (src < end)
    {
        uint8_t bit = (src_cur >> src_bit_pos) & 1;
        if (dst_bit == 7)
            buf[dst_byte] = 0;
        buf[dst_byte] |= bit << dst_bit;

        dst_bit--;
        if (dst_bit < 0)
        {
            dst_byte++;
            dst_bit = 7;
        }

        src += 2;
        src_bit_pos -= 2;
        if (src_bit_pos < 0)
        {
            src_byte = src >> 3;
            src_cur = rx_buf[src_byte];
            src_bit_pos = 7 - (src & 7);
        }
    }

    int len = buf[0];

    // check EDC
    if (crc16(buf, len + 2) != 0)
    {
        Serial_println("EDC error");
        return nullptr;
    }

    return buf;
}

void tx_byte(uint8_t b)
{
    static const uint8_t table[16] = {0x55, 0x56, 0x59, 0x5A, 0x65, 0x66, 0x69, 0x6A, 0x95, 0x96, 0x99, 0x9A, 0xA5, 0xA6, 0xA9, 0xAA};

    SPI_transfer(table[b >> 4]);
    SPI_transfer(table[b & 0xF]);
}

void send_response(packet_t response)
{
    if (response == nullptr)
        return;

    int len = response[0];

    // compute EDC (Error Detection Code) in advance
    uint16_t edc = crc16(response, len);

    enable_transmit(true);

    // stabilize output
    for (int i = 0; i < 10; i++)
        SPI_transfer(0xFF);

    // send header
    for (int i = 0; i < LEN_HEADER; i++)
        tx_byte(header[i]);

    // send body
    for (int i = 0; i < len; i++)
        tx_byte(response[i]);

    // send footer (EDC)
    tx_byte(edc >> 8);
    tx_byte(edc & 0xFF);

    // flash buffer
    SPI_transfer(0xFF);
    SPI_transfer(0xFF);
}

void setup()
{
    // set system clock: fclk = fc/4 = 3.39MHz using external clock source
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_EXTCLK_gc);
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_ENABLE_bm);

    // enable Analog Comparator 0 (AC0) with 50mV hysteresis
    PORTA.DIRSET = PIN5_bm;
    AC0.CTRLA = AC_OUTEN_bm | AC_HYSMODE_50mV_gc | AC_ENABLE_bm;

    // configure SPI
    PORTMUX.CTRLB |= PORTMUX_SPI0_ALTERNATE_gc;
    SPI0.CTRLA = 0;
    SPI0.CTRLB = SPI_BUFEN_bm | SPI_BUFWR_bm;
    SPI0.CTRLA = SPI_ENABLE_bm;

    // set SS low
    PORTA.DIRSET = PIN4_bm;
    PORTA.OUTCLR = PIN4_bm;

    // set SCK frequency to fclk/8 = 423.75kHz
    PORTB.DIRSET = PIN0_bm;
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SPLIT.CTRLA = 0;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.PER = 7; // fs = fclk/8
    TCA0.SINGLE.CMP0 = 3;
    TCA0.SINGLE.CMP2 = 5; // shift phase
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm;

    // CCL
    PORTMUX.CTRLA |= PORTMUX_LUT0_ALTERNATE_gc;
    PORTMUX.CTRLA |= PORTMUX_LUT1_ALTERNATE_gc;

    PORTB.DIRSET = PIN4_bm; // LUT0-OUT ALT
    PORTC.DIRSET = PIN1_bm; // LUT1-OUT ALT

    CCL.CTRLA = 0;

    CCL.LUT0CTRLA = 0;
    CCL.LUT0CTRLB = CCL_INSEL1_MASK_gc | CCL_INSEL0_MASK_gc;
    CCL.LUT0CTRLC = CCL_INSEL2_SPI0_gc;
    CCL.TRUTH0 = 0xF0;

    CCL.LUT1CTRLA = 0;
    CCL.LUT1CTRLB = CCL_INSEL1_MASK_gc | CCL_INSEL0_LINK_gc;
    CCL.LUT1CTRLC = CCL_INSEL2_TCA0_gc;
    CCL.TRUTH1 = 0xAA;

    CCL.LUT0CTRLA = CCL_OUTEN_bm | CCL_ENABLE_bm; // set OUTEN to avoid errata
    CCL.LUT1CTRLA = CCL_CLKSRC_bm | CCL_FILTSEL0_bm | CCL_OUTEN_bm | CCL_ENABLE_bm;

    CCL.CTRLA = CCL_ENABLE_bm;

    // configure USART for Serial output
    PORTMUX.CTRLB |= PORTMUX_USART0_ALTERNATE_gc;
    PORTA.OUTSET = PIN1_bm;
    PORTA.DIRSET = PIN1_bm;
    // USART0.BAUD = 1413; // 9600bps
    USART0.BAUD = 118; // 115200bps
    USART0.CTRLB = USART_TXEN_bm;

    initialize();

    Serial_println("SiliCa v1.1");
    Serial_print("Build on: ");
    Serial_println(__DATE__);
}

void loop()
{
    packet_t command = receive_command();
    if (command == nullptr)
        return;

    packet_t response = process(command);
    if (response == nullptr)
        return;

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
