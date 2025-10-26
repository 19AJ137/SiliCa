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

static uint8_t rx_buf[0x110] = {};
// static uint8_t rx_buf[0xFF] = {};

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

uint16_t crc_update_table(uint16_t crc, uint8_t data)
{
    static const uint16_t table[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};

    return (crc << 8) ^ table[((crc >> 8) ^ data) & 0xFF];
}

uint16_t crc16_table(const uint8_t *buf, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++)
        crc = crc_update_table(crc, buf[i]);
    return crc;
}

uint8_t get_x(int shift, uint8_t data1, uint8_t data2, uint8_t data3)
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

bool func2(int shift, bool invert)
{
    int len = 0xFF;

    uint8_t data1 = SPI_transfer();
    for (int i = 0; i < len + 2; i++)
    {
        uint8_t data2 = SPI_transfer();
        uint8_t data3 = SPI_transfer();

        uint8_t x = get_x(shift, data1, data2, data3);

        if (invert)
            x = ~x;

        if (i == 0)
            len = x;

        rx_buf[i] = x;

        data1 = data3;
    }

    return true;
}

// also receive response packets from another card
packet_t receive_packet()
{
    uint8_t sync1;
    while (true)
    {
        sync1 = SPI_transfer();
        if (sync1 != 0x55 && sync1 != 0xAA)
            break;
    }
    uint8_t a1 = sync1 & 0xAA;
    uint8_t b1 = sync1 & 0x55;

    uint8_t sync2 = SPI_transfer();
    uint8_t a2 = sync2 & 0xAA;
    uint8_t b2 = sync2 & 0x55;

    int shift = -1;

    if (a1 == 0x8A && a2 == 0x08)
        shift = 0;
    if (b1 == 0x45 && b2 == 0x04)
        shift = 1;
    if (a1 == 0x22 && a2 == 0x82)
        shift = 2;
    if (b1 == 0x11 && b2 == 0x41)
        shift = 3;
    if (a1 == 0x08 && a2 == 0xA0)
        shift = 4;
    if (b1 == 0x04 && b2 == 0x50)
        shift = 5;
    if (a1 == 0x02 && a2 == 0x28)
        shift = 6;
    if (b1 == 0x01 && b2 == 0x14)
        shift = 7;

    if (shift < 0)
        return nullptr;

    // skip the next 2 sync bytes
    SPI_transfer();
    SPI_transfer();

    // if (!func2(shift, false))
    // {
    //     Serial_println("EDC error");
    //     return nullptr;
    // }

    func2(shift, false);

    int len = rx_buf[0];
    if (crc16(rx_buf, len + 2) != 0)
    {
        Serial_println("EDC error");
        return nullptr;
    }

    return rx_buf;
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

uint16_t transmit_byte_with_crc(uint8_t data, uint16_t crc = 0)
{
    static const uint8_t table[16] = {0x55, 0x56, 0x59, 0x5A, 0x65, 0x66, 0x69, 0x6A, 0x95, 0x96, 0x99, 0x9A, 0xA5, 0xA6, 0xA9, 0xAA};

    SPI_transfer(table[data >> 4]);
    SPI_transfer(table[data & 0xF]);

    // calculate EDC (Error Detection Code) on the fly
    return crc_update_table(crc, data);
}

void send_response(packet_t response)
{
    if (response == nullptr)
        return;

    int len = response[0];

    // calculate EDC (Error Detection Code) in advance
    uint16_t edc = crc16(response, len);
    // uint16_t edc = 0;

    enable_transmit(true);

    // send header
    for (int i = 0; i < sizeof(header); i++)
        transmit_byte(header[i]);

    // send body
    for (int i = 0; i < len; i++)
        transmit_byte(response[i]);
    // edc = transmit_byte(response[i], edc);

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

void loop()
{
    packet_t command = receive_packet();
    if (command == nullptr)
        return;

    packet_t response = process(command);
    if (response == nullptr)
    {
        Serial_print("Unsupported command: ");
        print_packet(command);
        return;
    }

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
