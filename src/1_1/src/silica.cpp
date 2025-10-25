// Implementation of the physical and data link layers for
// JIS X 6319-4 compatible card "SiliCa"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <util/crc16.h>
#include "silica.h"

static const uint8_t header[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x4D};

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

packet_t receive_command()
{
    return nullptr;
}

void enable_transmit(bool enable)
{
    if (enable)
        CCL.CTRLA = CCL_ENABLE_bm;
    else
        CCL.CTRLA = 0;
}

uint16_t transmit_byte(uint8_t data, uint16_t crc = 0)
{
    static const uint8_t table[16] = {0x55, 0x56, 0x59, 0x5A, 0x65, 0x66, 0x69, 0x6A, 0x95, 0x96, 0x99, 0x9A, 0xA5, 0xA6, 0xA9, 0xAA};

    SPI_transfer(table[data >> 4]);
    SPI_transfer(table[data & 0xF]);

    // compute the Error Detection Code (EDC) on the fly
    return _crc_xmodem_update(crc, data);
}

void send_response(packet_t response)
{
    if (response == nullptr)
        return;

    int len = response[0];

    uint16_t edc = 0;

    enable_transmit(true);

    // send header
    for (int i = 0; i < sizeof(header); i++)
        transmit_byte(header[i]);

    // send body
    for (int i = 0; i < len; i++)
        edc = transmit_byte(response[i], edc);

    // send footer (EDC)
    transmit_byte(edc >> 8);
    transmit_byte(edc & 0xFF);

    // flash buffer
    SPI_transfer(0xFF);
    SPI_transfer(0xFF);
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

#include <util/delay.h>
void test_1()
{
    uint8_t polling[] = {20, 0x01, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xAB, 0xCD};
    send_response(polling);
    _delay_us(1000);
    return;
}

void loop()
{
    test_1();

    packet_t command = receive_command();
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
