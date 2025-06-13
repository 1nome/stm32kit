/**
* @file tft.h
 *
 * @author 1nome
 *
 * @brief ST7789 driver for the GMT130 display without Chip Select.
 * Uses two gpio pins and spi2 todo: maybe change to spi1
 *
 * @note Uses delays during initialization.
 */
#ifndef STM32_KIT_TFT
#define STM32_KIT_TFT

#include "pin.h"
#include "boards.h"

// Potentially move these to config
#define TFT_WIDTH 240
#define TFT_HEIGHT 240
#define TFT_XSTART 0
#define TFT_YSTART 0

// defines and logic sourced from:
// https://github.com/wilmsn/Arduino-ST7789-Library

#define ST_CMD_DELAY   0x80    // special signifier for command lists

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

#define ST7789_MADCTL_MY  0x80
#define ST7789_MADCTL_MX  0x40
#define ST7789_MADCTL_MV  0x20
#define ST7789_MADCTL_ML  0x10
#define ST7789_MADCTL_RGB 0x00

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

// r, g, b in range 0-255 (248/252); the least significant bits are discarded
uint16_t color565(const uint8_t r, const uint8_t g, const uint8_t b)
{
    return (r & 0xF8) << 8 | (g & 0xFC) << 3 | b >> 3;
}

void TFT_SPI2_init()
{
    __disable_irq();

    pin_setup_af(SPI2_SCK, PIN_MODE_AF, PIN_PULL_DEFAULT, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF5);
    pin_setup_af(SPI2_MOSI, PIN_MODE_AF, PIN_PULL_DEFAULT, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF5);

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    SPI2->CR1 &= ~SPI_CR1_BR_Msk;   // maximum possible speed
    SPI2->CR1 |= SPI_CR1_CPOL;      // clock 1 when idle
    SPI2->CR1 &= ~SPI_CR1_CPHA;     // capture data on the first transition (falling edge)
    // todo: test with 16-bit
    SPI2->CR1 &= ~SPI_CR1_DFF;      // 8-bit data
    SPI2->CR1 &= ~SPI_CR1_LSBFIRST; // msb first
    SPI2->CR1 |= SPI_CR1_MSTR;      // master mode
    SPI2->CR1 |= SPI_CR1_SPE;       // enable spi

    __enable_irq();
}

// busy waiting; should be used for small transfers
void SPI2_transmit(const uint8_t data)
{
    SPI2->DR = data;
    while (!(SPI2->SR & SPI_SR_TXE)){}
}

void TFT_write_command(const uint8_t cmd)
{
    io_set(TFT_DC, 0);
    SPI2_transmit(cmd);
}

void TFT_write_data(const uint8_t data)
{
    io_set(TFT_DC, 1);
    SPI2_transmit(data);
}

void TFT_pin_init()
{
    pin_setup(TFT_DC, PIN_MODE_OUTPUT, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL);
    pin_setup(TFT_RES, PIN_MODE_OUTPUT, PIN_PULL_NONE, PIN_SPEED_LOW, PIN_TYPE_PUSHPULL);
}

void TFT_display_init()
{
    io_set(TFT_DC, 1);

    TFT_write_command(ST7789_SWRESET); // reset
    delay_ms(150);
    TFT_write_command(ST7789_SLPOUT); // wake up
    delay_ms(500);
    TFT_write_command(ST7789_COLMOD); // color mode
    TFT_write_data(0x55); // 16-bit color
    delay_ms(10);
    TFT_write_command(ST7789_MADCTL); // memory access rules
    TFT_write_data(0x00); // [row, col], bottom to top
    TFT_write_command(ST7789_CASET); // column addresses
    TFT_write_data(0x00);
    TFT_write_data(TFT_XSTART);
    TFT_write_data((TFT_WIDTH + TFT_XSTART) >> 8);
    TFT_write_data((TFT_WIDTH + TFT_XSTART) & 0xFF);
    TFT_write_command(ST7789_RASET); // row addresses
    TFT_write_data(0x00);
    TFT_write_data(TFT_YSTART);
    TFT_write_data((TFT_HEIGHT + TFT_YSTART) >> 8);
    TFT_write_data((TFT_HEIGHT + TFT_YSTART) & 0xFF);
    TFT_write_command(ST7789_INVON); // inversion on
    delay_ms(10);
    TFT_write_command(ST7789_NORON); // normal display on
    delay_ms(10);
    TFT_write_command(ST7789_DISPON); // main screen turn on
    delay_ms(500);
}

void TFT_set_rotation(uint8_t rotation)
{
    TFT_write_command(ST7789_MADCTL);
    rotation %= 4;
    uint8_t arg = 0x00;
    switch(rotation)
    {
        case 0:
        arg = ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB;
        break;
        case 1:
        arg = ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB;
        break;
        case 2:
        arg = ST7789_MADCTL_RGB;
        break;
        case 3:
        arg = ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB;
        break;
    default: ;
    }
    TFT_write_data(arg);
}

void TFT_setup()
{
    TFT_pin_init();
    TFT_SPI2_init();

    io_set(TFT_RES, 1);
    delay_ms(50);
    io_set(TFT_RES, 0);
    delay_ms(50);
    io_set(TFT_RES, 1);
    delay_ms(50);

    TFT_display_init();

    TFT_set_rotation(2); // effectively a no-op
}

#endif //STM32_KIT_TFT
