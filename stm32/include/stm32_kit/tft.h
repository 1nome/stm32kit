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
#include <stdbool.h>

// Potentially move these to config
#define TFT_WIDTH 240
#define TFT_HEIGHT 240
#define TFT_XSTART 0
#define TFT_YSTART 0

// defines and logic sourced from:
// https://github.com/wilmsn/Arduino-ST7789-Library

// not all possible commands; read the datasheet for more
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

#define ST7789_WRMEMC     0x3C

// r, g, b in range 0-255 (248/252); the least significant bits are discarded
uint16_t color565(const uint8_t r, const uint8_t g, const uint8_t b)
{
    return (r & 0xF8) << 8 | (g & 0xFC) << 3 | b >> 3;
}

// internal, initializes the spi to simplex out
void TFT_SPI2_init()
{
    __disable_irq();

    pin_setup_af(SPI2_SCK, PIN_MODE_AF, PIN_PULL_DEFAULT, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF5);
    pin_setup_af(SPI2_MOSI, PIN_MODE_AF, PIN_PULL_DEFAULT, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF5);

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    SPI2->CR1 &= ~SPI_CR1_BR_Msk;   // maximum possible speed
    SPI2->CR1 |= SPI_CR1_CPOL;      // clock 1 when idle
    SPI2->CR1 |= SPI_CR1_CPHA;     // capture data on the second transition (rising edge)
    SPI2->CR1 &= ~SPI_CR1_DFF;      // 8-bit data
    SPI2->CR1 &= ~SPI_CR1_LSBFIRST; // msb first
    SPI2->CR1 |= SPI_CR1_MSTR;      // master mode
    // these are required as the nss pin isn't connected to anything and thus is pulled down
    // the spi then thinks a slave is requesting transfer or something
    SPI2->CR1 |= SPI_CR1_SSM;       // software slave management
    SPI2->CR1 |= SPI_CR1_SSI;       // internal slave select
    SPI2->CR1 |= SPI_CR1_SPE;       // enable spi

    __enable_irq();
}

// because the tft reads the data/command pin on the eight SCL rising edge of all places,
// and the TXE bit set doesn't mean the transfer is complete,
// race conditions will inevitably occur (currently transmitted data can be interpreted as a command and vice versa)
// ** THIS HAS MOST LIKELY BEEN FIXED **

// busy waiting; should be used for small transfers
void SPI2_transmit(const uint8_t data)
{
    while (!(SPI2->SR & SPI_SR_TXE)){}
    SPI2->DR = data;
}

// busily waits for the spi to not be busy
void SPI2_wait_for_idle()
{
    while (!(SPI2->SR & SPI_SR_TXE)){}
    while (SPI2->SR & SPI_SR_BSY){}
}

// writes a command to the display
void TFT_write_command(const uint8_t cmd)
{
    SPI2_wait_for_idle();
    io_set(TFT_DC, 0);
    SPI2_transmit(cmd);
}

// writes data to the display
void TFT_write_data(const uint8_t data)
{
    SPI2_wait_for_idle();
    io_set(TFT_DC, 1);
    SPI2_transmit(data);
}

// writes multiple data to the display
void TFT_write_multiple_data(const uint8_t * data, const uint32_t len)
{
    SPI2_wait_for_idle();
    io_set(TFT_DC, 1);
    for (uint32_t i = 0; i < len; i++)
    {
        SPI2_transmit(data[i]);
    }
}

// internal; initializes the Data/Command and Reset pins
void TFT_pin_init()
{
    __disable_irq();
    pin_setup(TFT_DC, PIN_MODE_OUTPUT, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL);
    pin_setup(TFT_RES, PIN_MODE_OUTPUT, PIN_PULL_NONE, PIN_SPEED_LOW, PIN_TYPE_PUSHPULL);
    __enable_irq();
}

// sets the window where pixels will be written
void TFT_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    x0 += TFT_XSTART;
    x1 += TFT_XSTART;
    y0 += TFT_YSTART;
    y1 += TFT_YSTART;

    TFT_write_command(ST7789_CASET); // column addresses
    {
        const uint8_t data[] = {x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF};
        TFT_write_multiple_data(data, 4);
    }

    TFT_write_command(ST7789_RASET); // row addresses
    {
        const uint8_t data[] = {y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF};
        TFT_write_multiple_data(data, 4);
    }
}

// switches the display to partial mode, parameters are partial area dimensions
void TFT_set_partial_mode(const uint16_t y0, const uint16_t y1)
{
    TFT_write_command(ST7789_PTLAR);
    {
        const uint8_t data[] = {y0 >> 8 , y0 & 0xFF, y1 >> 8 , y1 & 0xFF};
        TFT_write_multiple_data(data, 4);
    }
    TFT_write_command(ST7789_PTLON);
}

// switches the display to normal mode
void TFT_set_normal_mode()
{
    TFT_write_command(ST7789_NORON); // normal display on
}

// internal; initializes the display to 16-bit color and reasonable settings
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
    TFT_set_addr_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
    TFT_write_command(ST7789_INVON); // inversion on
    delay_ms(10);
    TFT_write_command(ST7789_NORON); // normal display on
    delay_ms(10);
    TFT_write_command(ST7789_DISPON); // main screen turns on
    delay_ms(500);
}

// rotates the image
// actually changes how the data is sent from color ram to the display
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

// sets up everything for the display to work
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

// use before writing pixel data to the display; usually after setting the address window
// will start writing from x0, y0
void TFT_start_colors()
{
    TFT_write_command(ST7789_RAMWR); // write to ram
}

// use before writing pixel data to the display; usually after setting the address window
// will start writing from x0, y0; will break if rotation was used
void TFT_continue_colors()
{
    TFT_write_command(ST7789_WRMEMC);
}

// simple drawing functions
// these hog the cpu during drawing

// writes a single pixel
// dismal performance; use only when necessary
void TFT_draw_pixel(const int16_t x, const int16_t y, const uint16_t color)
{
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT || y < 0 || x < 0)
    {
        return;
    }

    TFT_set_addr_window(x, y, x, y);
    TFT_start_colors();
    SPI2_wait_for_idle();
    io_set(TFT_DC, 1);
    SPI2_transmit(color >> 8);
    SPI2_transmit(color);
}

// writes a single color count times to the display
void TFT_push_color(const uint16_t color, int32_t count)
{
    SPI2_wait_for_idle();
    io_set(TFT_DC, 1);
    const uint8_t hi = color >> 8;
    const uint8_t lo = color;
    while (count--)
    {
        SPI2_transmit(hi);
        SPI2_transmit(lo);
    }
}

// draws a vertical line
void TFT_draw_vline(const int16_t x, const int16_t y, int16_t h, const uint16_t color)
{
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT || x < 0)
    {
        return;
    }
    if (y + h - 1 >= TFT_HEIGHT)
    {
        h = TFT_HEIGHT - y;
    }

    TFT_set_addr_window(x, y, x, y + h - 1);
    TFT_start_colors();
    TFT_push_color(color, h);
}

// draws a horizontal line
void TFT_draw_hline(const int16_t x, const int16_t y, int16_t w, const uint16_t color)
{
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT || y < 0)
    {
        return;
    }
    if (x + w - 1 >= TFT_WIDTH)
    {
        w = TFT_WIDTH - x;
    }

    TFT_set_addr_window(x, y, x + w - 1, y);
    TFT_start_colors();
    TFT_push_color(color, w);
}

// draws a rectangle
void TFT_draw_rectangle(const int16_t x, const int16_t y, int16_t w, int16_t h, const uint16_t color)
{
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT)
    {
        return;
    }
    if (y + h - 1 >= TFT_HEIGHT)
    {
        h = TFT_HEIGHT - y;
    }
    if (x + w - 1 >= TFT_WIDTH)
    {
        w = TFT_WIDTH - x;
    }

    TFT_set_addr_window(x, y, x + w - 1, y + h - 1);
    TFT_start_colors();
    TFT_push_color(color, w * h);
}

// inverts colors on the display; for example, black -> white, red -> cyan etc.
void TFT_invert_display(const bool enable)
{
    TFT_write_command(enable ? ST7789_INVOFF : ST7789_INVON);
}

#endif //STM32_KIT_TFT
