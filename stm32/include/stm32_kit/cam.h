/**
 * @file cam.h
 * @author 1nome
 *
 * @brief   Driver for the OV7670 camera module.
 *          It uses the onboard DCMI for image data, MCO1 for clock out and I2C1 for control.
 *
 * @attention As the DCMI uses pins otherwise used for I2S3, this driver is incompatible with audio_dac.h.
 */

#ifndef STM32_KIT_CAM
#define STM32_KIT_CAM

#include "stm32_kit/i2c.h"
#include "stm32_kit/dma.h"

typedef enum{
    HSI = 0,
    LSE,
    HSE,
    PLL
} MCO1_src;

typedef enum
{
    Div1 = 0,
    Div2 = 4,
    Div3,
    Div4,
    Div5
} MCO_pre;

// sets up the microcontroller clock out 1 pin
void MCO1_setup(const MCO1_src clkSrc, const MCO_pre div)
{
    pin_setup_af(MCO1, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF0);
    RCC->CFGR &= ~RCC_CFGR_MCO1;
    RCC->CFGR |= (clkSrc & RCC_CFGR_MCO1) << RCC_CFGR_MCO1_Pos;
    RCC->CFGR &= ~RCC_CFGR_MCO1PRE;
    RCC->CFGR |= (div & RCC_CFGR_MCO1PRE) << RCC_CFGR_MCO1PRE_Pos;
}

// sets up pins and enables clock for the DCMI
void DCMI_init()
{
    pin_setup_af(DCMI_D0, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF13);
    pin_setup_af(DCMI_D1, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF13);
    pin_setup_af(DCMI_D2, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF13);
    pin_setup_af(DCMI_D3, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF13);
    pin_setup_af(DCMI_D4, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF13);
    pin_setup_af(DCMI_D5, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF13);
    pin_setup_af(DCMI_D6, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF13);
    pin_setup_af(DCMI_D7, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF13);
    pin_setup_af(DCMI_PIXCLK, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL, PIN_AF13);
    pin_setup_af(DCMI_VSYNC, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_LOW, PIN_TYPE_PUSHPULL, PIN_AF13);
    pin_setup_af(DCMI_HSYNC, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_MID, PIN_TYPE_PUSHPULL, PIN_AF13);

    RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;
}

// configs the DCMI for the OV7670
void OV7670_DCMI_setup()
{
    DCMI->CR &= ~(DCMI_CR_JPEG |        // raw video
        DCMI_CR_ESS |                   // use hardware sync
        DCMI_CR_HSPOL |                 // hsync active low
        DCMI_CR_EDM_0 | DCMI_CR_EDM_1); // use 8-bit DCMI

    DCMI->CR |= DCMI_CR_PCKPOL |        // data valid on rising edge
        DCMI_CR_VSPOL |                 // vsync active high
        DCMI_CR_ENABLE;                 // enable DCMI
}

#define OV7670_WRITE_ADDR 0x42
#define OV7670_READ_ADDR 0x43

// config code from the linux kernel (GPL license)
// used mostly for making the image look any good
// ***********************************************************************

// https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov7670.c
/* Registers */
#define REG_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BLUE	0x01	/* blue gain */
#define REG_RED		0x02	/* red gain */
#define REG_VREF	0x03	/* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1	0x04	/* Control 1 */
#define  COM1_CCIR656	  0x40  /* CCIR656 enable */
#define REG_BAVE	0x05	/* U/B Average level */
#define REG_GbAVE	0x06	/* Y/Gb Average level */
#define REG_AECHH	0x07	/* AEC MS 5 bits */
#define REG_RAVE	0x08	/* V/R Average level */
#define REG_COM2	0x09	/* Control 2 */
#define  COM2_SSLEEP	  0x10	/* Soft sleep mode */
#define REG_PID		0x0a	/* Product ID MSB */
#define REG_VER		0x0b	/* Product ID LSB */
#define REG_COM3	0x0c	/* Control 3 */
#define  COM3_SWAP	  0x40	  /* Byte swap */
#define  COM3_SCALEEN	  0x08	  /* Enable scaling */
#define  COM3_DCWEN	  0x04	  /* Enable downsamp/crop/window */
#define REG_COM4	0x0d	/* Control 4 */
#define REG_COM5	0x0e	/* All "reserved" */
#define REG_COM6	0x0f	/* Control 6 */
#define REG_AECH	0x10	/* More bits of AEC value */
#define REG_CLKRC	0x11	/* Clocl control */
#define   CLK_EXT	  0x40	  /* Use external clock directly */
#define   CLK_SCALE	  0x3f	  /* Mask for internal clock scale */
#define REG_COM7	0x12	/* Control 7 */
#define   COM7_RESET	  0x80	  /* Register reset */
#define   COM7_FMT_MASK	  0x38
#define   COM7_FMT_VGA	  0x00
#define	  COM7_FMT_CIF	  0x20	  /* CIF format */
#define   COM7_FMT_QVGA	  0x10	  /* QVGA format */
#define   COM7_FMT_QCIF	  0x08	  /* QCIF format */
#define	  COM7_RGB	  0x04	  /* bits 0 and 2 - RGB format */
#define	  COM7_YUV	  0x00	  /* YUV */
#define	  COM7_BAYER	  0x01	  /* Bayer format */
#define	  COM7_PBAYER	  0x05	  /* "Processed bayer" */
#define REG_COM8	0x13	/* Control 8 */
#define   COM8_FASTAEC	  0x80	  /* Enable fast AGC/AEC */
#define   COM8_AECSTEP	  0x40	  /* Unlimited AEC step size */
#define   COM8_BFILT	  0x20	  /* Band filter enable */
#define   COM8_AGC	  0x04	  /* Auto gain enable */
#define   COM8_AWB	  0x02	  /* White balance enable */
#define   COM8_AEC	  0x01	  /* Auto exposure enable */
#define REG_COM9	0x14	/* Control 9  - gain ceiling */
#define REG_COM10	0x15	/* Control 10 */
#define   COM10_HSYNC	  0x40	  /* HSYNC instead of HREF */
#define   COM10_PCLK_HB	  0x20	  /* Suppress PCLK on horiz blank */
#define   COM10_HREF_REV  0x08	  /* Reverse HREF */
#define   COM10_VS_LEAD	  0x04	  /* VSYNC on clock leading edge */
#define   COM10_VS_NEG	  0x02	  /* VSYNC negative */
#define   COM10_HS_NEG	  0x01	  /* HSYNC negative */
#define REG_HSTART	0x17	/* Horiz start high bits */
#define REG_HSTOP	0x18	/* Horiz stop high bits */
#define REG_VSTART	0x19	/* Vert start high bits */
#define REG_VSTOP	0x1a	/* Vert stop high bits */
#define REG_PSHFT	0x1b	/* Pixel delay after HREF */
#define REG_MIDH	0x1c	/* Manuf. ID high */
#define REG_MIDL	0x1d	/* Manuf. ID low */
#define REG_MVFP	0x1e	/* Mirror / vflip */
#define   MVFP_MIRROR	  0x20	  /* Mirror image */
#define   MVFP_FLIP	  0x10	  /* Vertical flip */

#define REG_AEW		0x24	/* AGC upper limit */
#define REG_AEB		0x25	/* AGC lower limit */
#define REG_VPT		0x26	/* AGC/AEC fast mode op region */
#define REG_HSYST	0x30	/* HSYNC rising edge delay */
#define REG_HSYEN	0x31	/* HSYNC falling edge delay */
#define REG_HREF	0x32	/* HREF pieces */
#define REG_TSLB	0x3a	/* lots of stuff */
#define   TSLB_YLAST	  0x04	  /* UYVY or VYUY - see com13 */
#define REG_COM11	0x3b	/* Control 11 */
#define   COM11_NIGHT	  0x80	  /* NIght mode enable */
#define   COM11_NMFR	  0x60	  /* Two bit NM frame rate */
#define   COM11_HZAUTO	  0x10	  /* Auto detect 50/60 Hz */
#define	  COM11_50HZ	  0x08	  /* Manual 50Hz select */
#define   COM11_EXP	  0x02
#define REG_COM12	0x3c	/* Control 12 */
#define   COM12_HREF	  0x80	  /* HREF always */
#define REG_COM13	0x3d	/* Control 13 */
#define   COM13_GAMMA	  0x80	  /* Gamma enable */
#define	  COM13_UVSAT	  0x40	  /* UV saturation auto adjustment */
#define   COM13_UVSWAP	  0x01	  /* V before U - w/TSLB */
#define REG_COM14	0x3e	/* Control 14 */
#define   COM14_DCWEN	  0x10	  /* DCW/PCLK-scale enable */
#define REG_EDGE	0x3f	/* Edge enhancement factor */
#define REG_COM15	0x40	/* Control 15 */
#define   COM15_R10F0	  0x00	  /* Data range 10 to F0 */
#define	  COM15_R01FE	  0x80	  /*            01 to FE */
#define   COM15_R00FF	  0xc0	  /*            00 to FF */
#define   COM15_RGB565	  0x10	  /* RGB565 output */
#define   COM15_RGB555	  0x30	  /* RGB555 output */
#define REG_COM16	0x41	/* Control 16 */
#define   COM16_AWBGAIN   0x08	  /* AWB gain enable */
#define REG_COM17	0x42	/* Control 17 */
#define   COM17_AECWIN	  0xc0	  /* AEC window - must match COM4 */
#define   COM17_CBAR	  0x08	  /* DSP Color bar */

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define	REG_CMATRIX_BASE 0x4f
#define   CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58

#define REG_BRIGHT	0x55	/* Brightness */
#define REG_CONTRAS	0x56	/* Contrast control */

#define REG_GFIX	0x69	/* Fix gain control */

#define REG_DBLV	0x6b	/* PLL control an debugging */
#define   DBLV_BYPASS	  0x0a	  /* Bypass PLL */
#define   DBLV_X4	  0x4a	  /* clock x4 */
#define   DBLV_X6	  0x8a	  /* clock x6 */
#define   DBLV_X8	  0xca	  /* clock x8 */

#define REG_SCALING_XSC	0x70	/* Test pattern and horizontal scale factor */
#define   TEST_PATTTERN_0 0x80
#define REG_SCALING_YSC	0x71	/* Test pattern and vertical scale factor */
#define   TEST_PATTTERN_1 0x80

#define REG_REG76	0x76	/* OV's name */
#define   R76_BLKPCOR	  0x80	  /* Black pixel correction enable */
#define   R76_WHTPCOR	  0x40	  /* White pixel correction enable */

#define REG_RGB444	0x8c	/* RGB 444 control */
#define   R444_ENABLE	  0x02	  /* Turn on RGB444, overrides 5x5 */
#define   R444_RGBX	  0x01	  /* Empty nibble at end */

#define REG_HAECC1	0x9f	/* Hist AEC/AGC control 1 */
#define REG_HAECC2	0xa0	/* Hist AEC/AGC control 2 */

#define REG_BD50MAX	0xa5	/* 50hz banding step limit */
#define REG_HAECC3	0xa6	/* Hist AEC/AGC control 3 */
#define REG_HAECC4	0xa7	/* Hist AEC/AGC control 4 */
#define REG_HAECC5	0xa8	/* Hist AEC/AGC control 5 */
#define REG_HAECC6	0xa9	/* Hist AEC/AGC control 6 */
#define REG_HAECC7	0xaa	/* Hist AEC/AGC control 7 */
#define REG_BD60MAX	0xab	/* 60hz banding step limit */

struct regval_list {
    unsigned char reg_num;
    unsigned char value;
};

static struct regval_list ov7670_rgb565_config[] = {
    /* Gamma curve values */
    { 0x7a, 0x20 },		{ 0x7b, 0x10 },
    { 0x7c, 0x1e },		{ 0x7d, 0x35 },
    { 0x7e, 0x5a },		{ 0x7f, 0x69 },
    { 0x80, 0x76 },		{ 0x81, 0x80 },
    { 0x82, 0x88 },		{ 0x83, 0x8f },
    { 0x84, 0x96 },		{ 0x85, 0xa3 },
    { 0x86, 0xaf },		{ 0x87, 0xc4 },
    { 0x88, 0xd7 },		{ 0x89, 0xe8 },

    /* AGC and AEC parameters.  Note we start by disabling those features,
       then turn them only after tweaking the values. */
    { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT },
    { REG_GAIN, 0 },	{ REG_AECH, 0 },
    { REG_COM4, 0x40 }, /* magic reserved bit */
    { REG_COM9, 0x38 },	/* 16x gain ceiling; 0x8 is reserved bit */
    { REG_BD50MAX, 0x05 },	{ REG_BD60MAX, 0x07 },
    { REG_AEW, 0x95 },	{ REG_AEB, 0x33 },
    { REG_VPT, 0xe3 },	{ REG_HAECC1, 0x78 },
    { REG_HAECC2, 0x68 },	{ 0xa1, 0x03 }, /* magic */
    { REG_HAECC3, 0xd8 },	{ REG_HAECC4, 0xd8 },
    { REG_HAECC5, 0xf0 },	{ REG_HAECC6, 0x90 },
    { REG_HAECC7, 0x94 },
    { REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC },

    /* Almost all of these are magic "reserved" values.  */
    { REG_COM5, 0x61 },	{ REG_COM6, 0x4b },
    { 0x16, 0x02 },		{ REG_MVFP, 0x07 },
    { 0x21, 0x02 },		{ 0x22, 0x91 },
    { 0x29, 0x07 },		{ 0x33, 0x0b },
    { 0x35, 0x0b },		{ 0x37, 0x1d },
    { 0x38, 0x71 },		{ 0x39, 0x2a },
    { REG_COM12, 0x78 },	{ 0x4d, 0x40 },
    { 0x4e, 0x20 },		{ REG_GFIX, 0 },
    { 0x6b, 0x0a },		{ 0x74, 0x10 },
    { 0x8d, 0x4f },		{ 0x8e, 0 },
    { 0x8f, 0 },		{ 0x90, 0 },
    { 0x91, 0 },		{ 0x96, 0 },
    { 0x9a, 0 },		{ 0xb0, 0x84 },
    { 0xb1, 0x0c },		{ 0xb2, 0x0e },
    { 0xb3, 0x82 },		{ 0xb8, 0x0a },

    /* More reserved magic, some of which tweaks white balance */
    { 0x43, 0x0a },		{ 0x44, 0xf0 },
    { 0x45, 0x34 },		{ 0x46, 0x58 },
    { 0x47, 0x28 },		{ 0x48, 0x3a },
    { 0x59, 0x88 },		{ 0x5a, 0x88 },
    { 0x5b, 0x44 },		{ 0x5c, 0x67 },
    { 0x5d, 0x49 },		{ 0x5e, 0x0e },
    { 0x6c, 0x0a },		{ 0x6d, 0x55 },
    { 0x6e, 0x11 },		{ 0x6f, 0x9f }, /* "9e for advance AWB" */
    { 0x6a, 0x40 },		{ REG_BLUE, 0x40 },
    { REG_RED, 0x60 },
    { REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC|COM8_AWB },

    /* Matrix coefficients */
    { 0x58, 0x9e },
    { 0x4f, 0xb3 },		/* "matrix coefficient 1" */
    { 0x50, 0xb3 },		/* "matrix coefficient 2" */
    { 0x51, 0    },		/* vb */
    { 0x52, 0x3d },		/* "matrix coefficient 4" */
    { 0x53, 0xa7 },		/* "matrix coefficient 5" */
    { 0x54, 0xe4 },		/* "matrix coefficient 6" */

    { REG_COM16, COM16_AWBGAIN },	{ REG_EDGE, 0 },
    { 0x75, 0x05 },		{ 0x76, 0xe1 },
    { 0x4c, 0 },		{ 0x77, 0x01 },
    { REG_COM13, 0xc3 },	{ 0x4b, 0x09 },
    { 0xc9, 0x60 },		{ REG_COM16, 0x38 },
    { 0x56, 0x40 },

    { 0x34, 0x11 },		{ REG_COM11, COM11_EXP|COM11_HZAUTO },
    { 0xa4, 0x88 },		{ 0x96, 0 },
    { 0x97, 0x30 },		{ 0x98, 0x20 },
    { 0x99, 0x30 },		{ 0x9a, 0x84 },
    { 0x9b, 0x29 },		{ 0x9c, 0x03 },
    { 0x9d, 0x4c },		{ 0x9e, 0x3f },
    { 0x78, 0x04 },

    /* Extra-weird stuff.  Some sort of multiplexor register */
    { 0x79, 0x01 },		{ 0xc8, 0xf0 },
    { 0x79, 0x0f },		{ 0xc8, 0x00 },
    { 0x79, 0x10 },		{ 0xc8, 0x7e },
    { 0x79, 0x0a },		{ 0xc8, 0x80 },
    { 0x79, 0x0b },		{ 0xc8, 0x01 },
    { 0x79, 0x0c },		{ 0xc8, 0x0f },
    { 0x79, 0x0d },		{ 0xc8, 0x20 },
    { 0x79, 0x09 },		{ 0xc8, 0x80 },
    { 0x79, 0x02 },		{ 0xc8, 0xc0 },
    { 0x79, 0x03 },		{ 0xc8, 0x40 },
    { 0x79, 0x05 },		{ 0xc8, 0x30 },
    { 0x79, 0x26 },

    // rgb565 and qvga
    { REG_COM7, COM7_RGB | COM7_FMT_QVGA },	/* Selects RGB mode and QVGA */
    { REG_RGB444, 0 },	/* No RGB444 please */
    { REG_COM1, 0x0 },	/* CCIR601 */
    { REG_COM15, COM15_RGB565 | COM15_R00FF },
    { REG_COM13, COM13_GAMMA|COM13_UVSAT },
    { 0xff, 0xff },

    { 0xff, 0xff },	/* END MARKER */
};

// ***********************************************************************

// writes to a register of the OV7670
void CAM_reg_write(const uint8_t reg, const uint8_t value)
{
    I2C1_start();
    I2C1_send_addr(OV7670_WRITE_ADDR);
    I2C1_write(reg);
    I2C1_write(value);
    I2C1_stop();
}

// writes multiple data to the registers of the OV7670
// regvals has to end with a {0xFF, 0xFF}, else this will access out of bounds memory
void CAM_reg_write_multiple(const struct regval_list* regvals)
{
    int i = 0;
    while (1)
    {
        const struct regval_list regval = regvals[i++];
        if (regval.reg_num == 0xFF && regval.value == 0xFF)
        {
            break;
        }
        CAM_reg_write(regval.reg_num, regval.value);
        delay_ms(10);
    }
}

// reads from a register of the OV7670
uint8_t CAM_reg_read(const uint8_t reg)
{
    I2C1_start();
    I2C1_send_addr(OV7670_WRITE_ADDR);
    I2C1_write(reg);
    I2C1_stop();
    I2C1_start();
    I2C1_stop_ack();
    I2C1_send_addr(OV7670_READ_ADDR);
    const uint8_t data = I2C1_read();
    I2C1_stop();
    return data;
}

// sets up everything required for the OV7670 camera and configures it for RGB565 colors and QVGA resolution
void CAM_setup()
{
    pin_setup(CAM_RST, PIN_MODE_OUTPUT, PIN_PULL_UP, PIN_SPEED_DEFAULT, PIN_TYPE_PUSHPULL);

    MCO1_setup(HSI, Div1);
    I2C1_setup();

    io_set(CAM_RST, 0);
    delay_ms(10);
    io_set(CAM_RST, 1);
    delay_ms(10);

    DCMI_init();
    OV7670_DCMI_setup();

    CAM_reg_write_multiple(ov7670_rgb565_config);
}

// enables the DCMI's crop functionality
void CAM_mcu_crop_en(const uint16_t x, const uint16_t y, const uint16_t w, const uint16_t h)
{
    DCMI->CWSTRTR = x * 2 & DCMI_CWSTRT_HOFFCNT | y << DCMI_CWSTRT_VST_Pos & DCMI_CWSTRT_VST;
    DCMI->CWSIZER = w * 2    - 1 & DCMI_CWSIZE_CAPCNT | h - 1 << DCMI_CWSIZE_VLINE_Pos & DCMI_CWSIZE_VLINE;
    DCMI->CR |= DCMI_CR_CROP;
}

// disables the DCMI's crop functionality
void CAM_mcu_crop_off()
{
    DCMI->CR &= ~DCMI_CR_CROP;
}

// sets up the DMA to transfer data from the DCMI to memory
// buff2 is optional - set to 0 for a single buffer; else double buffering
// buffSize should be even
// buffers should be word-aligned, else the image could be shifted
void CAM_DMA_setup(uint16_t* buff1, uint16_t* buff2, const uint16_t buffSize)
{
    DMA2_init();

    // DCMI dma request is mapped to DMA 2 stream 7 (and 1)
    // set the peripheral address
    // DR has an offset of 0x28
    // set the memory addresses
    // config the number of item transfers
    DMA_setup_addr(DMA2_Stream7, DCMI_BASE + 0x28, (uint32_t)buff1, (uint32_t)buff2, buffSize / 2);

    // DCMI dma requests occur on channel 1
    // set the direction to periph -> mem
    // potentially enable double buffer mode (also automatically circular)
    DMA_setup_behav(DMA2_Stream7, 1, PerToMem, 1, buff2, 0);

    // increment memory pointer after each transfer
    // set the memory widths to 32b for the peripheral and 16b for the memory
    DMA_setup_data(DMA2_Stream7, 0, 1, Word, HalfWord, 0);

    // enable the dma
    DMA_enable(DMA2_Stream7);
}

// resets the memory pointer to the initial value
// use when using buffers larger than the scanned image or when the pointer somehow breaks
void CAM_DMA_ptr_reset()
{
    DMA_disable(DMA2_Stream7);
    DMA_enable(DMA2_Stream7);
}

// captures a single frame from the camera
void CAM_snapshot()
{
    DCMI->CR |= DCMI_CR_CM;
    DCMI->CR |= DCMI_CR_CAPTURE;
}

// starts continuous captures
void CAM_continuous_start()
{
    DCMI->CR &= ~DCMI_CR_CM;
    DCMI->CR |= DCMI_CR_CAPTURE;
}

// stops continuous capturing after the last frame transfer
void CAM_continuous_stop()
{
    DCMI->CR &= ~DCMI_CR_CAPTURE;
}

// busily waits for the image transfer to end
// will get stuck when called after continuous_start; you have to call continuous_stop first
void CAM_wait_for_capture_end()
{
    while (DCMI->CR & DCMI_CR_CAPTURE){}
}

// returns what buffer is written to at the moment
uint8_t CAM_DMA_get_ct()
{
    return DMA_get_ct(DMA2_Stream7);
}

#endif //STM32_KIT_CAM
