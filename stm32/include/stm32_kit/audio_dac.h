/**
 * @file	audio_dac.h
 * @author	1nome (Adam Wiszczor)
 * 
 * @brief	Driver for the onboard CS43L22 stereo DAC over i2c and i2s
 *			Here, it is connected to I2C1 using PB9 and PB6
 *			It recieves audio data over I2S3 using PC12, PC10, PC7 and PA4
 *			It is also reset by setting PD4 low
 */
 
#ifndef STM32_KIT_AUDIO_DAC
#define STM32_KIT_AUDIO_DAC

#include "i2c.h"

#define CS43L22_ADDRESS 0x94

// defines for CS43L22 registers, see the datasheet for more info
#define CS43L22_CHIPIDREV	0x01
#define CS43L22_PWRCTL1		0x02
#define CS43L22_PWRCTL2		0x04
#define CS43L22_CLKCTL		0x05
#define CS43L22_IFCTL1		0x06
#define CS43L22_IFCTL2		0x07
#define CS43L22_PASSASEL	0x08
#define CS43L22_PASSBSEL	0x09
#define CS43L22_ANLGZCSRS	0x0A
#define CS43L22_PASSGANGC	0x0C
#define CS43L22_PBCTL1		0x0D
#define CS43L22_MISCCTL		0x0E
#define CS43L22_PBCTL2		0x0F
#define CS43L22_PASSAVOL	0x14
#define CS43L22_PASSBVOL	0x15
#define CS43L22_PCMAVOL		0x1A
#define CS43L22_PCMBVOL		0x1B
#define CS43L22_BFREQONT	0x1C
#define CS43L22_BVOLOFFT	0x1D
#define CS43L22_BTCFG		0x1E
#define CS43L22_TCTL		0x1F
#define CS43L22_MSTVOLCTLA	0x20
#define CS43L22_MSTVOLCTLB	0x21
#define CS43L22_HPVOLCTLA	0x22
#define CS43L22_HPVOLCTLB	0x23
#define CS43L22_SPKVOLCTLA	0x24
#define CS43L22_SPKVOLCTLB	0x25
#define CS43L22_PCMCHSWP	0x26
#define CS43L22_LIMCTL1		0x27
#define CS43L22_LIMCTL2		0x28
#define CS43L22_LIMARATE	0x29
#define CS43L22_STATUS		0x2E
#define CS43L22_BATTCOMP	0x2F
#define CS43L22_VPBATTLVL	0x30
#define CS43L22_SPKSTATUS	0x31
#define CS43L22_CHGPMPFREQ	0x34

#define PLLI2SN_86 258
#define PLLI2SR_86 3

#define AUDIO_DAC_MAX 8388607

// setup i2s3 in master mode with 12.288Mhz (12.2857 actual) master clock
void I2S3_setup(){
	__disable_irq();
	
	// setting the i2s pll parameters
	// 86Mhz output
	RCC->PLLI2SCFGR =	(PLLI2SN_86 << 6)
						| (PLLI2SR_86 << 28);
	
	// turning the pll on and waiting for its ready flag
	RCC->CR |= RCC_CR_PLLI2SON;
	while(!(RCC->CR & RCC_CR_PLLI2SRDY));
	
	pin_setup_af(I2S3_MCK, PIN_MODE_AF, PIN_PULL_DEFAULT, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF6);
	pin_setup_af(I2S3_CK, PIN_MODE_AF, PIN_PULL_DEFAULT, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF6);
	pin_setup_af(I2S3_WS, PIN_MODE_AF, PIN_PULL_DEFAULT, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF6);
	pin_setup_af(I2S3_SD, PIN_MODE_AF, PIN_PULL_DEFAULT, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF6);
	
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
	
	// real divider value is 7
	// we are also outputting a master clock
	SPI3->I2SPR = 	3
					| SPI_I2SPR_ODD
					| SPI_I2SPR_MCKOE;
	
	// 24-bit data length
	// master transmitter mode
	// philips i2s standard
	SPI3->I2SCFGR = SPI_I2SCFGR_I2SMOD
					| SPI_I2SCFGR_DATLEN_0
					| SPI_I2SCFGR_I2SCFG_1;
					
	// enable dma requests on empty tx buffer
	SPI3->CR2 |= SPI_CR2_TXDMAEN;
	
	SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE;
	
	__enable_irq();
}

// busy waiting (bad)
void I2S3_transmit(uint32_t value){
	while(!(SPI3->SR & SPI_SR_TXE));
	SPI3->DR = (uint16_t)(value >> 8);
	while(!(SPI3->SR & SPI_SR_TXE));
	SPI3->DR = (uint16_t)(value << 8);
}

// use this to check if data is needed, else do something more useful than waiting
uint8_t I2S3_can_transmit(){
	return SPI3->DR & SPI_SR_TXE ? 1 : 0;
}

// false = left
uint8_t I2S3_channel_side(){
	return SPI3->DR & SPI_SR_CHSIDE ? 1 : 0;
}

//todo: move dma config to another file

// transmit using the dma, dual buffer mode for easy data generation
// using direct mode
// the mcu uses little endian but i2s transfers msb first,
// the input data then has to be formatted correctly (use the format function)
// buffSize should be even as there are two channels (left, right)
// it also should be 32766 max (32767 possible but that would put a sample's channels in different buffers)
void I2S3_transmit_dma_start(int32_t* buff1, int32_t* buff2, uint16_t buffSize){
	// enable clock to dma1 if not already enabled
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	
	// confirm that stream 5 is disabled
	DMA1_Stream5->CR &= ~DMA_SxCR_EN;
	while(DMA1_Stream5->CR & DMA_SxCR_EN);
	
	// I2S3 (SPI3) dma request is mapped to DMA 1 stream 5
	// set the peripheral address
	// DR has an offset of 0x0C
	DMA1_Stream5->PAR = SPI3_BASE + 0x0C;
	
	// set the memory addresses
	DMA1_Stream5->M0AR = (uint32_t)buff1;
	DMA1_Stream5->M1AR = (uint32_t)buff2;
	
	// config the number of item transfers
	DMA1_Stream5->NDTR = buffSize * 2;
	
	// I2S3 (SPI3) dma requests occur on channel 0 (no need to set)
	// set the direction to mem -> periph
	// increment memory pointer after each transfer
	// set peripheral and memory widths to 16b
	// enable double buffer mode (also automatically circular)
	DMA1_Stream5->CR =	DMA_SxCR_DIR_0
						| DMA_SxCR_MINC
						| DMA_SxCR_PSIZE_0
						| DMA_SxCR_MSIZE_0
						| DMA_SxCR_DBM;
						
	// enable the dma
	DMA1_Stream5->CR |= DMA_SxCR_EN;
}

// dec value is B2 B1 B0, sent value is the same
// however, it is written as B1 B0 and B3 B2, B2 will be ignored
// therefore	B1 = B2
//				B0 = B1
//				B3 = B0
// BEWARE: two's (or one's, idk) complement is used, therefore:
// 8388607 = max, 8388608 = min and -8388608 = min, -8388609 = max
void I2S_dma_formatData(int32_t* value){
	*value = *value >> 8 | *value << 24;
}

void I2S3_transmit_dma_stop(){
	DMA1_Stream5->CR &= ~DMA_SxCR_EN;
	while(DMA1_Stream5->CR & DMA_SxCR_EN);
}

uint8_t I2S3_dma_current_target(){
	return DMA1_Stream5->CR & DMA_SxCR_CT ? 1 : 0;
}

void audio_dac_reg_write(uint8_t addr, uint8_t data){
	I2C1_start();
	I2C1_send_addr(CS43L22_ADDRESS);
	
	I2C1_write(addr);
	I2C1_write(data);
	
	I2C1_stop();
}

//todo:
uint8_t audio_dac_reg_read(uint8_t addr){
	
}

void audio_dac_setup(){
	__disable_irq();
	
	pin_setup(AUDIO_DAC_RESET, PIN_MODE_OUTPUT, PIN_PULL_DOWN, PIN_SPEED_DEFAULT, PIN_TYPE_PUSHPULL);
	
	__enable_irq();
	
	I2C1_setup();
	
	io_set(AUDIO_DAC_RESET, 0); // pin should be low by default, so this shouldn't be necessary
	delay_ms(10); // an arbitrary value to ensure reset
	io_set(AUDIO_DAC_RESET, 1);
	
	// settings
	audio_dac_reg_write(CS43L22_PWRCTL2, 0xAF); // speakers off, headphones on
	audio_dac_reg_write(CS43L22_CLKCTL, 0x20); // single speed, everything else 0
	audio_dac_reg_write(CS43L22_IFCTL1, 0x04); // i2s format, else 0
	
	// initialization sequence
	audio_dac_reg_write(0x00, 0x99);
	audio_dac_reg_write(0x47, 0x80);
	audio_dac_reg_write(0x32, 0x80);
	audio_dac_reg_write(0x32, 0x80);
	audio_dac_reg_write(0x00, 0x00);
	
	I2S3_setup();
	
	audio_dac_reg_write(CS43L22_PWRCTL1, 0x9E);
}

void audio_dac_power_down(){
	// mute
	audio_dac_reg_write(CS43L22_HPVOLCTLA, 0x01);
	audio_dac_reg_write(CS43L22_HPVOLCTLB, 0x01);
	
	audio_dac_reg_write(CS43L22_MISCCTL, 0x00); // disable soft ramp and zero cross
	audio_dac_reg_write(CS43L22_ANLGZCSRS, 0x00); // same for analog
	
	audio_dac_reg_write(CS43L22_PWRCTL1, 0x9F);
	delay_ms(1);
	
	io_set(AUDIO_DAC_RESET, 0);
}

void audio_dac_volume(uint8_t vol){
	audio_dac_reg_write(CS43L22_HPVOLCTLA, vol + 1);
	audio_dac_reg_write(CS43L22_HPVOLCTLB, vol + 1);
}

#endif /* STM32_KIT_AUDIO_DAC */
