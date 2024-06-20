/**
 * @file	i2c_lcd.h
 * @author	1nome (Adam Wiszczor)
 * 
 * @brief	Driver for the HD44780 over i2c
 *			Here, it is connected to I2C3 using PC9 and PA8
 *			(Used to be I2C1 using PB7 and PB6)
 */

#ifndef STM32_KIT_I2C_LCD
#define STM32_KIT_I2C_LCD

#include "lcd.h"
#include "i2c.h"

uint8_t I2C_LCD_addr = 0;

void I2C_LCD_busy(){
	delay_us(20);
}


// user funcs     VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV

// Posle cmd LCD displeji
// Sends a command to the LCD
void I2C_LCD_set(uint8_t cmd){
	if(!I2C_LCD_addr){
		return;
	}
	I2C_LCD_busy();
	I2C3_start();
	//io_set(LED_IN_2, 1);	// status
	
	I2C3_send_addr(I2C_LCD_addr);
	I2C3_write(0x00);		// inst reg.
	
	I2C3_write((cmd & 0xF0) | 0x0C);
	delay_us(10);
	I2C3_write((cmd & 0xF0) | 0x08);
	
	I2C3_write((cmd << 4) | 0x0C);
	delay_us(10);
	I2C3_write((cmd << 4) | 0x08);
	
	I2C3_stop();
	//io_set(LED_IN_2, 0);	// status
}

void I2C_LCD_set_fast(uint8_t cmd){
	I2C3_write((cmd & 0xF0) | 0x0C);
	//delay_us(1);
	I2C3_write((cmd & 0xF0) | 0x08);
	
	I2C3_write((cmd << 4) | 0x0C);
	//delay_us(1);
	I2C3_write((cmd << 4) | 0x08);
}

// Vypise znak na LCD
// Prints out a character on the LCD
void I2C_LCD_symbol(uint8_t data){
	if(!I2C_LCD_addr){
		return;
	}
	I2C_LCD_busy();
	I2C3_start();
	//io_set(LED_IN_2, 1);	// status
	
	I2C3_send_addr(I2C_LCD_addr);
	I2C3_write(0x40);		// data reg.
	
	I2C3_write((data & 0xF0) | 0x0D);
	delay_us(10);
	I2C3_write((data & 0xF0) | 0x09);
	
	I2C3_write((data << 4) | 0x0D);
	delay_us(10);
	I2C3_write((data << 4) | 0x09);
	
	I2C3_stop();
	//io_set(LED_IN_2, 0);	// status
}

void I2C_LCD_symbol_fast(uint8_t cmd){
	I2C3_write((cmd & 0xF0) | 0x0D);
	//delay_us(1);
	I2C3_write((cmd & 0xF0) | 0x09);
	
	I2C3_write((cmd << 4) | 0x0D);
	//delay_us(1);
	I2C3_write((cmd << 4) | 0x09);
}

// Nastavi a popr. najde LCD
// Sets up and finds the LCD (if needed)
void I2C_LCD_setup(){
	I2C3_setup();
	if(!I2C_LCD_addr){
		I2C3_find_device(&I2C_LCD_addr);
	}
	if(!I2C_LCD_addr){
		return;
	}
	
	I2C_LCD_set(0x3);
	I2C_LCD_set(0x3);
	I2C_LCD_set(0x2);
	
	I2C_LCD_set(0x28);
	I2C_LCD_set(0x0F);
	I2C_LCD_set(0x06);
	I2C_LCD_set(0x01);
}

void I2C_LCD_setup_fast(){
	I2C3_setup();
	if(!I2C_LCD_addr){
		I2C3_find_device(&I2C_LCD_addr);
	}
	if(!I2C_LCD_addr){
		return;
	}
	
	I2C3_start();
	I2C3_send_addr(I2C_LCD_addr);
	
	I2C_LCD_set_fast(0x3);
	I2C_LCD_set_fast(0x3);
	I2C_LCD_set_fast(0x2);
	
	I2C_LCD_set_fast(0x28);
	I2C_LCD_set_fast(0x0F);
	I2C_LCD_set_fast(0x06);
	I2C_LCD_set_fast(0x01);
	delay_us(10);
}

// Vypise adresu LCD
// Uziti: nastaveni adresy aby se nemuselo LCD hledat
// LCD prints its own address
// Usage: set the address value in code, so it doesn't have to be found every time
void I2C_LCD_print_own_addr(){
	I2C_LCD_symbol(I2C_LCD_addr / 100);
	I2C_LCD_symbol(I2C_LCD_addr / 10 % 10);
	I2C_LCD_symbol(I2C_LCD_addr % 10);
}

// Vypise text na LCD
// Prints out text on the LCD
void I2C_LCD_print(const char *__restrict__ text){
	uint8_t i;
	
	for (i = 0; i < strlen(text); i++) {
		I2C_LCD_symbol(text[i]);
	}
}

void I2C_LCD_print_fast(const char *__restrict__ text){
	int len = strlen(text);
	
	for(int i = 0; i < len; i++){
		I2C_LCD_symbol_fast(text[i]);
	}
}

// user funcs     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#endif /* STM32_KIT_I2C_LCD */
