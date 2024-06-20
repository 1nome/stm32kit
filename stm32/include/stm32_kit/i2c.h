/**
 * @file	i2c.h
 * @author	1nome (Adam Wiszczor)
 * 
 * @brief	Driver for i2c, uses busy waiting and no dma
 */
 
#ifndef STM32_KIT_I2C
#define STM32_KIT_I2C
 
#include "pin.h"
#include "boards.h"
 
#define I2C1_FREQ 100000
#define I2C1_DUTY 0.5
#define I2C1_RISE 0.000001

#define I2C3_FREQ 100000
#define I2C3_DUTY 0.5
#define I2C3_RISE 0.000001

uint8_t clear_byte;

void I2C1_start(){
	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	//io_set(LED_IN_0, 1);	// status
}

void I2C1_write(uint8_t data){
	while(!(I2C1->SR1 & I2C_SR1_TXE));
	//io_set(LED_IN_1, 1);	// status
	I2C1->DR = data;
	while(!(I2C1->SR1 & I2C_SR1_BTF));
	//io_set(LED_IN_1, 0);	// status
}

void I2C1_send_addr(uint8_t addr){
	I2C1->DR = addr;
	//io_set(LED_IN_1, 1);	// status
	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	clear_byte = (I2C1->SR1 | I2C1->SR2);
	//io_set(LED_IN_1, 0);	// status
}

void I2C1_stop(){
	I2C1->CR1 |= I2C_CR1_STOP;
	//io_set(LED_IN_0, 0);	// status
}

void I2C1_setup(){
	int clock = SystemCoreClock;
	//io_set(LED_IN_3, 1);	// status
	__disable_irq();
	
	pin_setup_af(I2C1_SCL, PIN_MODE_AF, PIN_PULL_UP, PIN_SPEED_VERYHIGH, PIN_TYPE_OPENDRAIN, PIN_AF4);
	pin_setup_af(I2C1_SDA, PIN_MODE_AF, PIN_PULL_UP, PIN_SPEED_VERYHIGH, PIN_TYPE_OPENDRAIN, PIN_AF4);
	
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;
	
	// the default 16Mhz clock is fine for slow apb peripherals
	// however, faster clocks have to be divided to be used on the apb
	if(SystemCoreClock > 84000000){
		clock /= 4;
	}
	else if(SystemCoreClock > 42000000){
		clock /= 2;
	}
	
	I2C1->CR2 |= (clock / 1000000) & I2C_CR2_FREQ;
	I2C1->CCR |= (uint16_t)((1.0/I2C1_FREQ * I2C1_DUTY) / (1.0 / clock)) & I2C_CCR_CCR;
	I2C1->TRISE |= (uint16_t)(I2C1_RISE / (1.0 / clock)) & I2C_TRISE_TRISE;
	
	I2C1->CR1 |= I2C_CR1_PE;
	
	__enable_irq();
	//io_set(LED_IN_3, 0);	// status
}

void I2C1_find_device(uint8_t *res){
	uint8_t addr = 2;
	while(!(*res)){
		if(!addr){
			break;
		}
		I2C1_start();
		//io_set(LED_IN_3, 1);	// status
		
		I2C1->DR = addr;
		delay_us(100);
		if(!(I2C1->SR1 & I2C_SR1_ADDR)){
			//io_set(LED_IN_3, 0);	// status
			addr += 2;
			clear_byte = (I2C1->SR1 | I2C1->SR2);
			continue;
		}
		
		*res = addr;
		clear_byte = (I2C1->SR1 | I2C1->SR2);
		I2C1_stop();
	}
}

void I2C3_start(){
	I2C3->CR1 |= I2C_CR1_ACK;
	I2C3->CR1 |= I2C_CR1_START;
	while(!(I2C3->SR1 & I2C_SR1_SB));
	//io_set(LED_IN_0, 1);	// status
}

void I2C3_write(uint8_t data){
	while(!(I2C3->SR1 & I2C_SR1_TXE));
	//io_set(LED_IN_1, 1);	// status
	I2C3->DR = data;
	while(!(I2C3->SR1 & I2C_SR1_BTF));
	//io_set(LED_IN_1, 0);	// status
}

void I2C3_send_addr(uint8_t addr){
	I2C3->DR = addr;
	//io_set(LED_IN_1, 1);	// status
	while(!(I2C3->SR1 & I2C_SR1_ADDR));
	clear_byte = (I2C3->SR1 | I2C3->SR2);
	//io_set(LED_IN_1, 0);	// status
}

void I2C3_stop(){
	I2C3->CR1 |= I2C_CR1_STOP;
	//io_set(LED_IN_0, 0);	// status
}

void I2C3_setup(){
	int clock = SystemCoreClock;
	//io_set(LED_IN_3, 1);	// status
	__disable_irq();
	
	pin_setup_af(I2C3_SCL, PIN_MODE_AF, PIN_PULL_UP, PIN_SPEED_VERYHIGH, PIN_TYPE_OPENDRAIN, PIN_AF4);
	pin_setup_af(I2C3_SDA, PIN_MODE_AF, PIN_PULL_UP, PIN_SPEED_VERYHIGH, PIN_TYPE_OPENDRAIN, PIN_AF4);
	
	RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
	
	I2C3->CR1 |= I2C_CR1_SWRST;
	I2C3->CR1 &= ~I2C_CR1_SWRST;
	
	// the default 16Mhz clock is fine for slow apb peripherals
	// however, faster clocks have to be divided to be used on the apb
	if(SystemCoreClock > 84000000){
		clock /= 4;
	}
	else if(SystemCoreClock > 42000000){
		clock /= 2;
	}
	
	I2C3->CR2 |= (clock / 1000000) & I2C_CR2_FREQ;
	I2C3->CCR |= (uint16_t)((1.0/I2C3_FREQ * I2C3_DUTY) / (1.0 / clock)) & I2C_CCR_CCR;
	I2C3->TRISE |= (uint16_t)(I2C3_RISE / (1.0 / clock)) & I2C_TRISE_TRISE;
	
	I2C3->CR1 |= I2C_CR1_PE;
	
	__enable_irq();
	//io_set(LED_IN_3, 0);	// status
}

void I2C3_find_device(uint8_t *res){
	uint8_t addr = 2;
	while(!(*res)){
		if(!addr){
			break;
		}
		I2C3_start();
		//io_set(LED_IN_3, 1);	// status
		
		I2C3->DR = addr;
		delay_us(100);
		if(!(I2C3->SR1 & I2C_SR1_ADDR)){
			//io_set(LED_IN_3, 0);	// status
			addr += 2;
			clear_byte = (I2C3->SR1 | I2C3->SR2);
			continue;
		}
		
		*res = addr;
		clear_byte = (I2C3->SR1 | I2C3->SR2);
		I2C3_stop();
	}
}

#endif /* STM32_KIT_I2C */
