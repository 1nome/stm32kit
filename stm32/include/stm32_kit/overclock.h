/**
 * @file	overclock.h
 * @author	1nome (Adam Wiszczor)
 * 
 * @brief	Functions setting system clocks
 *			Do not call SystemCoreClockUpdate as it incorrectly uses a 25Mhz external oscilator value
 */
 
#ifndef STM32_KIT_OVERCLOCK
#define STM32_KIT_OVERCLOCK

#include "stm32_kit.h"
#define HSE_VALUE 8000000
#define PLLM_168 8
#define PLLN_168 336
#define PLLP_168 2
#define PLLQ_168 7

// core 168M, ahb 168M, apb high 84M, apb low 42M, usb 48M
void clock168Mhz(){
	
	// zapnuti ext osc.
	// turn on external oscilator
	RCC->CR |= RCC_CR_HSEON;
	// cekani na nej
	while(!(RCC->CR & RCC_CR_HSERDY));
	
	// zapnuti clocku na power interface
	// enable power interface
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	// power scale 1 (> 144Mhz)
	// set the power scale for high frequencies
	PWR->CR |= PWR_CR_VOS;
	
	// ahb prescaler 1
	RCC->CFGR &= ~RCC_CFGR_HPRE;
	// apb low prescaler 4
	RCC->CFGR |= RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_0;
	// apb high prescaler 2
	RCC->CFGR |= RCC_CFGR_PPRE2_2;

	// nastaveni parametru pll
	// setting the pll parameters
	RCC->PLLCFGR =					PLLM_168
								| (PLLN_168 << 6)
								| (((PLLP_168 >> 1) -1) << 16)
								| (PLLQ_168 << 24)
								| RCC_PLLCFGR_PLLSRC_HSE;
	
	// zapnuti pll
	// turning on the pll
	RCC->CR |= RCC_CR_PLLON;
	// cekani na nej
	while(!(RCC->CR & RCC_CR_PLLRDY));

	// nastaveni parametru flash radice
	// setting the flash controller parameters
	FLASH->ACR =  FLASH_ACR_PRFTEN
							| FLASH_ACR_ICEN
							| FLASH_ACR_DCEN
							| 5;							// 5 wait states
	
	// nastaveni pll jako zdroje clocku
	// set pll as the clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	// cekani na nej
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));
	
	SystemCoreClock = 168000000;
}

#endif /* STM32_KIT_OVERCLOCK */
