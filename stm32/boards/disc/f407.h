/**
 * @file       f407.h
 * @brief      Definice pinu a jejich funkci na desce DISCOVERY F407 (skolni kit).
 *
 * @author     Petr Madecki (petr.madecki@spsehavirov.cz)
 * @author     Tomas Michalek (tomas.michalek@spsehavirov.cz)
 *
 *  STM32NUCLEO-G071RB (STM32G071RBTx)
 *      Kod pro makro STM32_TYPE:
 *                  407
 *      CLK:
 *                  16MHz   HSI
 *      BTN:
 *                  B1      PA0     Blue      (Pokud neni sepnute tlacitko je na pinu log. 0)
 *      Vestavene LED:                                            (LED sviti v log. 1!)
 *                  LD3     PD13    Yellow
 *                  LD4     PD12    Green
 *                  LD5     PD14    Red
 *                  LD6     PD15    Blue
 *      Externi LED:                                              (LED sviti v log. 0!)
 *      (nepajive pole)
 *                  PE12    Red
 *                  PE13    Red
 *                  PE14    Red
 *                  PE15    Red
 *      LCD:
 *                  RS      PE3
 *                  R/W     PE4   (mozno zapojit primo na GND - omezeni prace s LCD)
 *                  E       PE5
 *                  DB4     PE6
 *                  DB5     PE7
 *                  DB6     PE8
 *                  DB7     PE9
 *      KeyPad:
 *                  COL0    PD0
 *                  COL1    PD1
 *                  COL2    PD2
 *                  COL3    PD3 (skolni pripravek ma keypad pouze 3x4, domaci pripravky maji keypad 4x4)
 *
 *                  ROW0    PD6
 *                  ROW1    PD7
 *                  ROW2    PD8
 *                  ROW3    PD9
 *
 * @date       2023-03-29
 * @copyright  Copyright SPSE Havirov (c) 2023
 */

#ifndef STM32_KIT_BOARDS_F407
#define STM32_KIT_BOARDS_F407

#include "stm32_kit/gpio.h"
#include "config.h"

/* LED setup */
#   define LED_IN_0     (PD12) // Vestavene LED
#   define LED_IN_1     (PD13)
#   define LED_IN_2     (PD14)
#   define LED_IN_3     (PD15)
#   define LED_EX_0     (PE12) // Externi LED
#   define LED_EX_1     (PE13)
#   define LED_EX_2     (PE14)
#   define LED_EX_3     (PE15)

/* BTN setup */
#   define USER_BUTTON  (PA0)  // Uzivatelske tlacitko pro: F401, F411, G071, L152

/* LCD Screen setup */
#   define LCD_RS       (PE3)
#   define LCD_RW       (PE4)
#   define LCD_EN       (PE5)
#   define LCD_DB4      (PE6)
#   define LCD_DB5      (PE7)
#   define LCD_DB6      (PE8)
#   define LCD_DB7      (PE9)

/* Keypad setup */
#   define KEYPAD_C0    (PD0)
#   define KEYPAD_C1    (PD1)
#   define KEYPAD_C2    (PD2)
#   define KEYPAD_C3    (PD3)
#   define KEYPAD_R0    (PD6)
#   define KEYPAD_R1    (PD7)
#   define KEYPAD_R2    (PD8)
#   define KEYPAD_R3    (PD9)

/* ADC setup */
#   define ADC_1        (PA1)

/* UART setup */
#if UART_ALT == 0
#   define UART_TX      (PA2)
#   define UART_RX      (PA3)
#else
#	define UART_TX	    (PD8)
#	define UART_RX  	(PD9)
#endif

/* I2C setup */
#   define I2C1_SDA     (PB9)
#if I2C_ALT1 == 1
#   define I2C1_SCL     (PB6)
#else
#   define I2C1_SCL    (PB8)
#endif

/* I2S setup */
#	define I2S3_WS		(PA4)
#	define I2S3_MCK		(PC7)
#	define I2S3_CK		(PC10)
#	define I2S3_SD		(PC12)

/* audio dac setup */
#	define AUDIO_DAC_RESET	(PD4)

/* DCMI setup */
#define DCMI_D0     (PC6)
#define DCMI_D1     (PC7)
#define DCMI_D2     (PC8)
#define DCMI_D3     (PC9)
#define DCMI_D4     (PC11)
#define DCMI_D5     (PB6)
#define DCMI_D6     (PE5)
#define DCMI_D7     (PE6)
#define DCMI_HSYNC  (PA4)
#define DCMI_VSYNC  (PB7)
#define DCMI_PIXCLK (PA6)

/* MCO setup */
#define MCO1 (PA8)
#define MCO2 (PC9)

/* cam setup */
#define CAM_RST     (PC12)

#endif /* STM32_KIT_BOARDS_F407 */
