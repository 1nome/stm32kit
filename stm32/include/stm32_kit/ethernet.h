/**
 * @file ethernet.h
 *
 * @author 1nome
 *
 * @brief LAN8720A PHY driver.
 * @note System core clock has to be at least 25 MHz.
 */

#ifndef STM32_KIT_ETHERNET
#define STM32_KIT_ETHERNET

#define PHY_SMI_ADDR 0x01

// LAN8720A registers, see datasheet for more info
#define PHY_BCR 0
#define PHY_BSR 1
#define PHY_IDR1 2
#define PHY_IDR2 3
#define PHY_ANAR 4
#define PHY_ANLPAR 5
#define PHY_ANER 6
#define PHY_MCSR 17
#define PHY_SMR 18
#define PHY_SECR 26
#define PHY_CSIR 27
#define PHY_ISR 29
#define PHY_IMR 30
#define PHY_SCSR 31


#include "pin.h"

uint32_t SMI_get_div()
{
    if (SystemCoreClock < 35000000)
    {
        return ETH_MACMIIAR_CR_Div16;
    }
    if (SystemCoreClock < 60000000)
    {
        return ETH_MACMIIAR_CR_Div26;
    }
    if (SystemCoreClock < 100000000)
    {
        return ETH_MACMIIAR_CR_Div42;
    }
    if (SystemCoreClock < 150000000)
    {
        return ETH_MACMIIAR_CR_Div62;
    }
    return ETH_MACMIIAR_CR_Div102;
}

void ETH_SMI_set_address(const uint32_t addr)
{
    ETH->MACMIIAR &= ~ETH_MACMIIAR_PA_Msk;
    ETH->MACMIIAR |= addr << ETH_MACMIIAR_PA_Pos & ETH_MACMIIAR_PA_Msk;
}

/* Busy waiting functions for interfacing with the PHY */

void ETH_SMI_write(const uint8_t reg, const uint16_t data)
{
    ETH->MACMIIAR &= ~(ETH_MACMIIAR_MR_Msk | ETH_MACMIIAR_MW);
    ETH->MACMIIAR |= reg << ETH_MACMIIAR_MR_Pos & ETH_MACMIIAR_MR_Msk |
        ETH_MACMIIAR_MW;
    ETH->MACMIIDR = data;

    ETH->MACMIIAR |= ETH_MACMIIAR_MB;
    while (ETH->MACMIIAR & ETH_MACMIIAR_MB){}
}

uint16_t ETH_SMI_read(const uint8_t reg)
{
    ETH->MACMIIAR &= ~(ETH_MACMIIAR_MR_Msk | ETH_MACMIIAR_MW);
    ETH->MACMIIAR |= reg << ETH_MACMIIAR_MR_Pos & ETH_MACMIIAR_MR_Msk;

    ETH->MACMIIAR |= ETH_MACMIIAR_MB;
    while (ETH->MACMIIAR & ETH_MACMIIAR_MB){}

    return ETH->MACMIIDR & ETH_MACMIIDR_MD_Msk;
}

void ETH_setup()
{
    pin_setup_af(ETH_RMII_REF_CLK, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF11);
    pin_setup_af(ETH_RMII_CRS_DV, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF11);
    pin_setup_af(ETH_RMII_RXD0, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF11);
    pin_setup_af(ETH_RMII_RXD1, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF11);
    pin_setup_af(ETH_RMII_TX_EN, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF11);
    pin_setup_af(ETH_RMII_TXD0, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF11);
    pin_setup_af(ETH_RMII_TXD1, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF11);
    pin_setup_af(ETH_MDC, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_MID, PIN_TYPE_PUSHPULL, PIN_AF11);
    pin_setup_af(ETH_MDIO, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_MID, PIN_TYPE_PUSHPULL, PIN_AF11);

    RCC->AHB1ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->PMC |= SYSCFG_PMC_MII_RMII;

    RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN | RCC_AHB1ENR_ETHMACRXEN | RCC_AHB1ENR_ETHMACTXEN;
    ETH->MACMIIAR |= SMI_get_div();
    ETH_SMI_set_address(PHY_SMI_ADDR);

    // phy init
    // soft reset
    ETH_SMI_write(PHY_BCR, 0x8000);
    delay_ms(500);

    // wait for the link to go up
    while (!(ETH_SMI_read(PHY_BSR) & 0x0004)){}
}


#endif //STM32_KIT_ETHERNET
