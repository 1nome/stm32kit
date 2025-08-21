/**
 * @file usb.h
 * @author 1nome
 *
 * @brief Driver for the on-board USB FS PHY.
 *
 * @attention Written for the f407-disco board.
 */

#ifndef STM32_KIT_USB
#define STM32_KIT_USB

#include "disc/f407.h"

uint32_t _trdt()
{
    if (SystemCoreClock < 32000000)
    {
        return USB_OTG_GUSBCFG_TRDT_Msk; // 0xf
    }
    return USB_OTG_GUSBCFG_TRDT_2 | USB_OTG_GUSBCFG_TRDT_1; // 6
}

uint8_t _mode;

void usb_core_init()
{
    USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT; // unmask global int
    USB_OTG_FS->GAHBCFG &= ~USB_OTG_GAHBCFG_PTXFELVL; // gen int @ half-fill level
    USB_OTG_FS->GAHBCFG &= ~USB_OTG_GAHBCFG_TXFELVL; // gen int @ half-fill level

    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_HNPCAP; // hnp on
    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_SRPCAP; // srp on
    USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT_Msk;
    USB_OTG_FS->GUSBCFG |= _trdt(); // turnaround time
    USB_OTG_FS->GUSBCFG |= 2; // timeout calibration (didn't test, think 2 is neat)

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_OTGINT; // unmask OTG int
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_MMISM; // unmask mode mismatch int

    _mode = USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD; // what mode are we in
}

#endif //STM32_KIT_USB
