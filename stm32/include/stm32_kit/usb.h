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

#include "chrono.h"
#include "disc/f407.h"

uint32_t calc_trdt()
{
    if (SystemCoreClock < 32000000)
    {
        return USB_OTG_GUSBCFG_TRDT; // 0xf
    }
    return USB_OTG_GUSBCFG_TRDT_2 | USB_OTG_GUSBCFG_TRDT_1; // 6
}

uint8_t usb_mode;

void USB_core_init()
{
    USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT; // unmask global int
    USB_OTG_FS->GAHBCFG &= ~USB_OTG_GAHBCFG_PTXFELVL; // gen int @ half-fill level
    USB_OTG_FS->GAHBCFG &= ~USB_OTG_GAHBCFG_TXFELVL; // gen int @ half-fill level

    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_HNPCAP; // hnp on
    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_SRPCAP; // srp on
    USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
    USB_OTG_FS->GUSBCFG |= calc_trdt(); // turnaround time
    USB_OTG_FS->GUSBCFG |= 2; // timeout calibration (didn't test, think 2 is neat)

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_OTGINT; // unmask OTG int
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_MMISM; // unmask mode mismatch int

    usb_mode = USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD; // what mode are we in
}

#define USB_OTG_FS_HOST ((USB_OTG_HostTypeDef *) USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_BASE)
#define USB_OTG_FS_HPRT ((uint32_t *) USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_PORT_BASE)

typedef enum
{
    Full_speed = 1,
    Low_speed = 2
} USB_speed;

USB_speed usb_speed;

// sizes in terms of 32-bit words; min 16, max 256
void USB_host_init(const uint16_t rx_fifo_size, const uint16_t np_tx_fifo_size, const uint16_t np_tx_ram_start,
                   const uint16_t p_tx_fifo_size, const uint16_t p_tx_ram_start)
{
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_PRTIM; // unmask host port int

    *USB_OTG_FS_HPRT |= USB_OTG_HPRT_PPWR; // drive the Vbus
    while (!(*USB_OTG_FS_HPRT & USB_OTG_HPRT_PCDET)){} // wait for a device to connect
    *USB_OTG_FS_HPRT |= USB_OTG_HPRT_PCDET; // clear interrupt
    *USB_OTG_FS_HPRT |= USB_OTG_HPRT_PRST; // start port reset
    delay_ms(11); // waiting at least 10 millis
    *USB_OTG_FS_HPRT &= ~USB_OTG_HPRT_PRST; // end port reset
    while (!(*USB_OTG_FS_HPRT & USB_OTG_HPRT_PENCHNG)){} // wait for port to change state
    *USB_OTG_FS_HPRT |= USB_OTG_HPRT_PENCHNG; // clear interrupt
    usb_speed = (*USB_OTG_FS_HPRT & USB_OTG_HPRT_PSPD) >> USB_OTG_HPRT_PSPD_Pos; // read speed

    const uint8_t prev_speed = USB_OTG_FS_HOST->HCFG & USB_OTG_HCFG_FSLSPCS; // get curr port speed
    if (prev_speed != usb_speed)
    {
        USB_OTG_FS_HOST->HCFG &= ~USB_OTG_HCFG_FSLSPCS;
        USB_OTG_FS_HOST->HCFG |= usb_speed & USB_OTG_HCFG_FSLSPCS; // set new speed
        *USB_OTG_FS_HPRT |= USB_OTG_HPRT_PRST; // start port reset
        delay_ms(11); // waiting at least 10 millis
        *USB_OTG_FS_HPRT &= ~USB_OTG_HPRT_PRST; // end port reset
    }

    USB_OTG_FS->GRXFSIZ = rx_fifo_size;
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = np_tx_fifo_size << USB_OTG_NPTXFD_Pos | np_tx_ram_start;
    USB_OTG_FS->HPTXFSIZ = p_tx_fifo_size << USB_OTG_HPTXFSIZ_PTXFD_Pos | p_tx_ram_start;
}

#define USB_OTG_FS_DEVICE ((USB_OTG_DeviceTypeDef *) USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE)
#define USB_OTG_FS_DIEPCTL0 ((uint32_t *) USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE)

typedef enum
{
    Max64B = 0,
    Max32B,
    Max16B,
    Max8B,
} UBS_FS_MPSIZ;

void USB_device_init(const UBS_FS_MPSIZ max_packet_size)
{
    USB_OTG_FS_DEVICE->DCFG |= USB_OTG_DCFG_DSPD; // full speed
    USB_OTG_FS_DEVICE->DCFG &= ~USB_OTG_DCFG_NZLSOHSK; // todo: potentially make a parameter
    // see the datasheet for more info

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBRST; // enable USB reset int
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_ENUMDNEM; // enable enumeration done int
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_ESUSPM; // enable early suspend int
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM; // enable USB suspend int
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_SOFM; // enable SOF int

    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN; // enable Vbus sensing in b device mode

    while (!(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_USBRST)){} // wait for reset
    USB_OTG_FS->GINTSTS |= USB_OTG_GINTSTS_USBRST; // clear interrupt

    while (!(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)){} // wait for enumeration to finish
    if ((USB_OTG_FS_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD) != USB_OTG_DSTS_ENUMSPD) // read enumeration speed
    {
        return; // should never happen
    }
    *USB_OTG_FS_DIEPCTL0 |= max_packet_size; // set maximum packet size
}

#endif //STM32_KIT_USB
