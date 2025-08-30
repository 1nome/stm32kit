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
#include "pin.h"
#include "i2c_lcd.h" // for debug purposes

#define USB_OTG_FS_HOST ((USB_OTG_HostTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_BASE))
#define USB_OTG_FS_HPRT ((__IO uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_PORT_BASE))
#define USB_OTG_FS_DEVICE ((USB_OTG_DeviceTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_FS_IEP0 ((USB_OTG_INEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE))
#define USB_OTG_FS_HC0 ((USB_OTG_HostChannelTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_CHANNEL_BASE))

void usb_debug_write(const char* str, const uint8_t add)
{
    if (!add)
    {
        I2C_LCD_set(LCD_LINE2);
        I2C_LCD_print("                ");
        I2C_LCD_set(LCD_LINE2);
    }
    I2C_LCD_print(str);
}

volatile struct
{
    uint8_t pcdet : 1;
    uint8_t penchng : 1;
} usb_ints;

void USB_handle_port()
{
    usb_debug_write("HPRT-", 1);
    if (*USB_OTG_FS_HPRT & USB_OTG_HPRT_PCDET)
    {
        *USB_OTG_FS_HPRT |= USB_OTG_HPRT_PCDET; // clear by writing 1
        usb_ints.pcdet = 1;
        usb_debug_write("PCDET", 1);
    }
    if (*USB_OTG_FS_HPRT & USB_OTG_HPRT_PENCHNG)
    {
        *USB_OTG_FS_HPRT |= USB_OTG_HPRT_PENCHNG; // clear by writing 1
        usb_ints.penchng = 1;
        usb_debug_write("PENCHNG", 1);
    }
}

void OTG_FS_IRQHandler(void)
{
    io_set(LED_IN_1, io_get(LED_IN_1) ? 0 : 1);
    usb_debug_write("USB-", 0);
    if (USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_HPRTINT)
    {
        USB_handle_port();
    }

}

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

    // USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_HNPCAP; // hnp on
    // USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_SRPCAP; // srp on
    USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
    USB_OTG_FS->GUSBCFG |= calc_trdt(); // turnaround time
    USB_OTG_FS->GUSBCFG |= 2; // timeout calibration (didn't test, think 2 is neat)

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_OTGINT; // unmask OTG int
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_MMISM; // unmask mode mismatch int

    usb_mode = USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD; // read what mode are we in
}

typedef enum
{
    Full_speed = 1,
    Low_speed = 2
} USB_speed;

USB_speed usb_speed;

// sizes min 16, max 256, in terms of 32-bit words
// np_tx_ram_start >= rx_fifo_size,
// p_tx_ram_start >= np_tx_ram_start + np_tx_ram_size
// 320 words (1280B) > p_tx_ram_start + p_tx_ram_size
void USB_host_init(const uint16_t rx_fifo_size, const uint16_t np_tx_fifo_size, const uint16_t np_tx_ram_start,
                   const uint16_t p_tx_fifo_size, const uint16_t p_tx_ram_start)
{
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_PRTIM; // unmask host port int
    USB_OTG_FS_HOST->HCFG |= USB_OTG_HCFG_FSLSPCS_0; // FS host mode
    *USB_OTG_FS_HPRT |= USB_OTG_HPRT_PPWR; // drive the Vbus
    while (!usb_ints.pcdet){} // wait for a device to connect
    usb_ints.pcdet = 0; // clear interrupt
    *USB_OTG_FS_HPRT |= USB_OTG_HPRT_PRST; // start port reset
    delay_ms(11); // waiting at least 10 millis
    *USB_OTG_FS_HPRT &= ~USB_OTG_HPRT_PRST; // end port reset
    while (!usb_ints.penchng){} // wait for port to change state
    usb_ints.penchng = 0; // clear interrupt
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
    USB_OTG_FS_IEP0->DIEPCTL |= max_packet_size; // set maximum packet size
}

typedef enum
{
    DATA0 = 0,
    DATA1,
    DATA2,
    MDATA_SETUP
} USB_DPID;

// channels 0 - 7
// transfer completed, channel halted, STALL response received, NAK response received,
// ACK response received/transmitted, transaction error, babble error, frame overrun, data toggle error
void USB_host_channel_init_ints(const uint8_t chan, const uint8_t xfrc, const uint8_t chh, const uint8_t stall,
                                const uint8_t nak, const uint8_t ack, const uint8_t txerr, const uint8_t bberr,
                                const uint8_t frmor, const uint8_t dterr)
{
    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_NPTXFEM; // enable np tx fifo empty int
    USB_OTG_FS_HOST->HAINTMSK |= 1 << chan & USB_OTG_HAINTMSK_HAINTM; // enable ints from the selected channel

    USB_OTG_HostChannelTypeDef* hc = USB_OTG_FS_HC0 + chan;
    if (xfrc) hc->HCINTMSK |= USB_OTG_HCINTMSK_XFRCM;
    else hc->HCINTMSK &= ~USB_OTG_HCINTMSK_XFRCM;
    if (chh) hc->HCINTMSK |= USB_OTG_HCINTMSK_CHHM;
    else hc->HCINTMSK &= ~USB_OTG_HCINTMSK_CHHM;
    if (stall) hc->HCINTMSK |= USB_OTG_HCINTMSK_STALLM;
    else hc->HCINTMSK &= ~USB_OTG_HCINTMSK_STALLM;
    if (nak) hc->HCINTMSK |= USB_OTG_HCINTMSK_NAKM;
    else hc->HCINTMSK &= ~USB_OTG_HCINTMSK_NAKM;
    if (ack) hc->HCINTMSK |= USB_OTG_HCINTMSK_ACKM;
    else hc->HCINTMSK &= ~USB_OTG_HCINTMSK_ACKM;
    if (txerr) hc->HCINTMSK |= USB_OTG_HCINTMSK_TXERRM;
    else hc->HCINTMSK &= ~USB_OTG_HCINTMSK_TXERRM;
    if (bberr) hc->HCINTMSK |= USB_OTG_HCINTMSK_BBERRM;
    else hc->HCINTMSK &= ~USB_OTG_HCINTMSK_BBERRM;
    if (frmor) hc->HCINTMSK |= USB_OTG_HCINTMSK_FRMORM;
    else hc->HCINTMSK &= ~USB_OTG_HCINTMSK_FRMORM;
    if (dterr) hc->HCINTMSK |= USB_OTG_HCINTMSK_DTERRM;
    else hc->HCINTMSK &= ~USB_OTG_HCINTMSK_DTERRM;
}

typedef enum
{
    Control = 0,
    Isochronous,
    Bulk,
    Interrupt
} USB_EPTYP;

typedef enum
{
    Out = 0,
    In
} USB_EPDIR;

// channels 0-7
void USB_host_channel_init_txsize(const uint8_t chan, const uint32_t transfer_size, const uint16_t packet_count, const USB_DPID data_pid)
{
    USB_OTG_HostChannelTypeDef* hc = USB_OTG_FS_HC0 + chan;

    hc->HCTSIZ &= ~(USB_OTG_HCTSIZ_DPID | USB_OTG_HCTSIZ_PKTCNT | USB_OTG_HCTSIZ_XFRSIZ);
    hc->HCTSIZ |= transfer_size & USB_OTG_HCTSIZ_XFRSIZ;
    hc->HCTSIZ |= packet_count << USB_OTG_HCTSIZ_PKTCNT_Pos & USB_OTG_HCTSIZ_PKTCNT;
    hc->HCTSIZ |= data_pid << USB_OTG_HCTSIZ_DPID_Pos;
}

USB_EPTYP usb_types[8];

// channels 0-7
// low speed device, endpoint number, device address
void USB_host_channel_init_chars(const uint8_t chan, const USB_EPDIR dir, const USB_EPTYP type, const uint8_t lsdev,
                                 const uint8_t epnum, const uint8_t dad)
{
    USB_OTG_HostChannelTypeDef* hc = USB_OTG_FS_HC0 + chan;

    hc->HCCHAR &= ~USB_OTG_HCCHAR_EPDIR;
    hc->HCCHAR |= dir << USB_OTG_HCCHAR_EPDIR_Pos;
    hc->HCCHAR &= ~USB_OTG_HCCHAR_EPTYP;
    hc->HCCHAR |= type << USB_OTG_HCCHAR_EPTYP_Pos;
    usb_types[chan] = type; // storing channel types so they don't have to be read from registers
    if (lsdev) hc->HCCHAR |= USB_OTG_HCCHAR_LSDEV;
    else hc->HCCHAR &= ~USB_OTG_HCCHAR_LSDEV;
    hc->HCCHAR &= ~USB_OTG_HCCHAR_EPNUM;
    hc->HCCHAR |= epnum << USB_OTG_HCCHAR_EPNUM_Pos & USB_OTG_HCCHAR_EPNUM;
    hc->HCCHAR &= ~USB_OTG_HCCHAR_DAD;
    hc->HCCHAR |= dad << USB_OTG_HCCHAR_DAD_Pos & USB_OTG_HCCHAR_DAD;
}

void USB_host_channel_halt(const uint8_t chan, const uint8_t flush_requests)
{
    USB_OTG_HostChannelTypeDef* hc = USB_OTG_FS_HC0 + chan;

    if (flush_requests) hc->HCCHAR &= ~USB_OTG_HCCHAR_CHENA;

    hc->HCCHAR |= USB_OTG_HCCHAR_CHDIS;
}

typedef enum
{
    USB_res_ok = 0,
    USB_res_noSpace,
    USB_res_emptyQueue,
    USB_res_notIn,
    USB_res_noData
} USB_result;

USB_result USB_host_write_packet(const uint8_t chan, const uint8_t* data, const uint16_t len)
{
    uint16_t space;
    uint8_t qspace;
    if (usb_types[chan] == Control || usb_types[chan] == Bulk)
    {
        space = USB_OTG_FS->HNPTXSTS & USB_OTG_GNPTXSTS_NPTXFSAV;
        qspace = (USB_OTG_FS->HNPTXSTS & USB_OTG_GNPTXSTS_NPTQXSAV) >> USB_OTG_GNPTXSTS_NPTQXSAV_Pos;
    }
    else
    {
        space = USB_OTG_FS_HOST->HPTXSTS & USB_OTG_HPTXSTS_PTXFSAVL;
        qspace = (USB_OTG_FS_HOST->HPTXSTS & USB_OTG_HPTXSTS_PTXQSAV) >> USB_OTG_HPTXSTS_PTXQSAV_Pos;
    }

    const uint16_t n32 = (len + 3) / 4;

    if (space < n32 || qspace == 0)
    {
        return USB_res_noSpace;
    }

    uint32_t* fifo = (uint32_t*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + USB_OTG_FIFO_SIZE * chan);

    for (uint16_t i = 0; i < n32; i++, data += 4)
    {
        *fifo = *(uint32_t*)data;
    }

    return USB_res_ok;
}

// data needs to have enough space for a max size packet
USB_result USB_host_read_packet(const uint8_t chan, uint8_t* data, uint16_t* len)
{
    if (!(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL))
    {
        return USB_res_emptyQueue;
    }

    USB_OTG_FS->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;

    const uint32_t packet_info = USB_OTG_FS->GRXSTSP;
    if ((packet_info & USB_OTG_GRXSTSP_PKTSTS) != 2 << USB_OTG_GRXSTSP_PKTSTS_Pos)
    {
        USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
        return USB_res_notIn;
    }
    *len = (packet_info & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
    if (*len == 0)
    {
        USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
        return USB_res_noData;
    }

    const uint16_t n32 = (*len + 3) / 4;
    const uint32_t* fifo = (uint32_t*)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + USB_OTG_FIFO_SIZE * chan);

    for (uint16_t i = 0; i < n32; i++, data += 4)
    {
        *(uint32_t*)data = *fifo;
    }

    USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
    return USB_res_ok;
}

void USB_phy_init()
{
    pin_setup_af(USB_VBUS, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF10);
    pin_setup_af(USB_ID, PIN_MODE_AF, PIN_PULL_UP, PIN_SPEED_VERYHIGH, PIN_TYPE_OPENDRAIN, PIN_AF10);
    pin_setup_af(USB_DMINUS, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF10);
    pin_setup_af(USB_DPLUS, PIN_MODE_AF, PIN_PULL_NONE, PIN_SPEED_VERYHIGH, PIN_TYPE_PUSHPULL, PIN_AF10);

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

    pin_setup(USB_PWON, PIN_MODE_OUTPUT, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL);
    io_set(USB_PWON, 1); // reset
    delay_ms(200);
    io_set(USB_PWON, 0);

    pin_setup(USB_OC, PIN_MODE_INPUT, PIN_PULL_NONE, PIN_SPEED_HIGH, PIN_TYPE_PUSHPULL);
}

void EXTI9_5_IRQHandler(void)
{
    io_set(USB_PWON, 1); // disable vbus generation
    *USB_OTG_FS_HPRT &= ~USB_OTG_HPRT_PPWR; // disable port power

    EXTI->PR |= EXTI_PR_PR5; // clear int

    usb_debug_write("Overcurrent", 0);
}

void USB_ints_init()
{
    NVIC_SetPriority(SysTick_IRQn, 1); // needed so delays work inside our handlers

    NVIC_SetPriority(OTG_FS_IRQn, 3);
    NVIC_EnableIRQ(OTG_FS_IRQn);

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PD; // exti chan 5 source is port d
    EXTI->RTSR |= EXTI_RTSR_TR5; // triggered on rising edge
    EXTI->IMR |= EXTI_IMR_MR5; // enable int
    NVIC_SetPriority(EXTI9_5_IRQn, 2);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

#endif //STM32_KIT_USB
