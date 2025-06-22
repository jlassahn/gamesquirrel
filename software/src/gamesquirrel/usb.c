/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#pragma GCC optimize ("-fno-strict-aliasing")

#include "stm32h503xx.h"
#include "gamesquirrel/usb.h"
#include "gamesquirrel/charqueue.h"
#include "gamesquirrel/core.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct UsbState UsbState;
struct UsbState
{
    CharQueue rx_queue;
    CharQueue tx_queue;
    uint32_t scratch[4]; // staging area for data to go to or from USB SRAM
    const uint8_t *setup_data;
    int setup_count;
    int setup_rx_count;
    int usb_address;
    int active;
    int interrupts;
};

typedef struct UsbSetupPacket UsbSetupPacket;
struct UsbSetupPacket
{
    uint16_t request;
    uint16_t value;
    uint16_t index;
    uint16_t length;
};

// USB Product and Vendor IDs are from
// https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
// PID = 0x27dd  VID = 0x16c0   For CDC-ACM class devices (modems)
//
// These aren't unique to this device, but should only be used by other
// CDC-ACM devices that use default drivers.

const uint32_t zeros[4] = {0, 0, 0, 0};
const uint8_t device_descriptor[] =
{
    18, //bLength
    0x01, //bDescriptorType
    0x00, 0x02, //bcdUSB    // FIXME consider reporting v1.x so we don't have to respond to high-speed queries
    0x00, //bDeviceClass
    0x00, //bDeviceSubclass
    0x00, //bDeviceProtocol
    0x08, //bMaxPacketSize
    0xC0, 0x16, // idVendor
    0xDD, 0x27, // idProduct
    0x00, 0x01, // bcdDevice
    0x01, //iManufacturer
    0x02, //iProduct
    0x03, //iSerialNumber
    1, //bNumConfigurations
};

const uint8_t config_descriptor[] =
{
    // Configuration Descriptor
    9, //blength
    0x02, //bDescriptorType
    75, 0, //wTotalLength
    2,  //bNumInterfaces
    1, //bConfigurationValue
    0, //iConfiguration
    0x80, //bmAttributes
    250, //bMaxPower

    //CDC Descriptors

    // Interface Association
    8, //bLength
    0x0B, //TUSB_DESC_INTERFACE_ASSOCIATION
    0, //bFirstInterface
    2, //bInterfaceCount
    0x02, //bFunctionClass TUSB_CLASS_CDC,
    0x02, //bFunctionSubclass CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL
    0x00, //bFunctionProtocol CDC_COMM_PROTOCOL_NONE
    0, //iFunction

    // CDC Control Interface
    9,
    0x04, //TUSB_DESC_INTERFACE,
    0, //bInterfaceNumber
    0, //bAlternateSetting
    1, //bNumEndpoints
    0x02, //bInterfaceClass TUSB_CLASS_CDC,
    0x02, //bInterfaceSubclass CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL,
    0x00, //bInterfaceProtocol CDC_COMM_PROTOCOL_NONE,
    0,  //iInterface

    // CDC Header
    5,
    0x24, //TUSB_DESC_CS_INTERFACE,
    0x00, //CDC_FUNC_DESC_HEADER,
    0x20, 0x01, //U16_TO_U8S_LE(0x0120),

    // CDC Call
    5,
    0x24, //TUSB_DESC_CS_INTERFACE,
    0x01, //CDC_FUNC_DESC_CALL_MANAGEMENT,
    0,
    1, //(uint8_t)((_itfnum) + 1),

    // CDC ACM: support line request + send break
    4,
    0x24, //TUSB_DESC_CS_INTERFACE,
    0x02, //CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT,
    6,

    // CDC Union
    5,
    0x24, //TUSB_DESC_CS_INTERFACE,
    0x06, //CDC_FUNC_DESC_UNION,
    0, //_itfnum,
    1, //(uint8_t)((_itfnum) + 1),

    // Endpoint Notification
    7,
    0x05, //TUSB_DESC_ENDPOINT,
    0x82, // bEndpointAddress
    0x03, //TUSB_XFER_INTERRUPT,
    8, 0, //wMaxPacketSize
    16,

    // CDC Data Interface
    9,
    0x04, //TUSB_DESC_INTERFACE,
    0x01, //bInterfaceNumber
    0,
    2,
    10, //TUSB_CLASS_CDC_DATA,
    0,
    0,
    0,

    // Endpoint Out
    7,
    0x05, //TUSB_DESC_ENDPOINT,
    0x01, //bEndpointAddress
    0x02, //TUSB_XFER_BULK,
    16, 0, // wMaxPacketSize
    0,

    // Endpoint In
    7,
    0x05, //TUSB_DESC_ENDPOINT,
    0x81, // bEndpointAddress
    0x02, //TUSB_XFER_BULK,
    16, 0, // wMaxPacketSize
    0
};

const uint16_t strLangs[] =
{
    0x0304, // length 4
    0x0409  // English US
};

// FIXME figure out for real what Manufacturer and Product strings should be.
const uint16_t strManufacturer[] =
{
    0x030E, //length 14
    'M',
    'i',
    'a',
    'c',
    'i',
    'd'
};

const uint16_t strProduct[] =
{
    0x030C, //length 12
    'D',
    'e',
    'b',
    'u',
    'g'
};

// FIXME either use processor serial number or don't provide one
const uint16_t strSerial[] =
{
    0x030E, //length 14
    '1',
    '2',
    '3',
    '4',
    '5',
    '6'
};

const uint16_t *strings[] =
{
    strLangs,
    strManufacturer,
    strProduct,
    strSerial
};
const int stringCount = sizeof(strings)/sizeof(strings[0]);

static UsbState usb_state;


static inline void ClearChannelFlag(volatile uint32_t *chep, uint32_t mask)
{
    uint32_t val = *chep;
    val &= 0xFFFF8F8F; // clear toggle bits
    val |= 0x7E808080; // set write-zero bits
    val &= ~mask;
    *chep = val;
}

static inline void SetChannelToggle(volatile uint32_t *chep, uint32_t mask)
{
    uint32_t val = *chep;
    uint32_t flip = mask & ~val;
    val &= 0xFFFF8F8F; // clear toggle bits
    val |= 0x7E808080; // set write-zero bits
    val |= flip;
    *chep = val;
}

void UsbInit(void)
{
    // Perform USB peripheral reset
    DelayClocks(250);
    USB_DRD_FS->CNTR = USB_CNTR_USBRST | USB_CNTR_PDWN;
    DelayClocks(250);

    USB_DRD_FS->CNTR &= ~USB_CNTR_PDWN;

    // From STM32H523 datasheet, Tstartup for USB is <= 1us
    DelayClocks(250);

    USB_DRD_FS->CNTR = 0; // Enable USB

    // FIXME for some reason if I don't read back CNTR the CHEP registers don't
    // work.  WTF?  Is this a hardware quirk or a problem with compiler
    // optimization?  Maybe a compiler optimization issue, since removing the
    // volatile below prevents it from working.
    volatile uint32_t reg = USB_DRD_FS->CNTR;
    (void)reg;

    USB_DRD_FS->ISTR = 0; // Clear pending interrupts

    // Reset endpoints to disabled
    USB_DRD_FS->CHEP0R = 0;
    USB_DRD_FS->CHEP1R = 0;
    USB_DRD_FS->CHEP2R = 0;
    USB_DRD_FS->CHEP3R = 0;
    USB_DRD_FS->CHEP4R = 0;
    USB_DRD_FS->CHEP5R = 0;
    USB_DRD_FS->CHEP6R = 0;
    USB_DRD_FS->CHEP7R = 0;
}

static void UsbReset(void)
{
    usb_state.usb_address = 0;
    usb_state.setup_count = 0;
    usb_state.setup_rx_count = 0;
    usb_state.setup_data = NULL;

    USB_DRD_PMA_BUFF[0].TXBD = 0x00000000 | 0x00000040;
    USB_DRD_PMA_BUFF[0].RXBD = 0x10000000 | 0x00000048;
    USB_DRD_PMA_BUFF[1].TXBD = 0x00000000 | 0x00000050;
    USB_DRD_PMA_BUFF[1].RXBD = 0x20000000 | 0x00000060;
    USB_DRD_PMA_BUFF[2].TXBD = 0x00000000 | 0x00000070;

    // RX_DTOG1 | RX_DTOG2 puts receive side into valid mode
    // TX_DTOG2 puts transmit side into NAK mode
    USB_DRD_FS->CHEP0R = USB_EP_CONTROL | USB_CHEP_TX_DTOG2 | USB_CHEP_RX_DTOG1 | USB_CHEP_RX_DTOG2 | 0x00;
    USB_DRD_FS->CHEP1R = USB_EP_BULK | USB_CHEP_TX_DTOG2 | USB_CHEP_RX_DTOG1 | USB_CHEP_RX_DTOG2 | 0x01;
    USB_DRD_FS->CHEP2R = USB_EP_INTERRUPT | USB_CHEP_TX_DTOG2 | 0x02;

    USB_DRD_FS->DADDR = USB_DADDR_EF; // Enable USB Function
}

static void SendSetup(void)
{
    volatile uint32_t *usb_memory = (uint32_t *)USB_DRD_PMA_BUFF;
    volatile uint32_t *tx0 = &usb_memory[0x10];

    // might do misaligned uint32 accesses
    // might read up to 7 bytes past the end of the valid data
    const uint32_t *data = (const uint32_t *)(const void *)usb_state.setup_data;
    tx0[0] = data[0];
    tx0[1] = data[1];

    // FIXME if actual payload size is a multiple of 8, and also less
    // than the requested size, then send a zero length packet to terminate.
    uint32_t count = 0;
    if (usb_state.setup_count > 8)
    {
        count = 8;
        usb_state.setup_count -= 8;
        usb_state.setup_data += 8;
    }
    else
    {
        count = usb_state.setup_count;
        usb_state.setup_count = 0;
        usb_state.setup_data = NULL;
    }
    USB_DRD_PMA_BUFF[0].TXBD = (count << 16) | 0x00000040;
    SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
}

static void SendEmptySetup(void)
{
    USB_DRD_PMA_BUFF[0].TXBD = 0 | 0x00000040;
    SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
}

static void UsbTransfer(void)
{
    uint32_t *usb_memory = (uint32_t *)USB_DRD_PMA_BUFF;
    volatile uint32_t *rx0 = &usb_memory[0x12];
    volatile uint32_t *rx1 = &usb_memory[0x18];

    uint32_t val = USB_DRD_FS->CHEP0R;
    if (val & USB_CHEP_VTRX)
    {
        ClearChannelFlag(&USB_DRD_FS->CHEP0R, USB_CHEP_VTRX);
        if (val & USB_CHEP_SETUP)
        {
            usb_state.scratch[0] = rx0[0];
            usb_state.scratch[1] = rx0[1];
            UsbSetupPacket *setup = (UsbSetupPacket *)usb_state.scratch;

            if ((setup->request & 0x0080) == 0) // OUT data
            {
                if (setup->request == 0x0500) // SET_ADDRESS
                {
                    usb_state.usb_address = setup->value & 0x7F;
                }
                else if (setup->request == 0x0900) // SET_CONFIGURATION
                {
                    usb_state.active = true;
                }

                if (setup->length == 0)
                {
                    SendEmptySetup();
                }
                else
                {
                    usb_state.setup_rx_count = setup->length;
                }

            }
            else // IN data
            {
                if (setup->request == 0x0680) // GET_DESCRIPTOR
                {
                    const uint8_t *data = (const uint8_t *)zeros;
                    int length = 0;
                    if (setup->value == 0x0100)
                    {
                        data = device_descriptor;
                        length = sizeof(device_descriptor);
                    }
                    else if (setup->value == 0x0200)
                    {
                        data = config_descriptor;
                        length = sizeof(config_descriptor);
                    }
                    else if ((setup->value & 0xFF00) == 0x0300)
                    {
                        int n = setup->value & 0xFF;
                        if (n <= stringCount)
                        {
                            data = (const uint8_t *)strings[n];
                            length = strings[n][0] & 0xFF;
                        }
                    }

                    usb_state.setup_data = data;
                    usb_state.setup_count = length;
                }
                else
                {
                    usb_state.setup_data = (const uint8_t *)zeros;
                    usb_state.setup_count = 0;
                }
                if (setup->length < usb_state.setup_count)
                    usb_state.setup_count = setup->length;
                SendSetup();
            }

            // CDC specific commands
            // case 0x2221: //SET_CONTROL_LINE_STATE
            // case 0x2021: //SET_LINE_CODING
            // case 0x2321: //SEND_BREAK

            // Device Requests
            // case 0x0080: // GET_STATUS
            // case 0x0100: // CLEAR_FEATURE
            // case 0x0300: // SET_FEATURE
            // case 0x0700: // SET_DESCRIPTOR
            // case 0x0880: // GET_CONFIGURATION

            // Interface Requests
            // case 0x0081: // GET_STATUS
            // case 0x0181: // CLEAR_FEATURE
            // case 0x0301: // SET_FEATURE
            // case 0x0A81: // GET_INTERFACE
            // case 0x1101: // SET_INTERFACE

            // Endpoint Requests
            // case 0x0082: // GET_STATUS
            // case 0x0102: // CLEAR_FEATURE
            // case 0x0302: // SET_FEATURE
            // case 0x1282: // SYNCH_FRAME
        }
        else
        {
            if (usb_state.setup_rx_count > 0)
            {
                uint32_t reg = USB_DRD_PMA_BUFF[0].RXBD;
                int length = (reg >> 16) & 0x3FF;
                if (length >= usb_state.setup_rx_count)
                {
                    usb_state.setup_rx_count = 0;
                    SendEmptySetup();
                }
                else
                {
                    usb_state.setup_rx_count -= length;
                }
            }
        }
        SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_RX_STRX);
    }
    if (val & USB_CHEP_VTTX)
    {
        ClearChannelFlag(&USB_DRD_FS->CHEP0R, USB_CHEP_VTTX);
        if (usb_state.setup_data != NULL)
        {
            SendSetup();
        }
        else
        {
            USB_DRD_FS->DADDR = USB_DADDR_EF | usb_state.usb_address;
        }
    }

    val = USB_DRD_FS->CHEP1R;
    if (val & USB_CHEP_VTRX)
    {
        ClearChannelFlag(&USB_DRD_FS->CHEP1R, USB_CHEP_VTRX);

        uint32_t reg = USB_DRD_PMA_BUFF[1].RXBD;
        int length = (reg >> 16) & 0x3FF;
        usb_state.scratch[0] = rx1[0];
        usb_state.scratch[1] = rx1[1];
        usb_state.scratch[2] = rx1[2];
        usb_state.scratch[3] = rx1[3];

        CharQueue_Write(&usb_state.rx_queue, (char *)usb_state.scratch, length);
        SetChannelToggle(&USB_DRD_FS->CHEP1R, USB_CHEP_RX_STRX);
    }
    if (val & USB_CHEP_VTTX)
    {
        ClearChannelFlag(&USB_DRD_FS->CHEP1R, USB_CHEP_VTTX);
    }
}

void UsbStart(void)
{
    // Enable pull-up
    USB_DRD_FS->BCDR |= USB_BCDR_DPPU;
    UsbReset();

    // enable interrupts
    USB_DRD_FS->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;
    NVIC_EnableIRQ(USB_DRD_FS_IRQn);
}

void USB_DRD_FS_IRQHandler(void)
{
    usb_state.interrupts ++;

    // Chip errata says data in USB SRAM might be up to 800ns later
    // than receive complete interrupt.  Clock is about 4ns so we want
    // about 200 instructions of delay.
    DelayClocks(200);

    uint32_t stat = USB_DRD_FS->ISTR;
    uint32_t clear = (~stat) | 0xFFFC807F;
    USB_DRD_FS->ISTR = clear;

    // ignore expected start of frame flag USB_ISTR_ESOF
    // ignore start of frame flag USB_ISTR_SOF
    // ignore suspend flag USB_ISTR_SUSP
    // ignore wakeup flag USB_ISTR_WKUP
    // ignore error flag USB_ISTR_ERR (default recovery from errors is good enough)
    // ignore packet overrun USB_ISTR_PMAOVR (should never happen)
    // USB_ISTR_DDISC should be host-mode only
    // USB_ISTR_THR512 not used here

    if (stat & USB_ISTR_RESET)
    {
        // handle device leaving reset
        UsbReset();
    }

    if (stat & USB_ISTR_CTR)
    {
        // handle transfer complete
        UsbTransfer();
    }

    // If data endpoint TX buffer is empty...
    // Checking for this here instead of in UsbTransfer because if the
    // tx path has been idle we will have already received and cleared the
    // transfer complete interrupt before there is new data to transmit.
    if ((USB_DRD_FS->CHEP1R & USB_CHEP_TX_STTX) == USB_EP_TX_NAK)
    {
        int length = CharQueue_Read(&usb_state.tx_queue, (char *)usb_state.scratch, 16);
        if (length > 0)
        {
            volatile uint32_t *usb_memory = (uint32_t *)USB_DRD_PMA_BUFF;
            volatile uint32_t *tx1 = &usb_memory[0x14];
            tx1[0] = usb_state.scratch[0];
            tx1[1] = usb_state.scratch[1];
            tx1[2] = usb_state.scratch[2];
            tx1[3] = usb_state.scratch[3];
            USB_DRD_PMA_BUFF[1].TXBD = (length << 16) | 0x00000050;
            SetChannelToggle(&USB_DRD_FS->CHEP1R, USB_CHEP_TX_STTX);
        }
    }
}

int UsbSend(const char *data, int max_bytes)
{
    int n = CharQueue_Write(&usb_state.tx_queue, data, max_bytes);
    if (n > 0)
        NVIC_SetPendingIRQ(USB_DRD_FS_IRQn);
    return n;
}

int UsbReceive(char *data, int max_bytes)
{
    return CharQueue_Read(&usb_state.rx_queue, data, max_bytes);
}

bool UsbIsActive(void)
{
    return usb_state.active;
}

int UsbInterruptCount(void)
{
    return usb_state.interrupts;
}

