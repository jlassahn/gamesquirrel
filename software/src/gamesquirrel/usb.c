
#include "stm32h503xx.h"
#include "gamesquirrel/usb.h"
#include <stdio.h>
#include <string.h>

typedef struct UsbState UsbState;
struct UsbState
{
	uint32_t scratch[4]; // staging area for data to go to or from USB SRAM
	const uint8_t *setup_data;
	int setup_count;
	int setup_rx_count;
	int usb_address;
};

typedef struct UsbSetupPacket UsbSetupPacket;
struct UsbSetupPacket
{
	uint16_t request;
	uint16_t value;
	uint16_t index;
	uint16_t length;
};


const uint32_t zeros[4] = {0, 0, 0, 0};
const uint8_t device_descriptor[] =
{
	18, //bLength
	0x01, //bDescriptorType
	0x00, 0x02, //bcdUSB
	0x00, //bDeviceClass
	0x00, //bDeviceSubclass
	0x00, //bDeviceProtocol
	0x08, //bMaxPacketSize
	0x34, 0x12, // idVendor FIXME
	0xCD, 0xAB, // idProduct FIXME
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

_Static_assert(sizeof(config_descriptor) == 75);

const uint16_t strLangs[] =
{
	0x0304, // length 4
	0x0409  // English US
};

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


// FIXME use TitleCase for function names!

void usb_init(void)
{
  // Follow the RM mentions to use a special ordering of PDWN and FRES
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }

  // Perform USB peripheral reset
  USB_DRD_FS->CNTR = USB_CNTR_USBRST | USB_CNTR_PDWN;
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }

  USB_DRD_FS->CNTR &= ~USB_CNTR_PDWN;

  // Wait startup time, for F042 and F070, this is <= 1 us.
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }
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

static void usb_reset(void)
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

static void send_setup(void)
{
	volatile uint32_t *usb_memory = (uint32_t *)USB_DRD_PMA_BUFF;
	volatile uint32_t *tx0 = &usb_memory[0x10];

	// might do misaligned uint32 accesses
	// might read up to 7 bytes past the end of the valid data
	const uint32_t *data = (const uint32_t *)usb_state.setup_data;
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

	printf("t\r\n");
	//printf(" ...sending %d bytes %p [%.8lX %.8lX]\r\n", (int)count, data, tx0[0], tx0[1]);
	USB_DRD_PMA_BUFF[0].TXBD = (count << 16) | 0x00000040;
}

static void usb_transfer(void)
{
	uint32_t *usb_memory = (uint32_t *)USB_DRD_PMA_BUFF;
	volatile uint32_t *rx0 = &usb_memory[0x12];
	volatile uint32_t *tx1 = &usb_memory[0x14];
	volatile uint32_t *rx1 = &usb_memory[0x18];

	// FIXME data in USB SRAM might be up to 800ns later than receive complete interrupt.

	uint32_t val = USB_DRD_FS->CHEP0R;
	if (val & USB_CHEP_VTRX)
	{
		ClearChannelFlag(&USB_DRD_FS->CHEP0R, USB_CHEP_VTRX);
		if (val & USB_CHEP_SETUP)
		{
			putchar('S');
			//printf("setup\r\n");
			usb_state.scratch[0] = rx0[0];
			usb_state.scratch[1] = rx0[1];
			UsbSetupPacket *setup = (UsbSetupPacket *)usb_state.scratch;

			switch (setup->request)
			{
				case 0x0500: // SET_ADDRESS
					printf("->sa\r\n");
					usb_state.usb_address = setup->value & 0x7F;
					usb_state.setup_data = (const uint8_t *)zeros;
					usb_state.setup_count = 0;
					send_setup();
					SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
					break;

				case 0x0680: // GET_DESCRIPTOR
					printf("->gd\r\n");
					if (setup->value == 0x0100)
					{
						usb_state.setup_data = device_descriptor;
						usb_state.setup_count = sizeof(device_descriptor);
						if (setup->length < usb_state.setup_count)
							usb_state.setup_count = setup->length;
						send_setup();
						SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
					}
					else if (setup->value == 0x0200)
					{
						usb_state.setup_data = config_descriptor;
						usb_state.setup_count = sizeof(config_descriptor);
						if (setup->length < usb_state.setup_count)
							usb_state.setup_count = setup->length;
						send_setup();
						SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
					}
					else if ((setup->value & 0xFF00) == 0x0300)
					{
						int n = setup->value & 0xFF;
						if (n <= stringCount)
						{
							usb_state.setup_data = (const uint8_t *)strings[n];
							usb_state.setup_count = strings[n][0] & 0xFF;
							if (setup->length < usb_state.setup_count)
								usb_state.setup_count = setup->length;
							send_setup();
							SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
						}

					}
					else
					{
						printf("unknown descriptor %.4X\r\n", setup->value);
					}
					// setup->value is descriptor type and index
					// setup->index is language ID
					// setup->length is max transfer length
					break;

				case 0x0900: // SET_CONFIGURATION
					printf("->sc\r\n");
					usb_state.setup_data = (const uint8_t *)zeros;
					usb_state.setup_count = 0;
					send_setup();
					SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
					break;

				case 0x2221: //SET_CONTROL_LINE_STATE
					printf("Set Control Line State %d\r\n", setup->length);
					usb_state.setup_data = (const uint8_t *)zeros;
					usb_state.setup_count = 0;
					send_setup();
					SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
					break;

				case 0x2021: //SET_LINE_CODING
					printf("Set Control Line Coding %d\r\n", setup->length);
					usb_state.setup_rx_count = setup->length;
					break;

				case 0x2321: //SEND_BREAK
					printf("Send Break %d\r\n", setup->length);
					usb_state.setup_data = (const uint8_t *)zeros;
					usb_state.setup_count = 0;
					send_setup();
					SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
					break;

				// Device Requests
				case 0x0080: // GET_STATUS
				case 0x0100: // CLEAR_FEATURE
				case 0x0300: // SET_FEATURE
				case 0x0700: // SET_DESCRIPTOR
				case 0x0880: // GET_CONFIGURATION

				// Interface Requests
				case 0x0081: // GET_STATUS
				case 0x0181: // CLEAR_FEATURE
				case 0x0301: // SET_FEATURE
				case 0x0A81: // GET_INTERFACE
				case 0x1101: // SET_INTERFACE

				// Endpoint Requests
				case 0x0082: // GET_STATUS
				case 0x0102: // CLEAR_FEATURE
				case 0x0302: // SET_FEATURE
				case 0x1282: // SYNCH_FRAME

				default:
					printf("unknown request %.4X\r\n", setup->request);
			}
		}
		else
		{
			printf("Rx\r\n");
			if (usb_state.setup_rx_count > 0)
			{
				uint32_t reg = USB_DRD_PMA_BUFF[0].RXBD;
				int length = (reg >> 16) & 0x3FF;
				printf("expecting %d received %d\r\n", usb_state.setup_rx_count, length);
				if (length > usb_state.setup_rx_count)
					usb_state.setup_rx_count = 0;
				else
					usb_state.setup_rx_count -= length;

				if (usb_state.setup_rx_count == 0)
				{
					usb_state.setup_data = (const uint8_t *)zeros;
					usb_state.setup_count = 0;
					send_setup();
					SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
				}
			}
			// FIXME cases to handle
			// ACK after end of IN control sequence
			//     udate usb_address if needed
			// handle data from OUT control sequence, ACK when done
		}
		SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_RX_STRX);
	}
	if (val & USB_CHEP_VTTX)
	{
		ClearChannelFlag(&USB_DRD_FS->CHEP0R, USB_CHEP_VTTX);
		//printf("Tx");
		if (usb_state.setup_data != NULL)
		{
			//printf("->c\r\n");
			send_setup();
			SetChannelToggle(&USB_DRD_FS->CHEP0R, USB_CHEP_TX_STTX);
		}
		else
		{
			//printf("->d\r\n");
			USB_DRD_FS->DADDR = USB_DADDR_EF | usb_state.usb_address;
		}
	}

	val = USB_DRD_FS->CHEP1R;
	if (val & USB_CHEP_VTRX)
	{
		ClearChannelFlag(&USB_DRD_FS->CHEP1R, USB_CHEP_VTRX);
		printf("Endpoint 1 Rx\r\n");
		uint32_t reg = USB_DRD_PMA_BUFF[1].RXBD;
		int length = (reg >> 16) & 0x3FF;
		printf("Rx length = %d [%.8lX %.8lX]\r\n",
				length, rx1[0], rx1[1]);
		SetChannelToggle(&USB_DRD_FS->CHEP1R, USB_CHEP_RX_STRX);
	}
	if (val & USB_CHEP_VTTX)
	{
		ClearChannelFlag(&USB_DRD_FS->CHEP1R, USB_CHEP_VTTX);
		printf("Endpoint 1 Tx\r\n");
		(void)tx1;
	}
}

void usb_start(void)
{
	printf("USB start\r\n");

	// Enable pull-up
	USB_DRD_FS->BCDR |= USB_BCDR_DPPU;
	usb_reset();
}

void usb_tick(void)
{
	uint32_t stat = USB_DRD_FS->ISTR;
	uint32_t clear = (~stat) | 0xFFFC807F;
	USB_DRD_FS->ISTR = clear;

	// ignore expected start of frame flag USB_ISTR_ESOF;
	// ignore start of frame flag USB_ISTR_SOF;

	if (stat & USB_ISTR_RESET)
	{
		// handle device leaving reset
		printf("USB reset\r\n");
		usb_reset();
	}

	if (stat & USB_ISTR_SUSP)
	{
		// handle device suspend (probably a NOOP)
		// printf("USB suspend\r\n");
		printf(".");
	}

	if (stat & USB_ISTR_WKUP)
	{
		// handle wake up from suspend (probably a NOOP)
		printf("USB wakeup\r\n");
	}

	if (stat & USB_ISTR_ERR)
	{
		// handle error detect (probably a NOOP)
		printf("USB error\r\n");
	}

	if (stat & USB_ISTR_PMAOVR)
	{
		// packet memory overrun, can't happen if we're set up correctly
		printf("USB packet overrun\r\n");
	}

	if (stat & USB_ISTR_CTR)
	{
		// handle transfer complete
		//printf("USB transfer complete\r\n");
		//printf("{\r\n");
		usb_transfer();
		//printf("}\r\n");
		//printf("final CHEP0R = %.8lX istr = %.8lX\r\n", USB_DRD_FS->CHEP0R, USB_DRD_FS->ISTR);
	}

	// USB_ISTR_DDISC should be host-mode only
	// USB_ISTR_THR512 not used here
}

