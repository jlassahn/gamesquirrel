/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "board_api.h"
#include "tusb.h"
#include <stdio.h>
#include "stm32h503xx.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void cdc_task(void);


void hw_init(void)
{
  board_init();

  #define FSDEV_REG USB_DRD_FS_NS
  #define USB_CNTR_FRES USB_CNTR_USBRST
  #define FSDEV_EP_COUNT 8

  // Follow the RM mentions to use a special ordering of PDWN and FRES
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }

  // Perform USB peripheral reset
  FSDEV_REG->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }

  FSDEV_REG->CNTR &= ~USB_CNTR_PDWN;

  // Wait startup time, for F042 and F070, this is <= 1 us.
  for (volatile uint32_t i = 0; i < 200; i++) { // should be a few us
    asm("NOP");
  }
  FSDEV_REG->CNTR = 0; // Enable USB

  // FIXME for some reason if I don't read back CNTR the CHEP registers don't
  // work.  WTF?  Is this a hardware quirk or a problem with compiler
  // optimization?  Maybe a compiler optimization issue, since removing the
  // volatile below prevents it from working.
  volatile uint32_t reg = FSDEV_REG->CNTR;
  (void)reg;

#if 0
  // BTABLE register does not exist any more on 32-bit bus devices
  FSDEV_REG->BTABLE = FSDEV_BTABLE_BASE;
#endif

  FSDEV_REG->ISTR = 0; // Clear pending interrupts

  // Reset endpoints to disabled
  //for (uint32_t i = 0; i < FSDEV_EP_COUNT; i++) {
    // This doesn't clear all bits since some bits are "toggle", but does set the type to DISABLED.
    //ep_write(i, 0u, false);
    //FSDEV_REG->ep[i].reg = (fsdev_bus_t) 0;
    FSDEV_REG->CHEP0R = 0;
    FSDEV_REG->CHEP1R = 0;
    FSDEV_REG->CHEP2R = 0;
    FSDEV_REG->CHEP3R = 0;
    FSDEV_REG->CHEP4R = 0;
    FSDEV_REG->CHEP5R = 0;
    FSDEV_REG->CHEP6R = 0;
    FSDEV_REG->CHEP7R = 0;
  //}

  FSDEV_REG->CHEP0R = 0x2220;


  //handle_bus_reset(rhport);
    // Set up endpoint registers
    //FSDEV_REG->DADDR = 0u; // disable USB Function
    FSDEV_REG->DADDR = USB_DADDR_EF; // Enable USB Function

  // Enable pull-up if supported
  //dcd_connect(rhport);
    FSDEV_REG->BCDR |= USB_BCDR_DPPU;
}

/*------------- MAIN -------------*/
int main(void) {

  hw_init();
  printf("Hello, world\r\n");
  printf("CHEP0R == %.8lX\r\n", USB_DRD_FS_NS->CHEP0R);
  printf("CHEP1R == %.8lX\r\n", USB_DRD_FS_NS->CHEP1R);
  printf("CHEP2R == %.8lX\r\n", USB_DRD_FS_NS->CHEP2R);
  printf("CHEP3R == %.8lX\r\n", USB_DRD_FS_NS->CHEP3R);
  printf("CHEP4R == %.8lX\r\n", USB_DRD_FS_NS->CHEP4R);
  printf("CHEP5R == %.8lX\r\n", USB_DRD_FS_NS->CHEP5R);
  printf("CHEP6R == %.8lX\r\n", USB_DRD_FS_NS->CHEP6R);
  printf("CHEP7R == %.8lX\r\n", USB_DRD_FS_NS->CHEP7R);

  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  /*
  if (board_init_after_tusb) {
    printf("board_init_after_usb\r\n");
    board_init_after_tusb();
  }
  else
  {
    printf("NO AFTER USB\r\n");
  }
  */

  printf("after tusb\r\n");
  printf("CHEP0R == %.8lX\r\n", USB_DRD_FS_NS->CHEP0R);
  printf("CHEP1R == %.8lX\r\n", USB_DRD_FS_NS->CHEP1R);
  printf("CHEP2R == %.8lX\r\n", USB_DRD_FS_NS->CHEP2R);
  printf("CHEP3R == %.8lX\r\n", USB_DRD_FS_NS->CHEP3R);
  printf("CHEP4R == %.8lX\r\n", USB_DRD_FS_NS->CHEP4R);
  printf("CHEP5R == %.8lX\r\n", USB_DRD_FS_NS->CHEP5R);
  printf("CHEP6R == %.8lX\r\n", USB_DRD_FS_NS->CHEP6R);
  printf("CHEP7R == %.8lX\r\n", USB_DRD_FS_NS->CHEP7R);
  while (1) {
    tud_task(); // tinyusb device task
    led_blinking_task();

    cdc_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void) {
  // connected() check for DTR bit
  // Most but not all terminal client set this when making connection
  // if ( tud_cdc_connected() )
  {
    // connected and there are data available
    if (tud_cdc_available()) {
      // read data
      char buf[64];
      uint32_t count = tud_cdc_read(buf, sizeof(buf));

      for (uint32_t i=0; i<count; i++)
      {
        buf[i] = buf[i] ^ 0x20;
      }
      tud_cdc_write(buf, count);
      tud_cdc_write_flush();
    }
  }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  (void) itf;
  (void) rts;

  // TODO set some indicator
  if (dtr) {
    // Terminal connected
  } else {
    // Terminal disconnected
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) {
  (void) itf;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
