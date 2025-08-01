/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#ifndef INCLUDED_USB_H
#define INCLUDED_USB_H

#include <stdbool.h>

void UsbInit(void);
void UsbStart(void);
bool UsbIsActive(void);
int UsbSend(const char *data, int max_bytes);
int UsbReceive(char *data, int max_bytes);
int UsbInterruptCount(void);

#endif

