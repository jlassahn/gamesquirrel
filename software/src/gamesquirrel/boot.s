/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

/*
  Modified from startup_stm32h523xx.s
  Which is Copyright (c) 2023 STMicroelectronics.
*/

.syntax unified
.cpu cortex-m33
.thumb

.global Reset_Handler
.section .text.Reset_Handler
.type Reset_Handler, %function

Reset_Handler:
	ldr   r0, = _estack
	mov   sp, r0

	/* Initialize the data segment from flash. */
	ldr r0, =_sdata
	ldr r1, =_edata
	ldr r2, =_sidata
	movs r3, #0
	b datacopy_test

datacopy_body:
	ldr r4, [r2, r3]
	str r4, [r0, r3]
	adds r3, r3, #4

datacopy_test:
	adds r4, r0, r3
	cmp r4, r1
	bcc datacopy_body

	/* Zero initialize the bss segment. */
	ldr r2, =_sbss
	ldr r4, =_ebss
	movs r3, #0
	b zerofill_test

zerofill_body:
	str  r3, [r2]
	adds r2, r2, #4

zerofill_test:
	cmp r2, r4
	bcc zerofill_body

	/* Call static constructors. */
	bl __libc_init_array

	bl main

	/* Infinite loop on exit from main. */
	b .

