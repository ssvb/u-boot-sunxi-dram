/*
 * Copyright (c) 2014 Siarhei Siamashka <siarhei.siamashka@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

.syntax unified
.text
.arm
.arch armv7a
.p2align 2

.globl _start_fel
.globl s_init
.globl __bss_start
.globl __bss_end

_start_fel:
	/* Relocate stack to the 0x8000-0xBFFF area */
	mov	r0, #0xC000
	str	sp, [r0, #-4]!
	str	lr, [r0, #-4]!
	adr	lr, _exit_fel /* Return back to '_exit_fel' */
	mov	sp, r0

	/* Erase the BSS segment */
	ldr	r0, =__bss_start
	ldr	r1, =__bss_end
	mov	r2, #0
0:	cmp	r0, r1
	strbne	r2, [r0], #1
	bne	0b

	/* Pass control to the 's_init()' function */
	b	s_init

_exit_fel:
	/* Relocate stack back and return */
	mov	r0, #0xC000
	ldr	sp, [r0, #-4]!
	ldr	lr, [r0, #-4]!
	bx	lr
