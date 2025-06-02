	.cpu cortex-m0
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 2
	.eabi_attribute 34, 0
	.eabi_attribute 18, 4
	.file	"test.c"
	.text
	.global	__aeabi_idiv
	.align	1
	.p2align 2,,3
	.global	test
	.arch armv6s-m
	.syntax unified
	.code	16
	.thumb_func
	.fpu softvfp
	.type	test, %function
test:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3
	push	{r4, lr}
	ldr	r1, [r3]
	ldr	r2, [r3, #4]
	ldr	r4, .L3+4
	muls	r2, r1
	@ sp needed
	str	r2, [r4]
	ldr	r0, [r3]
	ldr	r1, [r3, #4]
	bl	__aeabi_idiv
	str	r0, [r4, #4]
	pop	{r4, pc}
.L4:
	.align	2
.L3:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.size	test, .-test
	.global	d
	.global	c
	.global	b
	.global	a
	.data
	.align	2
	.set	.LANCHOR0,. + 0
	.type	a, %object
	.size	a, 4
a:
	.word	7
	.type	b, %object
	.size	b, 4
b:
	.word	3
	.bss
	.align	2
	.set	.LANCHOR1,. + 0
	.type	c, %object
	.size	c, 4
c:
	.space	4
	.type	d, %object
	.size	d, 4
d:
	.space	4
	.ident	"GCC: (GNU Arm Embedded Toolchain 10.3-2021.07) 10.3.1 20210621 (release)"
