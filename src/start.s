	.cpu cortex-a9
	.align 4
	.code 32

	.text

	.global _start

# r0 = Sysroot buffer paddr
_start:
	# Disable interrupts and enter System mode
	cpsid aif, #0x1F

	# Get CPU ID
	mrc p15, 0, r1, c0, c0, 5
	ands r1, r1, #0xF

	# Setup the SP at the end of the scratchpad
	mov sp, #0x00008000
	sub sp, r1, lsl #13

	# Jump to the C code
	b main

	@ SMP DIAGNOSTIC: install low exception vectors for parked secondary CPUs.
	@ VBAR remains valid after the jump into secondary_startup because this
	@ payload stays resident at 0x40300000. Linux replaces VBAR during cpu_init.
	.global smp_diag_install_vectors
	.type smp_diag_install_vectors, %function
smp_diag_install_vectors:
	ldr r0, =smp_diag_vectors
	mcr p15, 0, r0, c12, c0, 0	@ VBAR
	mrc p15, 0, r1, c1, c0, 0	@ SCTLR
	bic r1, r1, #(1 << 13)		@ use VBAR, not high vectors
	mcr p15, 0, r1, c1, c0, 0
	isb
	bx lr

	@ Each vector is one ARM instruction; VBAR requires 32-byte alignment.
	.align 5
smp_diag_vectors:
	b smp_diag_reset
	b smp_diag_undef
	b smp_diag_svc
	b smp_diag_pabort
	b smp_diag_dabort
	b smp_diag_reserved
	b smp_diag_irq
	b smp_diag_fiq

smp_diag_reset:
	mov r12, #0
	b smp_diag_nonabort
smp_diag_undef:
	mov r12, #1
	b smp_diag_nonabort
smp_diag_svc:
	mov r12, #2
	b smp_diag_nonabort
smp_diag_pabort:
	mov r12, #3
	mrc p15, 0, r2, c5, c0, 1	@ IFSR
	mrc p15, 0, r3, c6, c0, 2	@ IFAR
	b smp_diag_record
smp_diag_dabort:
	mov r12, #4
	mrc p15, 0, r2, c5, c0, 0	@ DFSR
	mrc p15, 0, r3, c6, c0, 0	@ DFAR
	b smp_diag_record
smp_diag_reserved:
	mov r12, #5
	b smp_diag_nonabort
smp_diag_irq:
	mov r12, #6
	b smp_diag_nonabort
smp_diag_fiq:
	mov r12, #7
smp_diag_nonabort:
	mov r2, #0
	mov r3, #0
smp_diag_record:
	mrc p15, 0, r0, c0, c0, 5	@ MPIDR
	and r0, r0, #0xf
	ldr r1, =0x1F007FB0
	add r1, r1, r0, lsl #4		@ 16-byte record per core
	ldr r4, =0xAB000000
	orr r4, r4, r12, lsl #8		@ vector number in bits 10:8
	orr r4, r4, r0			@ CPU number in low nibble
	str r4, [r1]			@ marker
	str r2, [r1, #4]			@ IFSR/DFSR
	str r3, [r1, #8]			@ IFAR/DFAR
	str lr, [r1, #12]			@ exception return address
	dsb
1:	wfe
	b 1b
