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

	# Keep 0x7F00-0x7FFF free for Linux secondary mailboxes, then
	# partition the remaining scratchpad stacks by CPU ID.
	mov sp, #0x00008000
	sub sp, sp, #0x100
	sub sp, r1, lsl #13

	# Jump to the C code
	b main
