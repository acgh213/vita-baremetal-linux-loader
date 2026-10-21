	.cpu cortex-a9
	.align 2
	.code 32
	.text

	.global l1_cache_clean_invalidate

@ void l1_cache_clean_invalidate(void)
@
@ EXPERIMENT (not yet the fix): ARMv7-correct L1 D-cache INVALIDATE-only by
@ set/way (CLIDR/CCSIDR-driven, mirroring the kernel's v7_flush_dcache_all
@ structure), followed by an I-cache invalidate + barriers.
@
@ Rationale: the loader cleans only the PL310 L2 before handoff and never
@ touches L1.  At handoff SCTLR has C=0/I=0 (L1 disabled) but the cache
@ arrays still physically hold stale lines from VitaOS; those become live
@ again when the kernel's cache_on re-enables L1 without invalidating it.
@
@ We use INVALIDATE-only (c7 c6 2), NOT clean+invalidate (c7 c14 2):
@ the "clean" half forces writeback of stale dirty VitaOS lines, which
@ hangs this hardware at handoff.  We are discarding VitaOS state, not
@ preserving it, so invalidate is both safe and correct.

l1_cache_clean_invalidate:
	stmfd	sp!, {r4-r11, lr}
	dmb
	mrc	p15, 1, r0, c0, c0, 1		@ CLIDR
	ands	r3, r0, #0x7000000		@ extract LoC
	mov	r3, r3, lsr #23			@ left-align LoC field
	beq	1f				@ LoC == 0 -> nothing to clean
	mov	r10, #0				@ start at cache level 0
flush_levels:
	add	r2, r10, r10, lsr #1		@ 3x current level (CLIDR field shift)
	mov	r1, r0, lsr r2			@ cache type bits for this level
	and	r1, r1, #7
	cmp	r1, #2				@ skip if no cache / I-cache only
	blt	skip
	mcr	p15, 2, r10, c0, c0, 0		@ CSSELR = current level
	isb
	mrc	p15, 1, r1, c0, c0, 0		@ CCSIDR
	and	r2, r1, #7			@ log2(line size) - 4 field
	add	r2, r2, #4			@ r2 = log2(line size in bytes)
	movw	r4, #0x3ff
	ands	r4, r4, r1, lsr #3		@ max way index
	clz	r5, r4				@ way field shift
	movw	r7, #0x7fff
	ands	r7, r7, r1, lsr #13		@ max set index
loop1:
	mov	r9, r4				@ working copy of way
loop2:
	orr	r11, r10, r9, lsl r5		@ level + way into set/way operand
	orr	r11, r11, r7, lsl r2		@ index into set/way operand
	mcr	p15, 0, r11, c7, c6, 2		@ invalidate D by set/way (drop, no writeback)
	subs	r9, r9, #1
	bge	loop2
	subs	r7, r7, #1
	bge	loop1
skip:
	add	r10, r10, #2			@ next cache level (CSSELR step)
	cmp	r3, r10
	bgt	flush_levels
1:	mov	r10, #0
	mcr	p15, 2, r10, c0, c0, 0		@ CSSELR back to level 0
	dsb	st
	isb
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ ICIALLU — invalidate I-cache
	dsb
	isb
	ldmfd	sp!, {r4-r11, lr}
	bx	lr
