// SPDX-License-Identifier: MIT
/*
 * One-shot Cortex-A9 CPU:PERIPHCLK ratio probe for the PS Vita loader.
 *
 * Clock registers are read-only. The probe refuses to run unless the inherited
 * ARM BaseClk state is selector 7, divisor 0 and the Global Timer is already
 * enabled with prescaler field zero. It temporarily enables the PMU cycle
 * counter without resetting it, then restores the original PMU enable state.
 */

#include <stdint.h>

#define BASECLK_ARM_SELECTOR_ADDR 0xE3103000u
#define BASECLK_ARM_DIVISOR_ADDR  0xE3103004u
#define GT_BASE_ADDR              0x1A000200u
#define GT_CONTROL_OFFSET         0x08u

#define GT_CONTROL_ENABLE         (1u << 0)
#define PMCR_E  (1u << 0)
#define PMCR_P  (1u << 1)
#define PMCR_C  (1u << 2)
#define PMCR_D  (1u << 3)
#define PMCR_X  (1u << 4)
#define PMCR_DP (1u << 5)
#define PMCR_RESTORABLE_MASK (PMCR_E | PMCR_D | PMCR_X | PMCR_DP)

#define PMU_CYCLE_COUNTER_BIT (1u << 31)
#define PMU_EVENT_COUNTER_MASK 0x3fu
#define PMU_COUNTER_ENABLE_MASK (PMU_CYCLE_COUNTER_BIT | PMU_EVENT_COUNTER_MASK)
#define CPSR_IRQ_MASK (1u << 7)
#define CPSR_FIQ_MASK (1u << 6)
#define SAMPLE_COUNT 7u
#define SAMPLE_GT_TICKS 1000000u
#define MAX_SAMPLE_GT_TICKS 2000000u
#define MAX_GT_POLLS 5000000u
#define MAX_GT_READ_RETRIES 4u

typedef void (*clock_ratio_emit_fn)(const char *format, ...);

typedef struct {
	uint32_t index;
	uint32_t lo;
	uint32_t hi;
	uint32_t gt;
} ratio_record;

static inline void instruction_barrier(void)
{
	__asm__ __volatile__("isb" ::: "memory");
}

static inline void data_barrier(void)
{
	__asm__ __volatile__("dsb" ::: "memory");
}

static inline uint32_t interrupts_save_disable(void)
{
	uint32_t cpsr;

	__asm__ __volatile__("mrs %0, cpsr\n	cpsid if" : "=r"(cpsr) : : "memory");
	instruction_barrier();
	return cpsr;
}

static inline void interrupts_restore(uint32_t saved_cpsr)
{
	if (!(saved_cpsr & CPSR_IRQ_MASK))
		__asm__ __volatile__("cpsie i" ::: "memory");
	if (!(saved_cpsr & CPSR_FIQ_MASK))
		__asm__ __volatile__("cpsie f" ::: "memory");
	instruction_barrier();
}

static inline uint32_t mmio_read32(uintptr_t address)
{
	return *(volatile uint32_t *)address;
}

static int global_timer_read(uint64_t *value)
{
	volatile uint32_t *gt = (volatile uint32_t *)GT_BASE_ADDR;
	uint32_t read_attempt;
	uint32_t upper;
	uint32_t old_upper;
	uint32_t lower;

	for (read_attempt = 0; read_attempt < MAX_GT_READ_RETRIES; read_attempt++) {
		old_upper = gt[1];
		lower = gt[0];
		upper = gt[1];
		if (upper == old_upper) {
			*value = ((uint64_t)upper << 32) | lower;
			return 0;
		}
	}

	return -1;
}

static inline uint32_t pmcr_read(void)
{
	uint32_t value;

	__asm__ __volatile__("mrc p15, 0, %0, c9, c12, 0" : "=r"(value));
	return value;
}

static inline void pmcr_write(uint32_t value)
{
	instruction_barrier();
	__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 0" : : "r"(value) : "memory");
	instruction_barrier();
}

static inline uint32_t pmcntenset_read(void)
{
	uint32_t value;

	__asm__ __volatile__("mrc p15, 0, %0, c9, c12, 1" : "=r"(value));
	return value;
}

static inline void pmcntenset_write(uint32_t value)
{
	__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 1" : : "r"(value) : "memory");
	instruction_barrier();
}

static inline void pmcntenclr_write(uint32_t value)
{
	__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 2" : : "r"(value) : "memory");
	instruction_barrier();
}

static inline uint32_t pmovsr_read(void)
{
	uint32_t value;

	__asm__ __volatile__("mrc p15, 0, %0, c9, c12, 3" : "=r"(value));
	return value;
}

static inline void pmovsr_write(uint32_t value)
{
	__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 3" : : "r"(value) : "memory");
	instruction_barrier();
}

static inline uint32_t pmintenset_read(void)
{
	uint32_t value;

	__asm__ __volatile__("mrc p15, 0, %0, c9, c14, 1" : "=r"(value));
	return value;
}

static inline void pmintenset_write(uint32_t value)
{
	__asm__ __volatile__("mcr p15, 0, %0, c9, c14, 1" : : "r"(value) : "memory");
	instruction_barrier();
}

static inline void pmintenclr_write(uint32_t value)
{
	__asm__ __volatile__("mcr p15, 0, %0, c9, c14, 2" : : "r"(value) : "memory");
	instruction_barrier();
}

static inline uint32_t pmccntr_read(void)
{
	uint32_t value;

	instruction_barrier();
	__asm__ __volatile__("mrc p15, 0, %0, c9, c13, 0" : "=r"(value));
	return value;
}

static uint32_t u32_delta(uint32_t start, uint32_t end)
{
	return end - start;
}

static uint64_t u64_delta(uint64_t start, uint64_t end)
{
	return end - start;
}

int clock_ratio_probe_run(clock_ratio_emit_fn emit)
{
	uint32_t selector = mmio_read32(BASECLK_ARM_SELECTOR_ADDR);
	uint32_t divisor = mmio_read32(BASECLK_ARM_DIVISOR_ADDR);
	uint32_t gt_control = mmio_read32(GT_BASE_ADDR + GT_CONTROL_OFFSET);
	uint32_t saved_cpsr;
	uint32_t saved_pmcr;
	uint32_t saved_pmcntenset;
	uint32_t saved_pmintenset;
	ratio_record records[SAMPLE_COUNT];
	uint32_t sample_index;
	uint32_t pmu_mutated = 0;
	uint32_t cycle_overflow_seen = 0;
	uint32_t error_index = 0;
	uint32_t overflow_status;
	int result = 0;

	if (selector != 7u) {
		emit("CLOCK_RATIO_ERROR selector=%u expected=7\n", selector);
		return -1;
	}
	if (divisor != 0u) {
		emit("CLOCK_RATIO_ERROR divisor=%u expected=0\n", divisor);
		return -2;
	}
	if (gt_control != GT_CONTROL_ENABLE) {
		emit("CLOCK_RATIO_ERROR gt_control=0x%08X expected=0x00000001\n",
		     gt_control);
		return -3;
	}

	saved_cpsr = interrupts_save_disable();
	saved_pmcr = pmcr_read();
	saved_pmcntenset = pmcntenset_read() & PMU_COUNTER_ENABLE_MASK;
	saved_pmintenset = pmintenset_read() & PMU_COUNTER_ENABLE_MASK;
	if (pmovsr_read() & PMU_CYCLE_COUNTER_BIT) {
		result = -7;
		goto restore_pmu;
	}

	/* Run only PMCCNTR. Do not reset any PMU counter. */
	pmu_mutated = 1;
	pmintenclr_write(PMU_COUNTER_ENABLE_MASK);
	pmcntenclr_write(saved_pmcntenset);
	pmcr_write((saved_pmcr & PMCR_RESTORABLE_MASK & ~(PMCR_P | PMCR_C | PMCR_D)) |
		   PMCR_E);
	pmcntenset_write(PMU_CYCLE_COUNTER_BIT);

	for (sample_index = 0; sample_index < SAMPLE_COUNT; sample_index++) {
		uint32_t pmu_outer_start;
		uint32_t pmu_inner_start;
		uint32_t pmu_inner_end;
		uint32_t pmu_outer_end;
		uint64_t gt_start;
		uint64_t gt_end;
		uint64_t gt_elapsed;
		uint32_t selector_after;
		uint32_t divisor_after;
		uint32_t gt_control_after;
		uint32_t poll_count = 0;

		data_barrier();
		instruction_barrier();

		pmu_outer_start = pmccntr_read();
		if (global_timer_read(&gt_start)) {
			error_index = sample_index;
			result = -6;
			goto restore_pmu;
		}
		pmu_inner_start = pmccntr_read();

		for (;;) {
			if (global_timer_read(&gt_end)) {
				error_index = sample_index;
				result = -6;
				goto restore_pmu;
			}
			gt_elapsed = u64_delta(gt_start, gt_end);
			if (gt_elapsed >= SAMPLE_GT_TICKS)
				break;
			poll_count++;
			if (poll_count >= MAX_GT_POLLS) {
				error_index = sample_index;
				result = -5;
				goto restore_pmu;
			}
		}

		pmu_inner_end = pmccntr_read();
		if (global_timer_read(&gt_end)) {
			error_index = sample_index;
			result = -6;
			goto restore_pmu;
		}
		pmu_outer_end = pmccntr_read();
		gt_elapsed = u64_delta(gt_start, gt_end);
		if (gt_elapsed > MAX_SAMPLE_GT_TICKS) {
			error_index = sample_index;
			result = -8;
			goto restore_pmu;
		}
		if (pmovsr_read() & PMU_CYCLE_COUNTER_BIT) {
			cycle_overflow_seen = 1;
			result = -7;
			goto restore_pmu;
		}

		selector_after = mmio_read32(BASECLK_ARM_SELECTOR_ADDR);
		divisor_after = mmio_read32(BASECLK_ARM_DIVISOR_ADDR);
		gt_control_after = mmio_read32(GT_BASE_ADDR + GT_CONTROL_OFFSET);
		if (selector_after != selector || divisor_after != divisor ||
		    gt_control_after != gt_control) {
			error_index = sample_index;
			result = -4;
			goto restore_pmu;
		}

		records[sample_index].index = sample_index;
		records[sample_index].lo = u32_delta(pmu_inner_start, pmu_inner_end);
		records[sample_index].hi = u32_delta(pmu_outer_start, pmu_outer_end);
		records[sample_index].gt = (uint32_t)gt_elapsed;
	}

restore_pmu:
	if (pmu_mutated) {
		overflow_status = pmovsr_read();
		if (overflow_status & PMU_CYCLE_COUNTER_BIT)
			cycle_overflow_seen = 1;
		pmcntenclr_write(PMU_COUNTER_ENABLE_MASK);
		pmcr_write(saved_pmcr & PMCR_RESTORABLE_MASK);
		pmcntenset_write(saved_pmcntenset);
		if (cycle_overflow_seen)
			pmovsr_write(PMU_CYCLE_COUNTER_BIT);
		pmintenset_write(saved_pmintenset);
	}
	interrupts_restore(saved_cpsr);

	if (result == -5)
		emit("CLOCK_RATIO_ERROR timeout i=%u\n", error_index);
	else if (result == -6)
		emit("CLOCK_RATIO_ERROR gt_read i=%u\n", error_index);
	else if (result == -7)
		emit("CLOCK_RATIO_ERROR pmu_overflow i=%u\n", error_index);
	else if (result == -8)
		emit("CLOCK_RATIO_ERROR gt_interval i=%u\n", error_index);
	else if (result == -4)
		emit("CLOCK_RATIO_ERROR state i=%u\n", error_index);
	else if (result == 0) {
		for (sample_index = 0; sample_index < SAMPLE_COUNT; sample_index++)
			emit("CLOCK_RATIO i=%u s=%u d=%u lo=%u hi=%u gt=%u\n",
			     records[sample_index].index, selector, divisor,
			     records[sample_index].lo, records[sample_index].hi,
			     records[sample_index].gt);
	}

	return result;
}
