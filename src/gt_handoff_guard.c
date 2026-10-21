#include <stdint.h>

extern void LOG(const char *str, ...);

/* Read-only telemetry immediately before the Linux handoff.  This deliberately
 * does not derive a clock ratio or touch any timer, clock, or PMU register. */
void gt_handoff_guard(void)
{
	volatile uint32_t * const gt_control =
		(volatile uint32_t *)0x1a000208;
	volatile uint32_t * const gt_selector =
		(volatile uint32_t *)0xe3103000;
	volatile uint32_t * const gt_divisor =
		(volatile uint32_t *)0xe3103004;
	uint32_t control = *gt_control;
	uint32_t selector = *gt_selector;
	uint32_t divisor = *gt_divisor;

	if (control & 1) {
		LOG("GT_HANDOFF ON control=0x%08X selector=0x%08X divisor=0x%08X "
		    "ratio REFUSED (read-only guard)\n",
		    control, selector, divisor);
	} else {
		LOG("GT_HANDOFF OFF control=0x%08X selector=0x%08X divisor=0x%08X "
		    "ratio REFUSED (GT disabled)\n",
		    control, selector, divisor);
	}
}
