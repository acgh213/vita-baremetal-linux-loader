#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <baremetal/pervasive.h>
#include <baremetal/cdram.h>
#include <baremetal/gpio.h>
#include <baremetal/i2c.h>
#include <baremetal/syscon.h>
#include <baremetal/sysroot.h>
#include <baremetal/display.h>
#include <baremetal/ctrl.h>
#include <baremetal/msif.h>
#include <baremetal/font.h>
#include <baremetal/uart.h>
#include <baremetal/utils.h>
#include "ff.h"

#define LINUX_DIR	"/linux/"

#define ZIMAGE 		LINUX_DIR "zImage"
#define VITA1000_DTB 	LINUX_DIR "vita1000.dtb"
#define VITA2000_DTB 	LINUX_DIR "vita2000.dtb"
#define PSTV_DTB 	LINUX_DIR "pstv.dtb"
#define FALLBACK_DTB	LINUX_DIR "vita.dtb"

#define DTB_LOAD_ADDR	0x4A000000
#define LINUX_LOAD_ADDR	0x44000000

#define CPU123_WAIT_BASE 0x1F007F00

extern unsigned int _bss_start;
extern unsigned int _bss_end;
extern void l1_cache_clean_invalidate(void);

static const unsigned char msif_key[32] = {
	0xD4, 0x19, 0xA2, 0xEB, 0x9D, 0x61, 0xA5, 0x2F,
	0x4F, 0xA2, 0x8B, 0x27, 0xE3, 0x2F, 0xCD, 0xD7,
	0xE0, 0x04, 0x8D, 0x44, 0x3D, 0x63, 0xC9, 0x2C,
	0x0B, 0x27, 0x13, 0x55, 0x41, 0xD9, 0x2E, 0xC4
};

static void LOG(const char *str, ...)
{
	static int console_y = 200;
	char buf[256];
	va_list argptr;

	va_start(argptr, str);
	vsnprintf(buf, sizeof(buf), str, argptr);
	va_end(argptr);

	uart_print(0, buf);
	font_draw_string(10, console_y+=20, WHITE, buf);
}

static void cpu123_wait(unsigned int cpu_id)
{
	volatile unsigned int val, *base = (unsigned int *)CPU123_WAIT_BASE;
	/* Per-core breadcrumbs at 0x1F007F40 + cpu_id*16 (read via devmem
	 * after boot; 4 words per core, 3 cores -> 0x40..0x6F):
	 *   +0  0xC0DE000n  reached the park loop
	 *   +1  raw MPIDR   affinity of this core
	 *   +2  0xC0DE000n  consumed mailbox, jumped to secondary_startup
	 *   +3  wake count  incremented each time wfe() returns
	 * 0xDEAD000n in +0 means the jump RETURNED (should never happen). */
	volatile unsigned int *bc = (unsigned int *)(CPU123_WAIT_BASE + 0x40 + cpu_id * 16);
	unsigned int mpidr;

	__asm__ __volatile__("mrc p15, 0, %0, c0, c0, 5" : "=r"(mpidr));

	bc[0] = 0xC0DE0000 | cpu_id;
	bc[1] = mpidr;
	bc[2] = 0;
	bc[3] = 0;
	dsb();

	base[cpu_id] = 0;
	dsb();

	while (1) {
		/* Check BEFORE parking: the kernel's SEV can arrive while we
		 * are still executing, and a stale event register makes wfe
		 * a no-op -- a wake-then-read ordering can sleep through a
		 * mailbox write that already happened. */
		val = base[cpu_id];
		if (val) {
			bc[1] = mpidr;	/* re-capture at jump time: park-entry
					 * copies get trampled by syscon frames */
			bc[2] = 0xC0DE0000 | cpu_id;
			dsb();
			((void (*)())val)();
			bc[0] = 0xDEAD0000 | cpu_id;
			base[cpu_id] = 0;
			dsb();
			continue;
		}
		wfe();
		bc[3]++;
	}
}

static FRESULT file_load(const char *path, uintptr_t addr, UINT *nread)
{
	FRESULT res;
	FIL file;
	FSIZE_t size;

	res = f_open(&file, path, FA_READ);
	if (res != FR_OK)
		return res;

	size = f_size(&file);

	res = f_read(&file, (void *)addr, size, nread);
	if (res != FR_OK)
		return res;

	f_close(&file);

	return FR_OK;
}

/* Read SCTLR and report the MMU / cache enable bits at handoff. */
static void dump_handoff_state(void)
{
	volatile uint32_t *l2 = (volatile uint32_t *)0x1A002000;
	uint32_t sctlr, ctrl, aux;

	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));
	ctrl = l2[0x100 / 4];		/* L2X0_CTRL */
	aux  = l2[0x104 / 4];		/* L2X0_AUX_CTRL */

	LOG("HANDOFF SCTLR=0x%08X M=%d C=%d I=%d\n",
	    sctlr, (sctlr >> 0) & 1, (sctlr >> 2) & 1, (sctlr >> 12) & 1);
	LOG("PL310 CTRL=0x%08X (L2en=%d) AUX=0x%08X\n", ctrl, ctrl & 1, aux);
}

/* Clean+invalidate the PL310 L2 cache (Vita base 0x1A002000).
 * CP15 cache ops don't propagate to PL310 (MMIO-controlled).
 * Call with MMU off, system clocks up. */
static void pl310_clean_inv_all(void)
{
	volatile uint32_t *l2 = (volatile uint32_t *)0x1A002000;
	uint32_t ctrl = l2[0x100 / 4];		/* L2X0_CTRL */
	uint32_t aux = l2[0x104 / 4];		/* L2X0_AUX_CTRL */
	uint32_t mask = 0xFF;			/* 8-way associativity */
	uint32_t pending;

	LOG("L2 clean: L2en=%d (CTRL bit0)\n", ctrl & 1);

	if (aux & (1 << 16))			/* 16-way associativity? */
		mask = 0xFFFF;

	l2[0x7FC / 4] = mask;			/* L2X0_CLEAN_INV_WAY */
	pending = l2[0x7FC / 4];		/* did the op actually engage? */
	LOG("CLEAN_INV_WAY wrote 0x%X, readback 0x%X\n", mask, pending);
	while (l2[0x7FC / 4] != 0)		/* wait for completion */
		;
	l2[0x730 / 4] = 0;			/* L2X0_CACHE_SYNC */
	__asm__ __volatile__("dsb" ::: "memory");
}

/* CRC32 (IEEE 802.3, poly 0xEDB88320) over a memory region. */
static unsigned int crc32_buf(const void *data, unsigned int len)
{
	static unsigned int table[256];
	static int table_ready;
	unsigned int crc = 0xFFFFFFFF;
	const unsigned char *p = data;
	unsigned int i;

	if (!table_ready) {
		for (i = 0; i < 256; i++) {
			unsigned int c = i;
			int k;
			for (k = 0; k < 8; k++)
				c = (c & 1) ? (0xEDB88320 ^ (c >> 1)) : (c >> 1);
			table[i] = c;
		}
		table_ready = 1;
	}

	while (len--)
		crc = table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);

	return crc ^ 0xFFFFFFFF;
}

static FRESULT file_load_log(const char *path, uintptr_t addr)
{
	FRESULT res;
	UINT nread;

	LOG("Loading '%s'...\n", path);

	res = file_load(path, addr, &nread);
	if (res != FR_OK) {
		LOG("Error loading '%s': %d\n", path, res);
		return res;
	}

	LOG("Loaded '%s' at 0x%08X, size: %dKiB\n", path, addr, nread / 1024);
	LOG("CRC32: 0x%08X\n", crc32_buf((void *)addr, nread));

	return FR_OK;
}

int main(struct sysroot_buffer *sysroot)
{
	FATFS fs;
	FRESULT res;
	unsigned int *bss;
	unsigned int cpu_id = get_cpu_id();

	/* CPUs 1-3 wait for Linux secondary startup */
	if (cpu_id != 0)
		cpu123_wait(cpu_id);

	/* Clear BSS */
	for (bss = &_bss_start; bss < &_bss_end; bss++)
		*bss = 0;

	sysroot_init(sysroot);

	pervasive_clock_enable_gpio();
	pervasive_reset_exit_gpio();
	pervasive_clock_enable_uart(0);
	pervasive_reset_exit_uart(0);
	pervasive_clock_enable_i2c(1);
	pervasive_reset_exit_i2c(1);

	uart_init(0, 115200);

	cdram_enable();
	i2c_init_bus(1);
	syscon_init();

	if (sysroot_model_is_dolce())
		display_init(DISPLAY_TYPE_HDMI);
	else if (sysroot_model_is_vita2k())
		display_init(DISPLAY_TYPE_LCD);
	else
		display_init(DISPLAY_TYPE_OLED);

	LOG("Vita baremetal Linux loader started!\n");

	if (!pervasive_msif_get_card_insert_state()) {
		LOG("Memory card not inserted.\n");
		goto fatal_error;
	}

	msif_init();
	syscon_msif_set_power(1);
	msif_setup(msif_key);

	LOG("Memory card authenticated!\n");

	res = f_mount(&fs, "/", 0);
	if (res != FR_OK) {
		LOG("Error mounting Memory card: %d\n", res);
		goto fatal_error;
	}

	LOG("Memory card mounted!\n");

	res = file_load_log(ZIMAGE, LINUX_LOAD_ADDR);
	if (res != FR_OK)
		goto fatal_error;

	if (sysroot_model_is_vita()) {
		res = file_load_log(VITA1000_DTB, DTB_LOAD_ADDR);
	} else if (sysroot_model_is_vita2k()) {
		res = file_load_log(VITA2000_DTB, DTB_LOAD_ADDR);
	} else if (sysroot_model_is_dolce()) {
		res = file_load_log(PSTV_DTB, DTB_LOAD_ADDR);
	} else {
		LOG("Unsupported Vita model.\n");
		goto fatal_error;
	}

	if (res != FR_OK) {
		LOG("Trying with the fallback DTB...\n");
		res = file_load_log(FALLBACK_DTB, DTB_LOAD_ADDR);
		if (res != FR_OK)
			goto fatal_error;
	}

	LOG("Jumping to Linux!\n");

	/* EXPERIMENT: record the handoff state before the jump.  L1 was
	 * found to be DISABLED (SCTLR C=0 I=0) at handoff, so there is no
	 * stale-L1 state to scrub — and set/way maintenance on a disabled
	 * L1 D-cache HANGS this hardware, so do NOT attempt it.  The only
	 * cache in play is L2 (PL310), cleaned below. */
	dump_handoff_state();

	/* Pre-handoff PL310 L2 clean+invalidate.  The kernel's cache_on
	 * reads stale L2 lines aliasing the zImage/DDR region (left by
	 * VitaOS / the loader's own FAT reads), corrupting its MMU page
	 * tables before decompression even starts.  Do it here, with the
	 * system fully initialized and MMU off. */
	pl310_clean_inv_all();
	LOG("PL310 L2 cleaned.\n");

	((void (*)(int, int, uintptr_t))LINUX_LOAD_ADDR)(0, 0, DTB_LOAD_ADDR);

fatal_error:
	LOG("\nPress X to restart.\n");

	while (1) {
		struct ctrl_data ctrl;
		ctrl_read(&ctrl);
		if (CTRL_BUTTON_HELD(ctrl.buttons, CTRL_CROSS))
			break;
	}

	syscon_reset_device(SYSCON_RESET_TYPE_COLD_RESET, 0);

	return 0;
}
