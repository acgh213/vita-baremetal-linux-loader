#include <stdio.h>
#include <string.h>
#include <stdint.h>
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

/* PL310 L2 cache controller */
#define L2X0_BASE		0x1A002000
#define L2X0_CTRL		0x100
#define L2X0_AUX_CTRL		0x104
#define L2X0_CLEAN_INV_WAY	0x7FC
#define L2X0_CACHE_SYNC		0x730

extern unsigned int _bss_start;
extern unsigned int _bss_end;

static const unsigned char msif_key[32] = {
	0xD4, 0x19, 0xA2, 0xEB, 0x9D, 0x61, 0xA5, 0x2F,
	0x4F, 0xA2, 0x8B, 0x27, 0xE3, 0x2F, 0xCD, 0xD7,
	0xE0, 0x04, 0x8D, 0x44, 0x3D, 0x63, 0xC9, 0x2C,
	0x0B, 0x27, 0x13, 0x55, 0x41, 0xD9, 0x2E, 0xC4
};

static void flush_l1_dcache(void)
{
	unsigned int ccsidr, num_sets, num_ways, log2_line_len, way_shift;
	unsigned int set, way;

	/* Select L1 D-cache */
	asm volatile("mcr p15, 2, %0, c0, c0, 0" : : "r"(0));
	asm volatile("isb");
	asm volatile("mrc p15, 1, %0, c0, c0, 0" : "=r"(ccsidr));

	log2_line_len = (ccsidr & 7) + 4;
	num_sets = ((ccsidr >> 13) & 0x7FFF) + 1;
	num_ways = ((ccsidr >> 3) & 0x3FF) + 1;
	way_shift = __builtin_clz(num_ways - 1);

	for (way = 0; way < num_ways; way++)
		for (set = 0; set < num_sets; set++)
			asm volatile("mcr p15, 0, %0, c7, c14, 2" : :
				"r"((way << way_shift) | (set << log2_line_len)));

	asm volatile("dsb");
	asm volatile("isb");
}

static void flush_and_disable_l2(void)
{
	volatile uint32_t *l2 = (volatile uint32_t *)L2X0_BASE;
	uint32_t aux = l2[L2X0_AUX_CTRL / 4];
	uint32_t ways = (aux & (1 << 16)) ? 16 : 8;
	uint32_t way_mask = (1 << ways) - 1;

	/* Clean and invalidate all ways */
	l2[L2X0_CLEAN_INV_WAY / 4] = way_mask;
	while (l2[L2X0_CLEAN_INV_WAY / 4] & way_mask)
		;
	/* Cache sync */
	l2[L2X0_CACHE_SYNC / 4] = 0;

	/* Do NOT disable L2 - decompressor needs it */
	/* l2[L2X0_CTRL / 4] = 0; */

	asm volatile("dsb");
	asm volatile("isb");
}

static void flush_caches_and_disable_l2(void)
{
	/* Flush L1 D-cache first (writes back to L2) */
	flush_l1_dcache();
	/* Then flush L2 and disable it */
	flush_and_disable_l2();
	/* Invalidate L1 I-cache */
	asm volatile("mcr p15, 0, %0, c7, c5, 0" : : "r"(0));
	asm volatile("dsb");
	asm volatile("isb");
}

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
	while (1) {
		wfe();
		val = base[cpu_id];
		if (val)
			((void (*)())val)();
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

	/* Flush all caches and disable L2 to prevent stale data issues */
	flush_caches_and_disable_l2();

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
