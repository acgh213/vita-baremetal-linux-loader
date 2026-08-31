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

/*
 * VitaOS lowio.skprx implements its GPU-domain-on export as SMC 0x110 with
 * r0=0 (GPU domain) and r1=1 (on).  Keep this wrapper explicit: the current
 * vita-libbaremetal smc() helper incorrectly loads r1-r3 from arg1, which
 * would turn this request into r0=0, r1=0 (off).
 */
static uint32_t gpu_power_domain_on(void)
{
	register uint32_t r0 asm("r0") = 0;
	register uint32_t r1 asm("r1") = 1;
	register uint32_t r2 asm("r2") = 0;
	register uint32_t r3 asm("r3") = 0;
	register uint32_t r12 asm("r12") = 0x110;

	asm volatile(
		"smc #0\n	"
		: "+r"(r0), "+r"(r1), "+r"(r2), "+r"(r3)
		: "r"(r12)
		: "memory", "cc"
	);

	return r0;
}

/*
 * VitaOS SceLowio gpu_on evidence (lowio.skprx.elf, local decompilation):
 *   ScePervasiveForDriver_84EAB5BC -> SMC 0x110, r0=0, r1=1 (power domain on)
 *   ScePervasiveForDriver_39E51AE2 -> gate[0x10] |= (gate & 0xF000F)
 *   ScePervasiveForDriver_8A85E36B -> reset[0x10] &= ~1
 * Physical bases resolved from the lowio init table:
 *   reset  0xE3101000   (s_data_8100B094)
 *   gate   0xE3102000   (s_data_8100B09C)
 *   clocks 0xE3103000   (s_data_8100B0A0)
 *   vid    0xE3104000   (s_data_8100B08C)
 * This is the platform enable only. No SGX register is written.
 */
#define PERVASIVE_RESET 0xE3101000u
#define PERVASIVE_GATE  0xE3102000u
#define PERVASIVE_CLK   0xE3103000u
#define PERVASIVE_VID   0xE3104000u

/*
 * VitaOS SceLowio/ScePower GPU Xbar evidence (SMC table + power.elf disasm):
 *   SMC 0x114 op 2 = smc_bus_freq_op(2, freq) = Set GPU Xbar frequency
 *   lowio export ScePervasiveForDriver_7D02215D = sub_8100001E(2, a1)
 *   ScePower stub 0x810046C0; passes freq index 0 (166 MHz) when GPU clk <= 166.
 * The GPU Xbar is the crossbar between the ARM and the SGX543 register file.
 */
static uint32_t gpu_xbar_freq_set(uint32_t freq)
{
	register uint32_t r0 asm("r0") = 2; /* op 2 = Set GPU Xbar frequency */
	register uint32_t r1 asm("r1") = freq;
	register uint32_t r2 asm("r2") = 0;
	register uint32_t r3 asm("r3") = 0;
	register uint32_t r12 asm("r12") = 0x114;

	asm volatile(
		"smc #0\n\t"
		: "+r"(r0), "+r"(r1), "+r"(r2), "+r"(r3)
		: "r"(r12)
		: "memory", "cc"
	);

	return r0;
}

static void gpu_pervasive_on_logged(void)
{
	volatile uint32_t *reset = (volatile uint32_t *)PERVASIVE_RESET;
	volatile uint32_t *gate  = (volatile uint32_t *)PERVASIVE_GATE;
	volatile uint32_t *clk   = (volatile uint32_t *)PERVASIVE_CLK;
	volatile uint32_t *vid   = (volatile uint32_t *)PERVASIVE_VID;

	LOG("GPU gate: pervasive before reset=%08X gate=%08X clk=%08X vid=%08X\n",
	    reset[0x10/4], gate[0x10/4], clk[0x10/4], vid[0x10/4]);

	LOG("GPU gate: SMC 0x110 domain=0 state=1\n");
	LOG("GPU gate: SMC return=0x%08X\n", gpu_power_domain_on());

	LOG("GPU gate: SMC 0x114 op=2 xbar_freq=0\n");
	LOG("GPU gate: SMC 0x114 return=0x%08X\n", gpu_xbar_freq_set(0));

	gate[0x10/4] |= 0xF000F;
	LOG("GPU gate: gate[0x10] set -> %08X\n", gate[0x10/4]);

	reset[0x10/4] &= ~1u;
	LOG("GPU gate: reset[0x10] clear -> %08X\n", reset[0x10/4]);

	LOG("GPU gate: pervasive after reset=%08X gate=%08X clk=%08X vid=%08X\n",
	    reset[0x10/4], gate[0x10/4], clk[0x10/4], vid[0x10/4]);
}

static void cpu123_wait(unsigned int cpu_id)
{
	volatile unsigned int val, *base = (unsigned int *)CPU123_WAIT_BASE;

	base[cpu_id] = 0;
	dsb();

	while (1) {
		val = base[cpu_id];
		if (val) {
			((void (*)())val)();
			base[cpu_id] = 0;
			dsb();
			continue;
		}
		wfe();
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

/* Clean and invalidate the PL310 L2 cache before entering Linux.  CP15 cache
 * operations do not propagate to this MMIO-controlled outer cache. */
static void pl310_clean_inv_all(void)
{
	volatile uint32_t *l2 = (volatile uint32_t *)0x1A002000;
	uint32_t aux = l2[0x104 / 4];		/* L2X0_AUX_CTRL */
	uint32_t mask = (aux & (1 << 16)) ? 0xFFFF : 0xFF;

	l2[0x7FC / 4] = mask;			/* L2X0_CLEAN_INV_WAY */
	while (l2[0x7FC / 4] != 0)
		;
	l2[0x730 / 4] = 0;			/* L2X0_CACHE_SYNC */
	__asm__ __volatile__("dsb" ::: "memory");
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

	LOG("GPU gate: SMC 0x110 domain=0 state=1\n");
	LOG("GPU gate: return=0x%08X\n", gpu_power_domain_on());
	LOG("GPU gate: pervasive enable sequence\n");
	gpu_pervasive_on_logged();
	LOG("Jumping to Linux!\n");
	pl310_clean_inv_all();

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
