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

/*
 * File logging.
 *
 * The framebuffer is NOT a usable log channel on this PSTV: FB_ADDR is
 * correctly 0x20000000 and the loader does write there, but the display does
 * not scan that buffer out during the loader phase -- the screen is black until
 * Linux configures fbcon.  So a hold can be running correctly and still show
 * nothing, which is exactly what happened.
 *
 * uart_print() is already called below but no serial adapter is attached to the
 * host.  So the log also goes to a file on the mounted FAT volume, which
 * survives the run and can be retrieved over FTP from VitaOS.
 *
 * FF_FS_READONLY is 0 in FatFs/ffconf.h to enable this.  Writes happen only
 * while VitaOS is destroyed and the loader owns the card, and only to this one
 * small file.
 */
#define VDDG_LOGFILE "/vddg.log"
static int log_file_ready;   /* set by main() once the volume is mounted */

/* FatFs requires a timestamp source when writes are enabled.  The loader has no
 * meaningful clock, and the log is removed at the start of every run, so a fixed
 * valid date is enough to satisfy the directory-update path. */
DWORD get_fattime(void)
{
	return ((DWORD)(2026 - 1980) << 25) | ((DWORD)1 << 21) | ((DWORD)1 << 16);
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

	if (log_file_ready) {
		FIL f;
		UINT bw;
		if (f_open(&f, VDDG_LOGFILE, FA_OPEN_ALWAYS | FA_WRITE) == FR_OK) {
			f_lseek(&f, f_size(&f));
			f_write(&f, buf, (UINT)strlen(buf), &bw);
			f_sync(&f);
			f_close(&f);
		}
	}
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

/*
 * Read-only sweep, v2: short 4-word lines that fit the 1280px framebuffer
 * (glyphs are ~16px wide, so ~78 chars max per line) and multiple read
 * contexts for the +0x10 slot to detect any read-order latching:
 *   v0: reads +0x10 FIRST, then +0x00/+0x04/+0x08
 *   v1: reads +0x0C/+0x14/+0x18/+0x1C (no +0x10)
 *   k:  reads reset/gate/clk +0x10, then +0x10 again as the 4th
 *   m:  reads misc+0xB0, gate+0x24, reset+0x24, then +0x10 again
 *   v2/v3: the rest of the VID block
 */
static void __attribute__((unused)) gpu_vid_sweep2(const char *tag)
{
	volatile uint32_t *vid  = (volatile uint32_t *)PERVASIVE_VID;
	volatile uint32_t *gate = (volatile uint32_t *)PERVASIVE_GATE;
	volatile uint32_t *reset = (volatile uint32_t *)PERVASIVE_RESET;
	volatile uint32_t *clk  = (volatile uint32_t *)PERVASIVE_CLK;
	volatile uint32_t *misc = (volatile uint32_t *)0xE3100000;

	LOG("%s v0: %08X %08X %08X %08X\n", tag,
	    vid[0x10/4], vid[0x00/4], vid[0x04/4], vid[0x08/4]);
	LOG("%s v1: %08X %08X %08X %08X\n", tag,
	    vid[0x0C/4], vid[0x14/4], vid[0x18/4], vid[0x1C/4]);
	LOG("%s k: %08X %08X %08X %08X\n", tag,
	    reset[0x10/4], gate[0x10/4], clk[0x10/4], vid[0x10/4]);
	LOG("%s m: %08X %08X %08X %08X\n", tag,
	    misc[0xB0/4], gate[0x24/4], reset[0x24/4], vid[0x10/4]);
	LOG("%s v2: %08X %08X %08X %08X\n", tag,
	    vid[0x20/4], vid[0x24/4], vid[0x28/4], vid[0x2C/4]);
	LOG("%s v3: %08X %08X %08X %08X\n", tag,
	    vid[0x30/4], vid[0x34/4], vid[0x38/4], vid[0x3C/4]);
}

/*
 * ---------------------------------------------------------------------------
 * VDDG gate (2026-08-31).  Static prerequisites recovered in
 * lab/gpu-re/PHASE1-VDDG-STATIC-RESULT-2026-08-31.md:
 *
 *   sceSysconCtrlVoltageForDriver = SceSysconForDriver NID 0x7F198FA2,
 *   syscon.skprx 0x81005AAC -> syscon command 0x88E, wire length 3,
 *   data bytes [rail, code].  rail 4 = VDDG.
 *
 *   ScePower computes code = 0x8A - vid, with two cached VIDs copied from
 *   the ScePervasiveVid block (0xE3104000):
 *       +0x44 = 0x23  high-clock VID -> code 0x67
 *       +0x48 = 0x2E  low-clock  VID -> code 0x5C
 *
 * Sony's handler is NOT a straight pass-through.  Decision tree at
 * 0x81005AAC (hw = hardware-info word, masked 0x00FF0000):
 *
 *   if (code <= 0x3F):
 *        if (hw >= 0x600000) -> extra checks; else send raw
 *   else:                                  <-- BOTH our codes land here
 *        if (hw >= 0x600000):
 *              if (hw != 0x700000) -> REJECT (0x80250001)
 *              else -> fall into the remap path below
 *        remap path: require (code - 0x4D) <= 0x3F, then code = 0x8C - code
 *
 * So for code 0x67 the byte that actually reaches syscon is either 0x67
 * (raw) or 0x8C-0x67 = 0x25 (remapped), depending on the SoC revision.
 * We do NOT know PSTV's hardware-info word, so we reproduce the tree
 * exactly rather than guessing, and we log the inputs and the resolved
 * byte so the run is interpretable even if the write has no visible
 * effect.
 *
 * libbaremetal note: syscon_short_command_write(cmd, data, length) writes
 * (length + 1) to the wire length field.  Sony's ksceSysconCtrlDeviceReset
 * uses wire length 3 and libbaremetal expresses the same call as
 * syscon_short_command_write(0x88F, ..., 2) -- so its `length` argument is
 * the DATA BYTE COUNT.  Our packet has 2 data bytes [rail, code] and wire
 * length 3, therefore length = 2.
 */
#define SYSCON_CMD_CTRL_VOLTAGE  0x88E
#define VDDG_RAIL                4

/*
 * RESOLVE-ONLY MODE.
 *
 * The 2026-10-20 run of the corrected gate hung the device: it was the first
 * build in which syscon_short_command_write(0x88E, ...) ever executed, and the
 * box went off the network instead of reaching Linux. Sony's gate logic is
 * therefore still unverified, and sending the write again would just re-hang.
 *
 * This build computes and logs the gate decision and returns BEFORE any write,
 * so the polarity fix can be checked with no syscon traffic at all. The rest of
 * main() (platform enable, probes, Linux handoff) is untouched: the box should
 * still boot Linux, which is what lets the CDRAM framebuffer log be read back.
 *
 * Set to 0 to restore write behaviour (and re-read the bounded-wait note first).
 */
#define VDDG_RESOLVE_ONLY 1

/*
 * HOLD MODE (resolve-only only).
 *
 * The 2026-10-20 run booted Linux successfully but the gate log was
 * unobservable: Linux's fbcon repainted the shared framebuffer, and the gate
 * text was gone before it could be read.  FatFs here is compiled read-only
 * (FF_FS_READONLY=1) so writing a log file would mean pulling in write support
 * for a marginal gain, and this box has no serial adapter attached to the host.
 *
 * Instead: after logging the decision, stop before the Linux handoff and park on
 * the existing fatal_error screen ("Press X to restart"), which loops on the
 * controller.  The loader's log then stays on screen indefinitely, so it can be
 * read directly, and X cold-resets back to VitaOS.  No writes, no new machinery.
 */
#define VDDG_HOLD 1

/* Returns the byte Sony would actually put on the wire, or 0xFFFFFFFF if
 * the request would be rejected.  `arm` receives a short label naming the
 * path taken so a hardware run can be interpreted without re-deriving it.
 *
 * Sony's tree has TWO INDEPENDENT gate blocks, each re-reading hardware
 * info.  Reading them as one shared tree is wrong, and an earlier version of
 * this function got the high-code path backwards as a result:
 *
 *   entry (0x81005aac):  cmp r1,#0x3F ; bhi BLOCK_B
 *   BLOCK_A (code <= 0x3F):
 *       bl hw ; and #0xFF0000 ; cmp #0x600000 ; bcs 0x81005b20
 *       fallthrough -> SEND RAW
 *   BLOCK_A_rev (0x81005b20, hw >= 0x600000):
 *       cmp #0x700000 ; beq SEND_RAW
 *       cmp #0x800000 ; bcs 0x81005b32
 *       cmp #0x720000 ; bne ERROR
 *   0x81005b32: cmp r5,#0x3B ; bhi SEND_RAW ; else ERROR
 *
 *   BLOCK_B (code > 0x3F, 0x81005b04):
 *       bl hw ; and #0xFF0000 ; cmp #0x600000 ; bcs 0x81005b3e
 *       fallthrough -> REMAP   (hw < 0x600000)
 *   BLOCK_B_rev (0x81005b3e, hw >= 0x600000):
 *       cmp #0x700000 ; bne SEND_RAW      <-- every non-0x700000 SoC sends RAW
 *       b REMAP                            <-- only 0x700000 remaps
 *   REMAP (0x81005b12):
 *       sub r3,r5,#0x4D ; cmp r3,#0x3F ; bhi ERROR
 *       rsb r5,r5,#0x8C ; b SEND
 *
 * So on PSTV (hw_masked == 0x720000) a high code such as 0x67 is sent RAW,
 * with no rejection and no remap.  The 2026-08-31 run reported "REJECTED by
 * revision gate" -- that was this function's error, not Sony's.
 */
static uint32_t vddg_resolve_code(uint32_t code, uint32_t hw_masked,
				  const char **arm)
{
	if (code <= 0x3F) {
		*arm = "A:raw";
		if (hw_masked >= 0x600000) {
			if (hw_masked == 0x700000)
				return code;
			if (hw_masked == 0x720000 || hw_masked >= 0x800000) {
				/* 0x81005b32 */
				if (code > 0x3B) {
					*arm = "A:raw-720/800(code>3B)";
					return code;
				}
				*arm = "A:reject(code<=3B)";
				return 0xFFFFFFFFu;
			}
			*arm = "A:reject(hw-not-listed)";
			return 0xFFFFFFFFu;
		}
		return code;
	}

	/* code > 0x3F */
	if (hw_masked >= 0x600000 && hw_masked != 0x700000) {
		*arm = "B:raw(hw!=0x700000)";
		return code;
	}

	/* REMAP path: hw < 0x600000, or hw == 0x700000. */
	if ((code - 0x4D) > 0x3F) {
		*arm = "B:reject(remap-range)";
		return 0xFFFFFFFFu;
	}
	*arm = (hw_masked == 0x700000) ? "B:remap(0x700000)" : "B:remap(hw<600000)";
	return 0x8C - code;
}

/*
 * Probe the SGX aperture read-only.  This is the effect check: if the VDDG
 * write is what the window was waiting on, 0xE8400000 should stop reading
 * back as the uniform dark pattern.
 *
 * SAFETY: this payload runs with NO exception vectors installed (start.s
 * sets a stack and branches to main; VBAR still points at whatever VitaOS
 * left).  Under Linux these addresses SIGBUS -- here an imprecise data
 * abort would be unrecoverable and would hang the box, needing a physical
 * power cycle.  So we install a minimal vector table with a data-abort
 * handler that skips the faulting instruction and flags the fault, giving
 * us a survivable read.
 *
 * Note the aborts on this bus are asynchronous/imprecise, so we bracket
 * each read with a dsb to force the fault to land inside our window.
 */
static volatile uint32_t abort_flag;
static uint32_t saved_vbar;

/* Data-abort handler: mark the fault and resume after the faulting insn.
 * lr_abt points to faulting_pc + 8 for data aborts. */
static void __attribute__((naked)) abort_handler(void)
{
	asm volatile(
		"ldr r0, =abort_flag\n	"
		"mov r1, #1\n	"
		"str r1, [r0]\n	"
		"subs pc, lr, #4\n	"
	);
}

static void install_abort_vectors(void)
{
	/*
	 * Build a tiny vector table at 0x1F000000 (scratch, well clear of the
	 * Linux mailboxes at 0x7F00 and the payload itself) and point VBAR at
	 * it.  Every entry is 'ldr pc, [pc, #24]' -> address slot, so the
	 * table is 8 branch words + 8 address words.
	 */
	volatile uint32_t *vt = (volatile uint32_t *)0x1F000000;
	unsigned int i;

	asm volatile("mrc p15, 0, %0, c12, c0, 0" : "=r"(saved_vbar));

	for (i = 0; i < 8; i++) {
		vt[i] = 0xE59FF018;              /* ldr pc, [pc, #24] */
		vt[8 + i] = (uint32_t)abort_handler;
	}

	asm volatile("dsb\n	"
		     "isb\n	"
		     "mcr p15, 0, %0, c12, c0, 0\n	"   /* VBAR = vt */
		     "isb\n	"
		     : : "r"(vt) : "memory");
}

/* Put VBAR back exactly as VitaOS left it, before handing off to Linux. */
static void restore_vectors(void)
{
	asm volatile("dsb\n	"
		     "isb\n	"
		     "mcr p15, 0, %0, c12, c0, 0\n	"
		     "isb\n	"
		     : : "r"(saved_vbar) : "memory");
}

/* Read one word with abort protection.  Returns 1 on success. */
static int safe_read32(uint32_t addr, uint32_t *out)
{
	uint32_t v;

	abort_flag = 0;
	asm volatile("dsb" ::: "memory");
	v = *(volatile uint32_t *)addr;
	asm volatile("dsb" ::: "memory");

	if (abort_flag) {
		*out = 0;
		return 0;
	}
	*out = v;
	return 1;
}

static void sgx_probe(const char *tag)
{
	uint32_t sgx = 0, rb = 0, gb = 0;
	int s, r, g;

	s = safe_read32(0xE8400000, &sgx);   /* the SGX itself */
	r = safe_read32(0xE8000000, &rb);    /* secure SonyRegbus */
	g = safe_read32(0xED800000, &gb);    /* GPU reg bus - positive control */

	LOG("%s sgx=%s%08X rb=%s%08X gb=%s%08X\n", tag,
	    s ? "" : "AB:", sgx,
	    r ? "" : "AB:", rb,
	    g ? "" : "AB:", gb);
}

static int vddg_gate(void)
{
	volatile uint32_t *vid = (volatile uint32_t *)PERVASIVE_VID;
	uint32_t vid_hi, vid_lo, code_hi, wire_hi;
	uint32_t hw, hw_masked;
	const char *arm = "?";

	/* 1. Read the VIDs from the block ScePower actually sources them from. */
	vid_hi = vid[0x44 / 4];
	vid_lo = vid[0x48 / 4];
	LOG("VDDG vid hi=%08X lo=%08X\n", vid_hi, vid_lo);

	/* 2. Sony's revision gate input. */
	hw = (uint32_t)syscon_get_hardware_info();
	hw_masked = hw & 0x00FF0000u;
	LOG("VDDG hw=%08X mask=%08X\n", hw, hw_masked);

	/* 3. Resolve the wire byte the way Sony would. */
	code_hi = 0x8A - vid_hi;
	arm = "?";
	wire_hi = vddg_resolve_code(code_hi, hw_masked, &arm);
	LOG("VDDG code=%02X arm=%s wire=%08X\n", code_hi, arm, wire_hi);
	{
		/* Second cached VID, resolved only for gate evidence. */
		const char *arm_lo = "?";
		uint32_t code_lo = 0x8A - vid_lo;
		uint32_t wire_lo = vddg_resolve_code(code_lo, hw_masked, &arm_lo);
		LOG("VDDG lo=%08X code=%02X arm=%s wire=%08X\n",
		    vid_lo, code_lo, arm_lo, wire_lo);
	}

	sgx_probe("VDDG pre");

#if VDDG_RESOLVE_ONLY
	/*
	 * No write. Log the decision, then either hold so the log can be read off
	 * the screen, or return into the platform enable and Linux handoff.
	 */
	LOG("VDDG RESOLVE-ONLY: no write issued\n");
#if VDDG_HOLD
	LOG("VDDG HOLD: stopping before Linux\n");
	LOG("READ THESE LINES ABOVE\n");
	return 1;      /* main() parks on fatal_error so the screen persists */
#else
	return 0;
#endif
#endif

	if (wire_hi == 0xFFFFFFFFu) {
		/* Sony's own handler would refuse this request.  Do not send it. */
		LOG("VDDG REJECTED by revision gate - no write\n");
		return 0;
	}

	/*
	 * 4. The write.  data = [rail, code] little-endian in the u32:
	 *    byte0 = rail (4), byte1 = resolved code.
	 *    length = 2 data bytes (libbaremetal adds 1 -> wire length 3).
	 */
	LOG("VDDG WRITE cmd=88E rail=4 code=%02X\n", wire_hi);
	syscon_short_command_write(SYSCON_CMD_CTRL_VOLTAGE,
				   VDDG_RAIL | (wire_hi << 8), 2);
	LOG("VDDG write issued\n");

	/* 5. Let the rail settle, then re-probe. */
	delay(200);
	sgx_probe("VDDG post");
}

/*
 * Zero the HDMI framebuffer (CDRAM 0x20000000) before the log starts so
 * stale pixels from the previous boot/VitaOS cannot blend into the glyphs.
 * The loader draws white-on-black without clearing otherwise.
 */
static void clear_framebuffer(void)
{
	volatile uint32_t *fb = (volatile uint32_t *)0x20000000;
	unsigned int i;

	for (i = 0; i < (1280u * 720u * 4u) / 4u; i += 4)
	{
		fb[i] = 0;
		fb[i+1] = 0;
		fb[i+2] = 0;
		fb[i+3] = 0;
	}
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

	/* Fresh per-run log file, then enable file logging. */
	f_unlink(VDDG_LOGFILE);
	log_file_ready = 1;
	LOG("=== loader log start ===\n");

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

	/*
	 * VDDG gate, 2026-08-31.
	 *
	 * Sony's ScePower raises the GPU core rail BEFORE clocking the GPU up
	 * (VDDG write at 0x81003576, then the Xbar SMC).  Every previous
	 * loader experiment skipped the voltage step, so this run reproduces
	 * the real order: clear the screen, install abort vectors so the SGX
	 * probe cannot hang the box, write VDDG, THEN run the platform enable,
	 * then re-probe the aperture.
	 *
	 * NOTE: sweep v2 defined clear_framebuffer() but never called it --
	 * fixed here.
	 */
	clear_framebuffer();
	install_abort_vectors();

	LOG("VDDG GATE %s\n", "2026-08-31");
	if (vddg_gate())
		goto fatal_error;   /* VDDG_HOLD: keep the log on screen */
	gpu_pervasive_on_logged();
	sgx_probe("VDDG final");

	/* Hand VBAR back the way we found it before entering Linux. */
	restore_vectors();

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
