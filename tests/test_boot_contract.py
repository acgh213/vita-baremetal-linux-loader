#!/usr/bin/env python3
"""Source-level boot-contract regression tests for the Vita loader."""

from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.c"
START = ROOT / "src" / "start.s"


class BootContractTests(unittest.TestCase):
    def test_pl310_clean_precedes_linux_jump(self):
        source = MAIN.read_text()
        self.assertIn("static void pl310_clean_inv_all(void)", source)
        helper_start = source.index("static void pl310_clean_inv_all(void)")
        helper_end = source.index("static FRESULT file_load_log", helper_start)
        helper = source[helper_start:helper_end]
        clean = helper.index("l2[0x7FC / 4] = mask;")
        poll = helper.index("while (l2[0x7FC / 4] != 0)", clean)
        sync = helper.index("l2[0x730 / 4] = 0;", poll)
        barrier = helper.index('__asm__ __volatile__("dsb"', sync)
        clean_call = source.rfind("\tpl310_clean_inv_all();")
        linux_jump = source.rfind("LINUX_LOAD_ADDR)(0, 0, DTB_LOAD_ADDR)")

        self.assertLess(clean, poll)
        self.assertLess(poll, sync)
        self.assertLess(sync, barrier)
        self.assertGreaterEqual(clean_call, 0)
        self.assertGreater(linux_jump, clean_call)

    def test_secondary_mailbox_is_cleared_and_checked_before_wfe(self):
        source = MAIN.read_text()
        start = source.index("static void cpu123_wait(unsigned int cpu_id)")
        end = source.index("static FRESULT file_load", start)
        wait = source[start:end]

        self.assertIn("base[cpu_id] = 0;", wait)
        clear = wait.index("base[cpu_id] = 0;")
        barrier = wait.index("dsb();", clear)
        loop = wait.index("while (1)")
        load = wait.index("val = base[cpu_id];", loop)
        jump = wait.index("((void (*)())val)();", load)
        park = wait.index("wfe();", load)

        self.assertLess(clear, loop)
        self.assertLess(clear, barrier)
        self.assertLess(barrier, loop)
        self.assertLess(loop, load)
        self.assertLess(load, jump)
        self.assertLess(jump, park)

    def test_stack_reserves_top_0x100_bytes_for_mailboxes(self):
        source = START.read_text()
        stack_top = source.index("mov sp, #0x00008000")
        self.assertIn("sub sp, sp, #0x100", source)
        reserve = source.index("sub sp, sp, #0x100")
        partition = source.index("sub sp, r1, lsl #13")

        self.assertLess(stack_top, reserve)
        self.assertLess(reserve, partition)

    def test_production_source_has_no_smp_diagnostics(self):
        source = MAIN.read_text() + START.read_text()
        for marker in (
            "0xC0DE0000",
            "0xDEAD0000",
            "0xAB000000",
            "diagnostic_vector",
            "dump_handoff_state",
        ):
            self.assertNotIn(marker, source)


if __name__ == "__main__":
    unittest.main()
