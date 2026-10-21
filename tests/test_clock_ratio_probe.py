#!/usr/bin/env python3
"""Build-contract tests for the freestanding Cortex-A9 ratio probe."""

from __future__ import annotations

import re
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "clock_ratio_probe.c"


class ClockRatioProbeBuildTests(unittest.TestCase):
    def test_arm_object_contains_required_pmu_operations_and_wire_schema(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            obj = Path(temp_dir) / "clock_ratio_probe.o"
            compile_result = subprocess.run(
                [
                    "arm-linux-gnueabihf-gcc",
                    "-std=gnu11",
                    "-mcpu=cortex-a9",
                    "-marm",
                    "-O2",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-ffreestanding",
                    "-fno-builtin",
                    "-c",
                    str(SOURCE),
                    "-o",
                    str(obj),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(compile_result.returncode, 0, compile_result.stderr)

            disassembly = subprocess.run(
                ["arm-linux-gnueabihf-objdump", "-dr", str(obj)],
                text=True,
                capture_output=True,
                check=True,
            ).stdout
            strings = subprocess.run(
                ["strings", str(obj)],
                text=True,
                capture_output=True,
                check=True,
            ).stdout
            symbols = subprocess.run(
                ["arm-linux-gnueabihf-nm", "-g", str(obj)],
                text=True,
                capture_output=True,
                check=True,
            ).stdout

        arm_register = r"(?:r\d+|sl|fp|ip|lr)"
        required_cp15 = [
            rf"mrc\s+15, 0, {arm_register}, cr9, cr12, \{{0\}}",  # PMCR read
            rf"mrc\s+15, 0, {arm_register}, cr9, cr12, \{{1\}}",  # PMCNTENSET read
            rf"mrc\s+15, 0, {arm_register}, cr9, cr13, \{{0\}}",  # PMCCNTR read
            rf"mcr\s+15, 0, {arm_register}, cr9, cr12, \{{0\}}",  # PMCR write/restore
            rf"mcr\s+15, 0, {arm_register}, cr9, cr12, \{{1\}}",  # cycle counter enable
            rf"mcr\s+15, 0, {arm_register}, cr9, cr12, \{{2\}}",  # conditional disable/restore
            rf"mrc\s+15, 0, {arm_register}, cr9, cr12, \{{3\}}",  # overflow status read
            rf"mcr\s+15, 0, {arm_register}, cr9, cr12, \{{3\}}",  # overflow status clear
            rf"mrc\s+15, 0, {arm_register}, cr9, cr14, \{{1\}}",  # interrupt enables read
            rf"mcr\s+15, 0, {arm_register}, cr9, cr14, \{{1\}}",  # interrupt enables restore
            rf"mcr\s+15, 0, {arm_register}, cr9, cr14, \{{2\}}",  # interrupts disable
        ]
        for instruction in required_cp15:
            with self.subTest(instruction=instruction):
                self.assertRegex(disassembly, instruction)

        self.assertIn("CLOCK_RATIO i=%u s=%u d=%u lo=%u hi=%u gt=%u", strings)
        self.assertRegex(symbols, r"\bT clock_ratio_probe_run\b")

    def test_loader_integrates_probe_before_linux_handoff(self) -> None:
        makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
        main_source = (ROOT / "src" / "main.c").read_text(encoding="utf-8")

        self.assertIn("clock_ratio_probe.o", makefile)
        declaration = main_source.index("extern int clock_ratio_probe_run")
        call = main_source.index("clock_ratio_probe_run(LOG)")
        handoff = main_source.index('LOG("Jumping to Linux!')
        self.assertLess(declaration, call)
        self.assertLess(call, handoff)

    def test_gt_wait_is_bounded_and_routes_through_pmu_restore(self) -> None:
        source = SOURCE.read_text(encoding="utf-8")

        self.assertIn("#define MAX_GT_POLLS", source)
        self.assertIn("#define MAX_GT_READ_RETRIES", source)
        self.assertIn("poll_count >= MAX_GT_POLLS", source)
        self.assertIn("read_attempt < MAX_GT_READ_RETRIES", source)
        self.assertIn("global_timer_read(&gt_start)", source)
        self.assertIn("global_timer_read(&gt_end)", source)
        self.assertNotIn("do {", source)
        self.assertIn("gt_control != GT_CONTROL_ENABLE", source)
        self.assertIn("gt_control_after != gt_control", source)
        self.assertIn("pmcntenclr_write(saved_pmcntenset);", source)
        self.assertIn("pmcntenset_write(saved_pmcntenset);", source)
        self.assertIn("pmintenclr_write(PMU_COUNTER_ENABLE_MASK);", source)
        self.assertIn("pmintenset_write(saved_pmintenset);", source)
        self.assertIn("pmovsr_read() & PMU_CYCLE_COUNTER_BIT", source)
        self.assertIn("gt_elapsed > MAX_SAMPLE_GT_TICKS", source)
        self.assertIn("pmu_mutated", source)
        self.assertIn("if (cycle_overflow_seen)", source)
        restore_body = source[source.index("restore_pmu:"):]
        self.assertIn("overflow_status = pmovsr_read();", restore_body)
        self.assertIn("overflow_status & PMU_CYCLE_COUNTER_BIT", restore_body)
        self.assertIn("cycle_overflow_seen = 1;", restore_body)
        self.assertIn("interrupts_save_disable()", source)
        self.assertIn("interrupts_restore(saved_cpsr);", source)
        self.assertIn("goto restore_pmu;", source)
        self.assertIn('emit("CLOCK_RATIO_ERROR state i=%u\\n", error_index);', source)
        restore = source.index("restore_pmu:")
        sample_emit = source.index('emit("CLOCK_RATIO i=%u s=%u d=%u lo=%u hi=%u gt=%u')
        mutation = source.index("pmintenclr_write(PMU_COUNTER_ENABLE_MASK);")
        self.assertNotIn('emit("CLOCK_RATIO i=', source[mutation:restore])
        self.assertLess(restore, sample_emit)
        disable = source.index("pmcntenclr_write(PMU_COUNTER_ENABLE_MASK);", restore)
        restore_pmcr = source.index("pmcr_write(saved_pmcr & PMCR_RESTORABLE_MASK);", restore)
        restore_enables = source.index("pmcntenset_write(saved_pmcntenset);", restore)
        restore_interrupts = source.index("interrupts_restore(saved_cpsr);", restore)
        self.assertLess(restore, disable)
        self.assertLess(disable, restore_pmcr)
        self.assertLess(restore_pmcr, restore_enables)
        self.assertLess(restore_enables, restore_interrupts)


if __name__ == "__main__":
    unittest.main()
