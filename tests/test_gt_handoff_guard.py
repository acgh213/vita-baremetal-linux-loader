#!/usr/bin/env python3
"""TDD contract tests for the read-only GT handoff guard."""

from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "src" / "main.c"
START = ROOT / "src" / "start.s"
GUARD = ROOT / "src" / "gt_handoff_guard.c"
DOCKER_IMAGE = "vitasdk/vitasdk:2026.08"

GT_CONTROL = "0x1a000208"
GT_SELECTOR = "0xe3103000"
GT_DIVISOR = "0xe3103004"


def docker_toolchain(command: str) -> str:
    result = subprocess.run(
        [
            "docker", "run", "--rm",
            "-v", f"{ROOT}:/src:ro",
            "-w", "/src",
            DOCKER_IMAGE,
            "bash", "-lc", command,
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return result.stdout


class GtHandoffGuardContractTests(unittest.TestCase):
    def test_guard_is_integrated_before_linux_jump(self):
        source = MAIN.read_text()
        guard_call = source.rfind("gt_handoff_guard();")
        linux_jump = source.rfind("LINUX_LOAD_ADDR)(0, 0, DTB_LOAD_ADDR)")
        self.assertGreaterEqual(guard_call, 0)
        self.assertGreater(linux_jump, guard_call)

    def test_guard_source_reads_only_the_three_required_registers(self):
        source = GUARD.read_text()
        self.assertEqual(source.count(GT_CONTROL), 1)
        self.assertEqual(source.count(GT_SELECTOR), 1)
        self.assertEqual(source.count(GT_DIVISOR), 1)
        self.assertIn("volatile", source)
        self.assertIn("GT_HANDOFF", source)
        self.assertIn("REFUSED", source)
        self.assertNotRegex(source, r"\b(?:ratio|pmu|clock|timer)\b.*(?:=|\+\+|--)")

    def test_guard_object_has_no_mmio_stores_or_pmu_mcr(self):
        with tempfile.TemporaryDirectory() as tmp:
            obj = Path(tmp) / "gt_handoff_guard.o"
            command = (
                "arm-vita-eabi-gcc -I/src/include -mcpu=cortex-a9 "
                "-mthumb-interwork -O0 -g3 -ffreestanding -c "
                "src/gt_handoff_guard.c -o /tmp/gt_handoff_guard.o && "
                "arm-vita-eabi-objdump -dr /tmp/gt_handoff_guard.o"
            )
            disasm = docker_toolchain(command)
        self.assertNotRegex(disasm, r"\b(?:str|strb|strh|stm(?:ia|db)?)\b\s+[^\n]*\[(?!sp|fp|r7)")
        self.assertNotRegex(disasm, r"\bmcr\s+p15\b.*(?:c9|c14|c15)")

    def test_guard_object_materializes_exact_register_addresses(self):
        with tempfile.TemporaryDirectory() as tmp:
            command = (
                "arm-vita-eabi-gcc -I/src/include -mcpu=cortex-a9 "
                "-mthumb-interwork -O0 -g3 -ffreestanding -c "
                "src/gt_handoff_guard.c -o /tmp/gt_handoff_guard.o && "
                "arm-vita-eabi-objdump -dr /tmp/gt_handoff_guard.o"
            )
            disasm = docker_toolchain(command)
        for immediate in ("0x208", "0x1a00", "0x3000", "0xe310", "0x3004"):
            self.assertIn(f"@ {immediate}", disasm)

    def test_stack_reservation_is_present_in_assembled_start(self):
        disasm = docker_toolchain(
            "arm-vita-eabi-gcc -mcpu=cortex-a9 -mthumb-interwork "
            "-c src/start.s -o /tmp/start.o && "
            "arm-vita-eabi-objdump -d /tmp/start.o"
        )
        self.assertRegex(disasm, r"mov\s+sp,\s*#?0x8000|mov\s+sp,\s*#32768")
        reserve = re.search(r"sub\s+sp,\s*sp,\s*#(?:0x100|256)", disasm)
        partition = re.search(r"sub\s+sp,\s*sp,\s*r1,\s*lsl\s*#13|sub\s+sp,\s*r1,\s*r1,\s*lsl\s*#13", disasm)
        self.assertIsNotNone(reserve)
        self.assertIsNotNone(partition)
        self.assertLess(reserve.start(), partition.start())


if __name__ == "__main__":
    unittest.main()
