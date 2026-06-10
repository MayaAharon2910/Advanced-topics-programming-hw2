#!/usr/bin/env python3
"""
Prepared edge-case runner for HW1.

The edge-case input files are already stored in the runtime format expected by
`drone_mapper`. This script does not convert maps, does not create a temporary
input directory, and does not change the input format.

For each selected case, it runs the executable directly on the case directory,
exactly like a manual command such as:

    ./build/drone_mapper edge_cases/case1

The only extra action is that the script creates or overwrites
`simulation_config.txt` in each selected case directory before running it.
The generated configuration enables full DEBUG simulator logging, so every run
through this script produces the most detailed simulator trace.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from pathlib import Path


TIMEOUT_SECONDS = 120
ITER_STEP = 250

OUTPUT_FILES = [
    "map_output.txt",
    "score.txt",
    "simulation_log.txt",
    "input_errors.txt",
]

SEP = "=" * 80
SEP2 = "-" * 80

_ITER_RE = re.compile(r"^Iteration\s+(\d+)", re.IGNORECASE)

FULL_DEBUG_SIMULATION_CONFIG = """# Auto-created and overwritten by run_all_edge_cases.py.
# Enables the most detailed simulator logging for this prepared EDGE_CASE.

log_enabled=true
log_level=DEBUG
debug_mode=true
"""


def write_simulation_config(case_dir: Path) -> Path:
    config_path = case_dir / "simulation_config.txt"

    config_path.write_text(
        FULL_DEBUG_SIMULATION_CONFIG,
        encoding="utf-8",
        newline="\n",
    )

    print(f"Overwrote simulation config: {config_path}")

    return config_path


def remove_old_outputs(case_dir: Path) -> None:
    for name in OUTPUT_FILES:
        output_path = case_dir / name
        if output_path.exists():
            output_path.unlink()


def should_print(line: str) -> bool:
    match = _ITER_RE.match(line.strip())

    if match:
        return int(match.group(1)) % ITER_STEP == 0

    return True


def extract_score(lines: list[str], case_dir: Path) -> str:
    for line in lines:
        s = line.strip()

        if s.startswith("Score:") or s.startswith("Final Score:"):
            return s

    score_file = case_dir / "score.txt"

    if score_file.exists():
        for line in score_file.read_text(encoding="utf-8").splitlines():
            s = line.strip()

            if s.startswith("Score:") or s.startswith("Final Score:"):
                return s

    return "score not found"


def run_case(exe: Path, case_dir: Path) -> tuple[bool, int, str]:
    write_simulation_config(case_dir)
    remove_old_outputs(case_dir)

    try:
        proc = subprocess.Popen(
            [str(exe), str(case_dir)],
            cwd=exe.parent,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
    except OSError as exc:
        print(f"ERROR launching executable: {exc}")
        return False, 1, "launch error"

    all_lines: list[str] = []

    assert proc.stdout is not None

    try:
        for raw in proc.stdout:
            line = raw.rstrip("\n")
            all_lines.append(line)

            if should_print(line):
                print(line)

        proc.wait(timeout=TIMEOUT_SECONDS)

    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        return False, 124, f"TIMEOUT after {TIMEOUT_SECONDS}s"

    score = extract_score(all_lines, case_dir)
    ok = proc.returncode == 0

    return ok, proc.returncode, score


def main() -> int:
    start_time = time.perf_counter()

    parser = argparse.ArgumentParser(description="Run HW1 prepared EDGE_CASEs")

    parser.add_argument(
        "--exe",
        type=Path,
        default=None,
        help="Path to drone_mapper. Default: ../build/drone_mapper",
    )

    parser.add_argument(
        "--case",
        action="append",
        metavar="CASE",
        help="Run only this case, for example: --case case1. Can be repeated.",
    )

    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent

    if args.exe:
        exe = args.exe.resolve()
    else:
        exe = (script_dir.parent / "build" / "drone_mapper").resolve()

    if not exe.exists() or not os.access(exe, os.X_OK):
        print(f"ERROR: executable not found or not runnable: {exe}")
        print("Build first:")
        print("  cmake --build build --target drone_mapper")
        return 1

    all_cases = sorted(
        p for p in script_dir.iterdir()
        if p.is_dir() and p.name.startswith("case")
    )

    if args.case:
        selected = set(args.case)
        cases = [p for p in all_cases if p.name in selected]

        if not cases:
            print(f"ERROR: no cases matched: {selected}")
            return 1
    else:
        cases = all_cases

    if not cases:
        print(f"ERROR: no case directories found under {script_dir}")
        return 1

    results_log: list[tuple[str, bool, int, str]] = []

    for case_dir in cases:
        print(SEP)
        print(f"TEST: {case_dir.name}")
        print(SEP)
        print("Direct run command:")
        print(f"{exe} {case_dir}")
        print(SEP2)
        print("Program output:")
        print()

        ok, code, score = run_case(exe, case_dir)

        print(SEP2)
        print(f"Return code: {code}")
        print(score)
        print()

        results_log.append((case_dir.name, ok, code, score))

    print(SEP)
    print("SUMMARY")
    print(SEP)

    failures = 0
    total = len(results_log)

    # ANSI colors for terminal output, similar to the visual feel of test runners.
    GREEN = "\033[32m"
    RED = "\033[31m"
    YELLOW = "\033[33m"
    RESET = "\033[0m"

    # Detailed summary for every EDGE_CASE.
    for index, (name, ok, code, score) in enumerate(results_log, start=1):
        if ok:
            status = f"{GREEN}PASS{RESET}"
        else:
            status = f"{RED}FAIL{RESET}"
            failures += 1

        print(f"{index:>2}. {status} {name:<12} code={code:<3} score={score}")

    passed = total - failures
    pass_percent = 0 if total == 0 else int(round((passed / total) * 100))
    elapsed = time.perf_counter() - start_time

    print()

    # CTest-like final summary line.
    if failures == 0:
        print(f"{GREEN}{pass_percent}% tests passed, {failures} tests failed out of {total}{RESET}")
    else:
        print(f"{RED}{pass_percent}% tests passed, {failures} tests failed out of {total}{RESET}")

    print()
    print(f"Total Test time (real) = {elapsed:6.2f} sec")

    if failures:
        print()
        print(f"{RED}The following tests FAILED:{RESET}")
        for index, (name, ok, code, score) in enumerate(results_log, start=1):
            if not ok:
                print(f"\t{index} - {name} (Failed) code={code} {score}")
    else:
        print()
        print(f"{GREEN}ALL EDGE CASES PASSED!{RESET}")

    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
