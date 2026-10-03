#!/usr/bin/env python3
"""Smoke test harness for IVP graphical examples.

Discovers the current sample executables from ivp_examples/CMakeLists.txt,
launches each one with SDL_VIDEODRIVER=dummy, and enforces that every
declared sample exists in the build directory.

Expected outcomes:
  PASS  - process exits cleanly within timeout, or times out (expected for
          interactive demos that run forever).
  FAIL  - process crashes (segfault, abort, non-zero exit before timeout),
          excessive stderr output, anomaly messages in debug output, or a
          declared executable is missing from the build directory.
  SKIP  - SDL init failure detected in stderr.

Usage:
    python smoke_test_examples.py [--build-dir DIR] [--timeout SECS]
                                  [--cmake-file FILE] [--verbose] [--list]
"""
import argparse
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import List, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_CMAKE_FILE = REPO_ROOT / "ivp_examples" / "CMakeLists.txt"
TARGET_PATTERN = re.compile(r"^\s*add_ivp_sample\(\s*([A-Za-z0-9_]+)\b")

# Stderr patterns that indicate expected SDL init failure (SKIP)
SDL_INIT_FAILURES = [
    "sdl_init",
    "renderer init failed",
    "could not initialize",
    "no available video device",
    "no video mode",
    "SDL_CreateWindow",
]

# Stderr patterns that indicate anomalies (FAIL)
ANOMALY_PATTERNS = [
    "instability flag",
    "NaN detected",
    "anomaly",
    "assertion failed",
    "abort",
    "segmentation fault",
]

# Maximum allowed stderr lines before flagging as excessive
MAX_STDERR_LINES = 50


def discover_examples(cmake_file: Path) -> List[str]:
    """Discover declared sample targets from the examples CMake file."""
    try:
        lines = cmake_file.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise RuntimeError(f"failed to read {cmake_file}: {exc}") from exc

    examples: List[str] = []
    for line in lines:
        match = TARGET_PATTERN.match(line)
        if match:
            examples.append(match.group(1))

    if not examples:
        raise RuntimeError(
            f"no add_ivp_sample(...) targets found in {cmake_file}"
        )

    return examples


def find_executable(build_dir: Path, name: str) -> Path | None:
    """Find example executable in build directory."""
    candidates = [
        build_dir / name,
        build_dir / f"{name}.exe",
        build_dir / "examples" / name,
        build_dir / "examples" / f"{name}.exe",
        build_dir / "Release" / name,
        build_dir / "Release" / f"{name}.exe",
        build_dir / "Debug" / name,
        build_dir / "Debug" / f"{name}.exe",
        build_dir / "examples" / "Release" / f"{name}.exe",
        build_dir / "examples" / "Debug" / f"{name}.exe",
    ]
    for c in candidates:
        if c.is_file():
            return c
    return None


def _output_text(value: str | bytes | None) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value or ""


def run_example(
    exe: Path, timeout: float, verbose: bool
) -> Tuple[str, str]:
    """Run an example and validate output even when its event loop times out."""
    env = os.environ.copy()
    env["SDL_VIDEODRIVER"] = "dummy"
    env["IVP_DEBUG"] = "1"
    timed_out = False
    returncode = None

    try:
        proc = subprocess.run(
            [str(exe.resolve())],
            timeout=timeout,
            capture_output=True,
            text=True,
            env=env,
        )
        stderr_text = proc.stderr or ""
        stdout_text = proc.stdout or ""
        returncode = proc.returncode
    except subprocess.TimeoutExpired as e:
        timed_out = True
        stderr_text = _output_text(e.stderr)
        stdout_text = _output_text(e.stdout)
        if verbose and stderr_text.strip():
            print(f"    stderr (timeout): {stderr_text[:200]}")
    except OSError as e:
        return "FAIL", f"OS error: {e}"

    combined_lower = (stderr_text + stdout_text).lower()
    for pattern in ANOMALY_PATTERNS:
        if pattern.lower() in combined_lower:
            return "FAIL", f"anomaly detected: {pattern}"

    stderr_lines = stderr_text.strip().splitlines()
    if len(stderr_lines) > MAX_STDERR_LINES:
        return "FAIL", f"excessive stderr ({len(stderr_lines)} lines)"

    # A crash must not be hidden by an earlier SDL initialization diagnostic.
    if returncode is not None and (returncode < 0 or returncode >= 128):
        return "FAIL", f"abnormal exit (rc={returncode})"

    # Only completed startup failures qualify as unavailable video drivers.
    if not timed_out and returncode != 0:
        stderr_lower = stderr_text.lower()
        for pattern in SDL_INIT_FAILURES:
            if pattern.lower() in stderr_lower:
                return "SKIP", f"SDL init failure: {pattern}"
        detail = f"exit code {returncode}"
        if stderr_text.strip():
            detail += f": {stderr_text.strip()[:200]}"
        return "FAIL", detail

    if timed_out:
        return "PASS", "timeout (expected)"
    return "PASS", f"clean exit (rc={returncode})"


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Smoke test IVP graphical examples declared in ivp_examples/CMakeLists.txt"
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=Path("."),
        help="Build directory containing example executables",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Timeout in seconds per example (default: 5)",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print detailed output",
    )
    parser.add_argument(
        "--cmake-file",
        type=Path,
        default=DEFAULT_CMAKE_FILE,
        help="Path to ivp_examples/CMakeLists.txt used to discover expected samples",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="Print the discovered sample target names and exit",
    )
    args = parser.parse_args(argv)

    try:
        examples = discover_examples(args.cmake_file)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if args.list:
        for name in examples:
            print(name)
        return 0

    results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
    failures: List[str] = []

    for name in examples:
        exe = find_executable(args.build_dir, name)
        if exe is None:
            status, detail = "FAIL", "declared by CMake but not found in build directory"
        else:
            status, detail = run_example(exe, args.timeout, args.verbose)

        results[status] += 1
        marker = {"PASS": ".", "FAIL": "F", "SKIP": "S"}[status]
        print(f"  [{marker}] {name}: {status} -- {detail}")

        if status == "FAIL":
            failures.append(f"{name}: {detail}")

    print()
    total = sum(results.values())
    print(
        f"smoke_test_examples: {total} declared examples, "
        f"{results['PASS']} passed, "
        f"{results['FAIL']} failed, "
        f"{results['SKIP']} skipped"
    )

    if failures:
        print("\nFailures:", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1

    if results["PASS"] == 0 and results["SKIP"] == total:
        print("All discovered examples were skipped due to SDL initialization limits")
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
