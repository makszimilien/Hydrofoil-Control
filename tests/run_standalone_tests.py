"""Compile and run the shipped header tests; no hardware or network access."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="hydrofoil-header-tests-") as temporary:
    for name in ("sensor_wait_test", "bench_recording_test"):
        executable = Path(temporary) / name
        subprocess.run(["clang++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                        "-O1", "-g", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                        "-I", str(ROOT / "include"), str(ROOT / "tests" / (name + ".cpp")),
                        "-o", str(executable)], check=True, timeout=60)
        subprocess.run([str(executable)], check=True, timeout=30)
