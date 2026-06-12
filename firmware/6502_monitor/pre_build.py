Import("env")
import subprocess
import os
import sys

makefile_dir = os.path.join(env["PROJECT_DIR"], "6502")

# Runs at script-load time, before any source file is compiled.
print("[pre-build] Assembling PBI handler (6502/)...")
result = subprocess.run(
    ["make", "-C", makefile_dir],
    capture_output=True, text=True
)
if result.stdout:
    print(result.stdout, end="")
if result.returncode != 0:
    print(result.stderr, file=sys.stderr)
    sys.exit(1)
