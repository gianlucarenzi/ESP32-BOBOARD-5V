Import("env")
import subprocess
import os
import sys

makefile_dir = os.path.join(env["PROJECT_DIR"], "6502")

def build_pbi_handler(source, target, env):
    print("[pre-build] Assembling PBI handler (6502/)...")
    result = subprocess.run(
        ["make", "-C", makefile_dir],
        capture_output=True, text=True
    )
    if result.stdout:
        print(result.stdout, end="")
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        env.Exit(1)

env.AddPreAction("buildprog", build_pbi_handler)
