"""
efuse_vdd_sdio.py -- PlatformIO post: script, runs before every upload.

GPIO12 (MTDI) carries A8 on the ESP32-BOBOARD-5V.  The ESP32 ROM samples it
at every chip reset (power-on, EN, brown-out, watchdog) to select the flash
voltage: HIGH means VDD_SDIO = 1.8 V and the WROOM-32 flash (3.3 V) does not
boot.  On the Atari bus A8 is random at reset, and the TXS0108E pull-ups hold
it HIGH when the bus is idle.

The fix is in the efuses: XPD_SDIO_FORCE = 1 (ignore GPIO12), XPD_SDIO_REG = 1
(regulator on), XPD_SDIO_TIEH = 1 (3.3 V), i.e. `espefuse.py set_flash_voltage
3.3V`.  Before every upload this script reads the efuses and:
  - all three set          -> nothing to do;
  - all three blank        -> burns them (irreversible) and checks the result;
  - any other combination  -> stops the upload without burning anything.

Enabled by `custom_efuse_vdd_sdio = 3.3V` in platformio.ini (empty = off).
Only ESP32 (classic) boards are accepted.

espefuse needs Python packages that the PlatformIO environment does not ship
(ecdsa, bitstring, reedsolo, cryptography): the script creates a private
virtualenv in .pio/espefuse-venv on first use (needs network access once).

Rule: the module is programmed on its own, connected only to the PC.
"""

Import("env")  # noqa: F821  (PlatformIO SCons global)

import json
import os
import subprocess
import sys
from os.path import isfile, join

EFUSE_DEPS = [
    "bitstring>=3.1.6,<4.2",
    "cryptography>=2.1.4",
    "ecdsa>=0.16.0",
    "pyserial>=3.0",
    "reedsolo>=1.5.3,<=1.6.0",
]
SDIO_FUSES = ("XPD_SDIO_FORCE", "XPD_SDIO_REG", "XPD_SDIO_TIEH")


def _fail(env, msg):
    sys.stderr.write("[efuse] ERROR: %s\n" % msg)
    env.Exit(1)


def _venv_python(env):
    venv = join(env.subst("$PROJECT_DIR"), ".pio", "espefuse-venv")
    py = join(venv, "Scripts", "python.exe") if os.name == "nt" else join(venv, "bin", "python")
    stamp = join(venv, ".deps-ok")
    if isfile(py) and isfile(stamp):
        return py
    print("[efuse] Creating %s for espefuse (first use)..." % venv)
    host = env.subst("$PYTHONEXE")
    if subprocess.call([host, "-m", "venv", venv]) != 0:
        _fail(env, "cannot create the virtualenv %s" % venv)
    if subprocess.call([py, "-m", "pip", "install", "-q"] + EFUSE_DEPS) != 0:
        _fail(env, "cannot install the espefuse dependencies into %s" % venv)
    open(stamp, "w").close()
    return py


def _read_sdio(env, py, espefuse, port):
    out = join(env.subst("$BUILD_DIR"), "efuse-summary.json")
    if isfile(out):
        os.remove(out)
    cmd = [py, espefuse, "--chip", "esp32", "--port", port,
           "summary", "--format", "json", "--file", out]
    if subprocess.call(cmd) != 0 or not isfile(out):
        _fail(env, "cannot read the efuses on %s" % port)
    with open(out) as f:
        fuses = json.load(f)
    return {name: _fuse_bit(fuses[name]["value"]) for name in SDIO_FUSES}


def _fuse_bit(value):
    # espefuse 4.x reports XPD_SDIO_FORCE / XPD_SDIO_REG as booleans and
    # XPD_SDIO_TIEH as "1.8V" (0) or "3.3V" (1).
    if isinstance(value, str):
        if value == "3.3V":
            return True
        if value == "1.8V":
            return False
        raise ValueError("unexpected efuse value %r" % value)
    return bool(value)


def check_vdd_sdio(source, target, env):
    want = env.GetProjectOption("custom_efuse_vdd_sdio", "").strip()
    if not want:
        return
    if want != "3.3V":
        _fail(env, "custom_efuse_vdd_sdio = %s: only 3.3V is supported" % want)
    if env.BoardConfig().get("build.mcu", "") != "esp32":
        _fail(env, "VDD_SDIO efuse check is for the ESP32 (classic) only")

    env.AutodetectUploadPort()
    port = env.subst("$UPLOAD_PORT")
    if not port:
        _fail(env, "no upload port")

    py = _venv_python(env)
    espefuse = join(env.PioPlatform().get_package_dir("tool-esptoolpy"), "espefuse.py")

    sdio = _read_sdio(env, py, espefuse, port)
    state = ", ".join("%s=%d" % (k, v) for k, v in sdio.items())
    if all(sdio.values()):
        print("[efuse] VDD_SDIO fixed at 3.3 V, GPIO12 ignored (%s)" % state)
        return
    if any(sdio.values()):
        _fail(env, "VDD_SDIO efuses in an unexpected state (%s): nothing burned, "
                   "check with `espefuse.py summary`" % state)

    print("[efuse] VDD_SDIO efuses blank (%s)" % state)
    print("[efuse] Burning VDD_SDIO = 3.3 V (XPD_SDIO_FORCE/REG/TIEH): IRREVERSIBLE")
    cmd = [py, espefuse, "--chip", "esp32", "--port", port, "--do-not-confirm",
           "set_flash_voltage", "3.3V"]
    if subprocess.call(cmd) != 0:
        _fail(env, "espefuse set_flash_voltage 3.3V failed")

    sdio = _read_sdio(env, py, espefuse, port)
    if not all(sdio.values()):
        _fail(env, "VDD_SDIO efuses not set after burning (%s)"
                   % ", ".join("%s=%d" % (k, v) for k, v in sdio.items()))
    print("[efuse] VDD_SDIO fixed at 3.3 V, GPIO12 ignored from now on")


env.AddPreAction("upload", check_vdd_sdio)  # noqa: F821
