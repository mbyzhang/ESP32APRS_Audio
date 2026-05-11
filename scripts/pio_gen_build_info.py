# PlatformIO pre-build hook: regenerate include/build_info.h with the
# current git commit/branch/build timestamp so the firmware can advertise
# its version via /api/version.  Run via:
#     extra_scripts = pre:scripts/pio_gen_build_info.py
import os
import subprocess
import sys

# `Import` is provided by PlatformIO's SConstruct environment.
Import("env")  # noqa: F821

ROOT = env["PROJECT_DIR"]  # noqa: F821
script = os.path.join(ROOT, "scripts", "gen_build_info.py")
subprocess.run([sys.executable, script], check=False)
