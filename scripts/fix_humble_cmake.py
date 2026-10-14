#!/usr/bin/env python3


import re
import subprocess
from pathlib import Path


def main():
    prefix = Path("/opt/ros/humble")
    export = prefix / "share/rcutils/cmake/ament_cmake_export_link_flags-extras.cmake"
    if not export.exists():
        print("rcutils has no legacy link flag export; no compatibility change needed")
        return
    contents = export.read_text()
    flag = 'set(_exported_link_flags "-latomic")'
    if flag not in contents:
        print("rcutils does not export the legacy atomic flag; no compatibility change needed")
        return
    library = prefix / "lib/librcutils.so"
    symbols = subprocess.check_output(["readelf", "-W", "--dyn-syms", str(library)], text=True)
    dynamic = subprocess.check_output(["readelf", "-d", str(library)], text=True)
    unresolved_atomic = any(
        " UND " in line and re.search(r"\b__atomic_", line) for line in symbols.splitlines()
    )
    if unresolved_atomic and "Shared library: [libatomic.so." not in dynamic:
        raise RuntimeError("rcutils requires atomic symbols without a direct libatomic dependency")
    replacement = (
        "# Humble compatibility: https://github.com/ros2/rcutils/pull/528\n"
        'set(_exported_link_flags "")'
    )
    export.write_text(contents.replace(flag, replacement, 1))
    print("Applied the rcutils atomic export compatibility fix; library unchanged")


if __name__ == "__main__":
    main()
