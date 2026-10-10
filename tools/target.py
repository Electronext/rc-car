#!/usr/bin/env python3
"""Build, flash or monitor independent ESP-IDF firmware targets."""
import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TARGETS = ("transmitter", "rx_skid_steer", "rx_conventional")
ACTIONS = {"build": ["build"], "flash": ["flash"],
           "monitor": ["monitor"], "flash-monitor": ["flash", "monitor"]}


def find_idf_path():
    """Use active IDF_PATH or the ESP-IDF extension's configured installation."""
    if os.environ.get("IDF_PATH"):
        return Path(os.environ["IDF_PATH"])
    settings = ROOT / ".vscode" / "settings.json"
    if settings.exists():
        try:
            setup = json.loads(settings.read_text(encoding="utf-8")).get("idf.currentSetup")
            if setup:
                return Path(setup)
        except (ValueError, OSError):
            pass
    return None


def run_idf(args):
    # An ESP-IDF terminal already exports its Python, tools and IDF_PATH.
    if os.environ.get("IDF_PATH") and shutil.which("idf.py"):
        subprocess.run(["idf.py", *args], check=True, cwd=ROOT)
        return

    idf_path = find_idf_path()
    if os.name != "nt":
        raise RuntimeError("Activate ESP-IDF in the terminal (idf.py on PATH) before running this task.")
    if idf_path is None:
        raise RuntimeError("ESP-IDF not found. Set IDF_PATH or idf.currentSetup in .vscode/settings.json.")
    export = idf_path / "export.bat"
    idf_script = idf_path / "tools" / "idf.py"
    if not export.is_file() or not idf_script.is_file():
        raise RuntimeError(f"Invalid ESP-IDF installation at {idf_path}: export.bat or tools/idf.py missing")

    # Export in the same cmd.exe process that invokes idf.py so that PATH,
    # Python environment and toolchain variables remain available.
    command = ('call ' + subprocess.list2cmdline([str(export)])
               + ' && python ' + subprocess.list2cmdline([str(idf_script), *args]))
    subprocess.run(["cmd.exe", "/d", "/c", command], check=True, cwd=ROOT)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("target", choices=(*TARGETS, "all"))
    p.add_argument("action", choices=tuple(ACTIONS))
    p.add_argument("--port", help="Serial port, or set RC_PORT_<TARGET>")
    args = p.parse_args()
    if args.target == "all" and args.action != "build":
        p.error("'all' supports build only")
    for target in TARGETS if args.target == "all" else (args.target,):
        project = ROOT / "firmware" / target
        build = ROOT / "build" / target
        port = args.port or os.getenv("RC_PORT_" + target.upper())
        if args.action != "build" and not port:
            p.error("Specify --port or RC_PORT_" + target.upper())
        command = ["-C", str(project), "-B", str(build)]
        if port:
            command += ["-p", port]
        command += ACTIONS[args.action]
        print(f"ESP-IDF {target}: {' '.join(command)}", flush=True)
        try:
            run_idf(command)
        except (OSError, RuntimeError) as exc:
            p.exit(1, f"Build launcher error: {exc}\n")


if __name__ == "__main__":
    main()
