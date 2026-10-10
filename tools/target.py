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
    """Invoke idf.py with the working interpreter, without shell indirection."""
    idf_path = find_idf_path()
    if idf_path is None:
        raise RuntimeError("Set IDF_PATH or idf.currentSetup in .vscode/settings.json")
    idf_script = idf_path / "tools" / "idf.py"
    if not idf_script.is_file():
        raise RuntimeError(f"ESP-IDF entry point not found: {idf_script}")

    tools_path = Path(os.environ.get("IDF_TOOLS_PATH") or
                      str(idf_path.parents[1] / "tools"))
    env_path = Path(os.environ.get("IDF_PYTHON_ENV_PATH") or
                    str(tools_path / "python_env" / "idf5.5_py3.11_env"))
    python_exe = env_path / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    if not python_exe.is_file():
        raise RuntimeError(f"ESP-IDF Python not found: {python_exe}")

    env = os.environ.copy()
    env["IDF_PATH"] = str(idf_path)
    env["IDF_TOOLS_PATH"] = str(tools_path)
    env["IDF_PYTHON_ENV_PATH"] = str(env_path)
    cmd = [str(python_exe), str(idf_script), *args]
    print("Executable:", python_exe, flush=True)
    try:
        subprocess.run(cmd, check=True, cwd=ROOT, env=env)
    except OSError as exc:
        raise RuntimeError(f"Could not launch {python_exe}: {exc}") from exc


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
