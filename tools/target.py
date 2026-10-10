#!/usr/bin/env python3
"""Build/flash/monitor an isolated ESP-IDF firmware target."""
import argparse
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TARGETS = ("transmitter", "rx_skid_steer", "rx_conventional")

def main():
    p = argparse.ArgumentParser()
    p.add_argument("target", choices=(*TARGETS, "all"))
    p.add_argument("action", choices=("build", "flash", "monitor", "flash-monitor"))
    p.add_argument("--port", help="Serial port; alternatively set RC_PORT_<TARGET>")
    args = p.parse_args()
    if args.target == "all" and args.action != "build":
        p.error("'all' supports build only")
    for target in TARGETS if args.target == "all" else (args.target,):
        project = ROOT / "firmware" / target
        build = ROOT / "build" / target
        port = args.port or os.getenv("RC_PORT_" + target.upper())
        if args.action != "build" and not port:
            p.error("Specify --port or RC_PORT_" + target.upper())
        command = ["idf.py", "-C", str(project), "-B", str(build)]
        if port:
            command += ["-p", port]
        command += {"build":["build"],"flash":["flash"],"monitor":["monitor"],"flash-monitor":["flash","monitor"]}[args.action]
        print("Running:", " ".join(command), flush=True)
        subprocess.run(command, check=True, cwd=ROOT)

if __name__ == "__main__":
    main()
