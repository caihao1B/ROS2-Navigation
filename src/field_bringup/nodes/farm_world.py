"""Own the Gazebo server and shut it down over its transport service."""

import os
from pathlib import Path
import hashlib
import json
import signal
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import subprocess
import sys
import threading


def gazebo_environment():
    env = os.environ.copy()
    lib = Path(get_package_prefix("field_bringup")) / "lib"
    compat = lib / "libfield_serialized_stop.so"
    pins = json.loads(
        (
            Path(get_package_share_directory("field_bringup"))
            / "config/shutdown_abi.json"
        ).read_text()
    )
    # Resolve the same shared objects that the current loader will use.
    dependencies = subprocess.check_output(
        ["ldd", str(lib / "libfield_drive.so"), str(compat)], text=True
    )
    loaded = {}
    for line in dependencies.splitlines():
        fields = line.split()
        if len(fields) >= 3 and fields[1] == "=>" and fields[0] in pins:
            resolved = Path(fields[2])
            if resolved.is_file():
                loaded[fields[0]] = hashlib.sha256(resolved.read_bytes()).hexdigest()
    if loaded == pins and not env.get("LD_PRELOAD"):
        env["LD_PRELOAD"] = str(compat)
        print(
            "Gazebo shutdown serialization enabled for the matching simulator ABI.",
            flush=True,
        )
    return env


def main():
    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.set())
    server = subprocess.Popen(
        sys.argv[1:], start_new_session=True, env=gazebo_environment()
    )
    try:
        while server.poll() is None and not stopping.wait(0.2):
            pass
        if server.poll() is not None:
            return server.returncode
        # The Ruby CLI has a separate server process. Stopping its world lets
        # Gazebo join the render and physics threads before the CLI exits.
        try:
            subprocess.run(
                [
                    "gz",
                    "service",
                    "-s",
                    "/server_control",
                    "--reqtype",
                    "gz.msgs.ServerControl",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "3000",
                    "--req",
                    "stop: true",
                ],
                timeout=5,
                check=False,
            )
            return server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid, signal.SIGTERM)
            return server.wait(timeout=4)
    finally:
        if server.poll() is None:
            os.killpg(server.pid, signal.SIGKILL)
            server.wait()


if __name__ == "__main__":
    raise SystemExit(main())
