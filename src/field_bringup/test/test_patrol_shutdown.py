"""Exercise a real ROS node's terminal cleanup without loading a world or sending goals."""

import atexit
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "nodes"))


def await_marker(path, process=None, timeout=12):
    deadline = time.monotonic() + timeout
    while not path.exists():
        if process is not None and process.poll() is not None:
            raise AssertionError(
                f"Worker exited before {path.name}: {process.returncode}"
            )
        if time.monotonic() >= deadline:
            raise TimeoutError(path)
        time.sleep(0.01)


def worker(directory, scenario, terminal):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from patrol_task import finish_patrol

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("patrol_terminal_probe")
    # Keep an actual DDS entity alive until destruction. There is no actuator publisher.
    node.create_subscription(String, "~/observation", lambda _: None, 1)
    trace = (directory / "observations.jsonl").open("w")
    trace.write('{"sample":"idle"}\n')
    signal.signal(signal.SIGINT, lambda *_: (directory / "stop_requested").touch())

    def boundary():
        (directory / "at_boundary").touch()
        await_marker(directory / "continue")

    if scenario == "during-cleanup":
        node.context.on_shutdown(boundary)
    if scenario == "after-report":
        atexit.register(boundary)
    code = finish_patrol(node, trace, directory, {"terminal": terminal})
    assert trace.closed and not rclpy.ok()
    return code


def launch_worker(directory, terminal):
    from launch import LaunchService, LaunchDescription
    from launch.actions import ExecuteProcess, RegisterEventHandler
    from launch.event_handlers import OnProcessExit

    path = PACKAGE / "launch/inspection.launch.py"
    spec = importlib.util.spec_from_file_location("inspection_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    task = ExecuteProcess(
        cmd=[sys.executable, __file__, "worker", str(directory), "natural", terminal]
    )
    service = LaunchService(argv=[])
    service.include_launch_description(
        LaunchDescription(
            [
                RegisterEventHandler(
                    OnProcessExit(target_action=task, on_exit=module.component_exited)
                ),
                task,
            ]
        )
    )
    return service.run()


@pytest.mark.parametrize("phase", ["during-cleanup", "after-report"])
def test_ctrl_c_at_terminal_boundary(tmp_path, phase):
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / "ros_logs"))
    child = subprocess.Popen(
        [sys.executable, __file__, "worker", str(tmp_path), phase, "COMPLETED"], env=env
    )
    report = tmp_path / "mission_result.json"
    try:
        await_marker(tmp_path / "at_boundary", child)
        assert report.exists() == (phase == "after-report")
        child.send_signal(signal.SIGINT)
        if phase == "during-cleanup":
            await_marker(tmp_path / "stop_requested", child)
        (tmp_path / "continue").touch()
        assert child.wait(timeout=10) == 0
        assert json.loads(report.read_text())["terminal"] == "COMPLETED"
        assert not (tmp_path / "mission_result.json.tmp").exists()
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()


@pytest.mark.parametrize("terminal,successful", [("COMPLETED", True), ("FAULT", False)])
def test_natural_exit_drives_launch(tmp_path, terminal, successful):
    process = subprocess.run(
        [sys.executable, __file__, "launch", str(tmp_path), "natural", terminal],
        env=dict(os.environ, ROS_LOG_DIR=str(tmp_path / "ros_logs")),
        text=True,
        capture_output=True,
        timeout=16,
    )
    assert (process.returncode == 0) == successful, process.stdout + process.stderr
    assert (
        json.loads((tmp_path / "mission_result.json").read_text())["terminal"]
        == terminal
    )
    if not successful:
        assert "exited with status 2" in process.stdout + process.stderr


if __name__ == "__main__":
    mode, folder, phase, terminal = sys.argv[1:]
    if mode == "worker":
        raise SystemExit(worker(Path(folder), phase, terminal))
    raise SystemExit(launch_worker(Path(folder), terminal))
