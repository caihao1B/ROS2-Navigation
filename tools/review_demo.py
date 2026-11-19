#!/usr/bin/env python3
"""Run a small field-navigation example from any checkout location."""

import argparse
from pathlib import Path
import subprocess
import sys


CASES = {
    'follow-stop-resume': ('follow_stop_resume', '巡检 · 前车跟随、停车等待与恢复'),
    'spot-spray': ('spot_spray', '点喷 · 规划点序、停稳服务与出液反馈'),
    'strip-mowing': ('strip_mowing', '割草 · 阿克曼换向、停车与面积计量'),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_mutually_exclusive_group(required=True)
    commands.add_argument('--list', action='store_true', help='列出案例及用途')
    commands.add_argument('--case', choices=CASES, help='运行案例并核对参考输出')
    options = parser.parse_args()
    if options.list:
        for name, (_, purpose) in CASES.items():
            print(f'{name:20} {purpose}')
        return 0
    repo = Path(__file__).resolve().parents[1]
    directory, _ = CASES[options.case]
    script = repo / 'examples' / directory / 'run.py'
    return subprocess.run([sys.executable, str(script)], check=False).returncode


if __name__ == '__main__':
    sys.exit(main())
