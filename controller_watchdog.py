#!/usr/bin/env python3
"""
Controller watchdog — 自动重试激活 inactive 的 robot_joint_controller
在三狗同 gzserver 环境下，controller 会因 DDS 超时反复掉线，此脚本持续监控并自动恢复。
"""
import subprocess
import time
import sys
import os

DOGS = ["go2_1", "go2_2", "go2_3"]
CHECK_INTERVAL = 10
KEYS_DIR = "/tmp/opencode"


def clean_ansi(text):
    import re
    return re.sub(r'\x1b\[[0-9;]*m', '', text)


def is_controller_active(dog):
    try:
        r = subprocess.run(
            ["ros2", "control", "list_controllers", "-c", f"{dog}/controller_manager"],
            capture_output=True, text=True, timeout=8,
            env={**os.environ, "ROS_LOCALHOST_ONLY": "1"})
        out = clean_ansi(r.stdout)
        return f"robot_joint_controller_{dog}" in out and "active" in out
    except Exception:
        return False


def reactivate(dog):
    idx = dog.replace("go2_", "")
    keys_fifo = f"{KEYS_DIR}/keys_{idx}"
    if not os.path.exists(keys_fifo):
        print(f"  {dog}: FIFO not found, skip")
        return

    print(f"  {dog}: sending R -> 0 -> 1 to reactivate controller...")
    for key, wait in [("R", 4), ("0", 6), ("1", 6)]:
        with open(keys_fifo, "w") as f:
            f.write(key)
        time.sleep(wait)

    # 等待 controller 激活
    for _ in range(15):
        if is_controller_active(dog):
            print(f"  {dog}: controller reactivated!")
            # 确保 navigation mode ON
            with open(keys_fifo, "w") as f:
                f.write("N")
            time.sleep(2)
            with open(keys_fifo, "w") as f:
                f.write(" ")
            return True
        time.sleep(2)

    print(f"  {dog}: FAILED to reactivate")
    return False


def main():
    print("Controller watchdog started. Monitoring:", DOGS)
    print(f"Check interval: {CHECK_INTERVAL}s")
    print("Press Ctrl+C to stop.\n")

    while True:
        for dog in DOGS:
            if not is_controller_active(dog):
                print(f"[{time.strftime('%H:%M:%S')}] {dog} controller inactive!")
                reactivate(dog)
        time.sleep(CHECK_INTERVAL)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nWatchdog stopped.")
