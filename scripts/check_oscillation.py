#!/usr/bin/env python3
"""读 /cmd_vel rosbag，统计拐弯窗口 angular.z 过零次数。

用法：
  ros2 bag record -o /tmp/teb_test.bag /cmd_vel
  # （跑完 L 形路径后）
  python3 scripts/check_oscillation.py /tmp/teb_test.bag

输出：
  Found N turning windows:
    Window 1 (start=T, duration=Ns, peak=±X rad/s):
      Zero crossings: K
      Verdict: PASS/FAIL (threshold: <= 2)
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

CROSSING_THRESHOLD = 0.3  # |wz| > 0.3 视为有效拐弯信号
TURN_START_HOLD_SEC = 0.5  # 持续 0.5s 才算拐弯开始
ZERO_BAND = 0.05  # |wz| < 0.05 视为零
MAX_ACCEPTABLE_CROSSINGS = 2  # 验收阈值


def load_cmd_vel_from_bag(bag_path: str):
    """读 rosbag 中 /cmd_vel 的 (t, wz) 序列。"""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores

    times = []
    wzs = []
    with AnyReader([Path(bag_path)]) as reader:
        reader.set_typestore(Stores.ROS2_HUMBLE)
        connections = [c for c in reader.connections if c.topic == "/cmd_vel"]
        for conn, ts, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            t = ts * 1e-9
            wz = float(msg.angular.z)
            times.append(t)
            wzs.append(wz)
    return np.array(times), np.array(wzs)


def find_turning_windows(times: np.ndarray, wzs: np.ndarray):
    """找到拐弯窗口：|wz| > CROSSING_THRESHOLD 持续 ≥ TURN_START_HOLD_SEC 的区间。"""
    if len(times) < 2:
        return []
    above = np.abs(wzs) > CROSSING_THRESHOLD
    windows = []
    i = 0
    while i < len(times):
        if not above[i]:
            i += 1
            continue
        j = i
        while j < len(times) and above[j]:
            j += 1
        # 持续时间
        if j > 0 and times[j - 1] - times[i] >= TURN_START_HOLD_SEC:
            windows.append((i, j - 1))
        i = j
    return windows


def count_zero_crossings(wzs_slice: np.ndarray) -> int:
    """统计符号翻转次数（|wz| > ZERO_BAND 才算有效翻转）。"""
    crossings = 0
    last_sign = 0
    for w in wzs_slice:
        if abs(w) < ZERO_BAND:
            continue
        sign = 1 if w > 0 else -1
        if last_sign != 0 and sign != last_sign:
            crossings += 1
        last_sign = sign
    return crossings


def analyze(bag_path: str) -> int:
    times, wzs = load_cmd_vel_from_bag(bag_path)
    if len(times) == 0:
        print(f"ERROR: No /cmd_vel messages in {bag_path}")
        return 1

    windows = find_turning_windows(times, wzs)
    print(f"Found {len(windows)} turning window(s):")
    overall_pass = True
    for i, (s, e) in enumerate(windows, 1):
        start_t = times[s]
        duration = times[e] - times[s]
        peak = float(np.max(np.abs(wzs[s:e + 1])))
        crossings = count_zero_crossings(wzs[s:e + 1])
        verdict = "PASS" if crossings <= MAX_ACCEPTABLE_CROSSINGS else "FAIL"
        if verdict == "FAIL":
            overall_pass = False
        print(f"  Window {i} (start={start_t:.1f}s, duration={duration:.1f}s, peak=±{peak:.2f} rad/s):")
        print(f"    Zero crossings: {crossings}")
        print(f"    Verdict: {verdict} (threshold: <= {MAX_ACCEPTABLE_CROSSINGS})")

    print()
    print(f"Overall: {'PASS' if overall_pass else 'FAIL'}")
    return 0 if overall_pass else 2


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 check_oscillation.py <bag_path>")
        sys.exit(1)
    sys.exit(analyze(sys.argv[1]))
