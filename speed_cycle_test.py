#!/usr/bin/env python3
"""Measure fast open/close cycle speeds for each finger."""

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path
from typing import List, Sequence

import numpy as np

from ruka_hand.control.hand import Hand
from ruka_hand.utils.constants import FINGER_NAMES_TO_MOTOR_IDS
from ruka_hand.utils.trajectory import move_to_pos


def finger_indices(hand: Hand, finger: str) -> List[int]:
    """Return position indices for the motors in a finger."""
    indices = []
    max_index = len(hand.motors) - 1
    for raw in FINGER_NAMES_TO_MOTOR_IDS[finger]:
        if 0 <= raw <= max_index:
            indices.append(raw)
        else:
            # If raw is a motor ID, translate to index
            try:
                idx = hand.motors.index(raw)
            except ValueError as exc:
                raise KeyError(f"Unknown motor reference {raw} for finger {finger}") from exc
            indices.append(idx)
    return indices


def finger_pose(hand: Hand, indices: List[int], ratio: float) -> np.ndarray:
    """Build a pose where only the given indices move to a ratio of curl range."""
    pose = np.array(hand.tensioned_pos, dtype=float)
    for idx in indices:
        pose[idx] = hand.tensioned_pos[idx] + ratio * (hand.curled_bound[idx] - hand.tensioned_pos[idx])
    return pose


def cycle_finger(
    hand: Hand,
    finger: str,
    cycles: int,
    flex_ratio: float,
    traj_len: int,
    traj_sleep: float,
    flex_hold: float,
    open_hold: float,
    writer: csv.DictWriter,
):
    """Run open/close cycles for a finger and log durations."""
    idxs = finger_indices(hand, finger)
    open_pose = np.array(hand.tensioned_pos, dtype=float)
    flex_pose = finger_pose(hand, idxs, flex_ratio)

    for cycle in range(1, cycles + 1):
        t0 = time.perf_counter()
        move_to_pos(open_pose, flex_pose, hand=hand, traj_len=traj_len, sleep_time=traj_sleep)
        if flex_hold > 0:
            time.sleep(flex_hold)
        move_to_pos(flex_pose, open_pose, hand=hand, traj_len=traj_len, sleep_time=traj_sleep)
        if open_hold > 0:
            time.sleep(open_hold)
        duration = time.perf_counter() - t0
        writer.writerow(
            {
                "finger": finger,
                "cycle": cycle,
                "duration_sec": duration,
            }
        )
        print(f"[{finger}] cycle {cycle}: {duration:.4f}s")


def main():
    parser = argparse.ArgumentParser(description="Measure fast open/close cycles per finger.")
    parser.add_argument(
        "--hand-type",
        choices=["left", "right"],
        default="right",
        help="Which hand to test.",
    )
    parser.add_argument(
        "--finger-order",
        type=str,
        default="Thumb,Index,Middle,Ring,Pinky",
        help="Comma-separated order of fingers to test.",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=30,
        help="Number of open/close cycles per finger.",
    )
    parser.add_argument(
        "--flex-ratio",
        type=float,
        default=1.0,
        help="Fraction of curl range to move (1.0 hits curled_bound).",
    )
    parser.add_argument(
        "--traj-len",
        type=int,
        default=10,
        help="Interpolation steps (smaller is faster).",
    )
    parser.add_argument(
        "--traj-sleep",
        type=float,
        default=0.002,
        help="Delay between trajectory steps (seconds).",
    )
    parser.add_argument(
        "--flex-hold",
        type=float,
        default=0.0,
        help="Hold time at flexed pose (seconds).",
    )
    parser.add_argument(
        "--open-hold",
        type=float,
        default=0.0,
        help="Hold time at open pose (seconds).",
    )
    parser.add_argument(
        "--log-path",
        type=Path,
        default=None,
        help="CSV to log cycle durations. Default: logs/speed_test_<timestamp>.csv",
    )
    args = parser.parse_args()

    fingers = [f.strip().title() for f in args.finger_order.split(",") if f.strip()]
    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    log_path = args.log_path or Path("logs") / f"speed_test_{timestamp}.csv"
    log_path.parent.mkdir(parents=True, exist_ok=True)

    hand = Hand(args.hand_type)
    fieldnames = ["finger", "cycle", "duration_sec"]
    with log_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        try:
            for finger in fingers:
                print(f"=== Testing {finger} ===")
                cycle_finger(
                    hand=hand,
                    finger=finger,
                    cycles=args.cycles,
                    flex_ratio=args.flex_ratio,
                    traj_len=args.traj_len,
                    traj_sleep=args.traj_sleep,
                    flex_hold=args.flex_hold,
                    open_hold=args.open_hold,
                    writer=writer,
                )
        finally:
            hand.close()

    print(f"[done] Logged cycle timings to {log_path}")


if __name__ == "__main__":
    main()
