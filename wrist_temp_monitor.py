#!/usr/bin/env python3
"""Move wrist motors (15,16) while logging motor temperatures."""

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path
from typing import List

from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

# Wrist joint indices in the 16-length pose vector
WRIST_INDICES = [14, 15]

def move_to_wrist_pos(
    hand: Hand,
    base_pos: List[float],
    wrist_pos: List[float],
    traj_len: int,
    traj_sleep: float,
):
    target_pos = list(base_pos)
    for idx, pos in zip(WRIST_INDICES, wrist_pos):
        target_pos[idx] = pos
    curr_pos = list(hand.actual_pos)
    move_to_pos(
        curr_pos=curr_pos,
        des_pos=target_pos,
        hand=hand,
        traj_len=traj_len,
        sleep_time=traj_sleep,
    )


def read_temps_row(hand: Hand) -> dict:
    temps = hand.read_temp()
    if temps is None or any(t is None for t in temps):
        return {}
    row = {}
    for motor_id, temp in zip(hand.motors, temps):
        row[f"m{motor_id:02d}_temp_c"] = float(temp)
    return row


def main():
    parser = argparse.ArgumentParser(
        description="Move wrist motors and log temperatures."
    )
    parser.add_argument(
        "--hand-type",
        choices=["left", "right"],
        default="right",
        help="Which hand to command.",
    )
    parser.add_argument(
        "--duration-minutes",
        type=float,
        default=30.0,
        help="Total run duration in minutes.",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=10.0,
        help="Seconds between temperature samples.",
    )
    parser.add_argument(
        "--traj-len",
        type=int,
        default=60,
        help="Interpolation steps for each move.",
    )
    parser.add_argument(
        "--traj-sleep",
        type=float,
        default=0.008,
        help="Delay between trajectory steps.",
    )
    parser.add_argument(
        "--wrist1-min",
        type=int,
        default=1800,
        help="Min position for wrist motor 15 (index 14).",
    )
    parser.add_argument(
        "--wrist1-max",
        type=int,
        default=2300,
        help="Max position for wrist motor 15 (index 14).",
    )
    parser.add_argument(
        "--wrist2-min",
        type=int,
        default=2500,
        help="Min position for wrist motor 16 (index 15).",
    )
    parser.add_argument(
        "--wrist2-max",
        type=int,
        default=1800,
        help="Max position for wrist motor 16 (index 15).",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=2.0,
        help="Seconds to hold at each wrist position.",
    )
    parser.add_argument(
        "--log-path",
        type=Path,
        default=None,
        help="CSV to log temps. Default: logs/wrist_temps_<timestamp>.csv",
    )
    parser.add_argument(
        "--chunk-minutes",
        type=float,
        default=30.0,
        help="Minutes per CSV chunk before starting a new log file.",
    )
    parser.add_argument(
        "--return-neutral",
        action="store_true",
        help="Move wrists to neutral position before exiting.",
    )
    parser.add_argument(
        "--neutral-wrist1",
        type=int,
        default=None,
        help="Neutral position for wrist motor 15 (index 14). Defaults to midpoint.",
    )
    parser.add_argument(
        "--neutral-wrist2",
        type=int,
        default=None,
        help="Neutral position for wrist motor 16 (index 15). Defaults to midpoint.",
    )
    args = parser.parse_args()

    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    log_path = args.log_path or Path("logs") / f"wrist_temps_{timestamp}.csv"
    log_path.parent.mkdir(parents=True, exist_ok=True)

    hand = Hand(args.hand_type)
    base_pos = list(hand.actual_pos)
    start_time = time.time()
    stop_time = start_time + args.duration_minutes * 60.0
    next_log = start_time

    fieldnames = ["timestamp", "elapsed_sec", "stage", "chunk_index"] + [
        f"m{motor_id:02d}_temp_c" for motor_id in hand.motors
    ]

    chunk_seconds = args.chunk_minutes * 60.0
    if chunk_seconds <= 0:
        raise ValueError("chunk_minutes must be positive.")
    chunk_idx = 0
    chunk_start = start_time
    chunk_end = chunk_start + chunk_seconds

    def chunk_path(idx: int) -> Path:
        suffix = log_path.suffix if log_path.suffix else ".csv"
        stem = log_path.stem if log_path.suffix else log_path.name
        return log_path.with_name(f"{stem}_chunk{idx:02d}{suffix}")

    log_file = None
    writer = None

    def open_chunk(idx: int):
        nonlocal log_file, writer
        if log_file:
            log_file.close()
        path = chunk_path(idx)
        path.parent.mkdir(parents=True, exist_ok=True)
        log_file = path.open("w", newline="")
        writer = csv.DictWriter(log_file, fieldnames=fieldnames)
        writer.writeheader()
        print(f"[wrist] Writing chunk {idx} to {path}")

    def maybe_rotate(now: float):
        nonlocal chunk_idx, chunk_start, chunk_end
        if now < chunk_end:
            return
        while now >= chunk_end:
            chunk_idx += 1
            chunk_start = chunk_end
            chunk_end = chunk_start + chunk_seconds
            open_chunk(chunk_idx)

    open_chunk(chunk_idx)

    try:
        while time.time() < stop_time:
            for stage, wrist_deg in [
                ("wrist_min_min", [args.wrist1_min, args.wrist2_min]),
                ("wrist_min_max", [args.wrist1_min, args.wrist2_max]),
                ("wrist_max_min", [args.wrist1_max, args.wrist2_min]),
                ("wrist_max_max", [args.wrist1_max, args.wrist2_max]),
            ]:
                if time.time() >= stop_time:
                    break
                move_to_wrist_pos(
                    hand=hand,
                    base_pos=base_pos,
                    wrist_pos=wrist_deg,
                    traj_len=args.traj_len,
                    traj_sleep=args.traj_sleep,
                )
                if args.hold > 0:
                    time.sleep(args.hold)
                now = time.time()
                if now >= next_log:
                    row = {
                        "timestamp": datetime.utcnow().isoformat(),
                        "elapsed_sec": round(now - start_time, 3),
                        "stage": stage,
                        "chunk_index": chunk_idx,
                    }
                    row.update(read_temps_row(hand))
                    if row and writer:
                        writer.writerow(row)
                        log_file.flush()
                    next_log = now + args.log_interval
                maybe_rotate(now)
    finally:
        if args.return_neutral:
            neutral1 = (
                args.neutral_wrist1
                if args.neutral_wrist1 is not None
                else int(round((args.wrist1_min + args.wrist1_max) / 2))
            )
            neutral2 = (
                args.neutral_wrist2
                if args.neutral_wrist2 is not None
                else int(round((args.wrist2_min + args.wrist2_max) / 2))
            )
            move_to_wrist_pos(
                hand=hand,
                base_pos=base_pos,
                wrist_pos=[neutral1, neutral2],
                traj_len=args.traj_len,
                traj_sleep=args.traj_sleep,
            )
        if log_file:
            log_file.close()
        hand.close()

    print(f"[done] Logged wrist temps to {log_path}")


if __name__ == "__main__":
    main()
