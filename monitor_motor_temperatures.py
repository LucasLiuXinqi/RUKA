#!/usr/bin/env python3
"""Cycle fingers while logging Dynamixel motor temperatures."""

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

DEFAULT_FINGER_ORDER = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
# Min/max degrees for each motor (matches move.py)
MIN_DEG = np.array(
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -60, -30], dtype=float
)
MAX_DEG = np.array(
    [90, 15, 120, 23, 110, 85, 120, 45, 90, 80, 110, 110, 170, 90, 60, 30],
    dtype=float,
)


class MotorTemperatureMonitor:
    """Runs a long-duration motion routine while logging motor temperatures."""

    def __init__(
        self,
        hand_type: str,
        duration_hours: float,
        log_path: Path,
        finger_order: Sequence[str],
        flex_ratio: float,
        flex_hold: float,
        open_hold: float,
        traj_len: int,
        traj_sleep: float,
        log_interval: float,
        chunk_minutes: float,
        idle_ref_hold: float,
    ):
        if duration_hours <= 0:
            raise ValueError("duration_hours must be positive.")
        if flex_ratio <= 0.0:
            raise ValueError("flex_ratio must be positive.")
        if traj_len < 1:
            raise ValueError("traj_len must be >= 1.")

        self.hand = Hand(hand_type)
        self.min_deg = MIN_DEG.copy()
        self.max_deg = MAX_DEG.copy()
        self.duration_seconds = duration_hours * 3600.0
        self.log_path = log_path
        self.flex_ratio = self._normalize_ratio(flex_ratio)
        self.flex_hold = max(0.0, flex_hold)
        self.open_hold = max(0.0, open_hold)
        self.traj_len = traj_len
        self.traj_sleep = max(0.0, traj_sleep)
        self.log_interval = max(0.0, log_interval)
        self.finger_order = list(finger_order)
        self.idle_ref_hold = max(0.0, idle_ref_hold)

        self.motor_index = {
            motor_id: idx for idx, motor_id in enumerate(self.hand.motors)
        }
        self.num_motors = len(self.hand.motors)
        self.open_deg_pose = np.zeros(len(self.hand.motors), dtype=float)
        self.open_pose = self._deg_to_motor_positions(self.open_deg_pose)

        base_fields = [
            "timestamp",
            "elapsed_sec",
            "cycle_index",
            "finger",
            "stage",
            "chunk_index",
        ]
        self.motor_fields = [
            f"m{motor_id:02d}_temp_c" for motor_id in self.hand.motors
        ]
        self.fieldnames = base_fields + self.motor_fields
        self._next_periodic_log = None
        self.chunk_seconds = chunk_minutes * 60.0
        if self.chunk_seconds <= 0:
            raise ValueError("chunk_minutes must be positive.")
        self._chunk_idx = 0
        self._chunk_start_time = None
        self._chunk_end_time = None
        self._log_file = None
        self._writer = None
        self._base_log_path = Path(log_path)

    def _chunk_log_path(self, chunk_idx: int) -> Path:
        base = self._base_log_path
        suffix = base.suffix if base.suffix else ".csv"
        stem = base.stem if base.suffix else base.name
        filename = f"{stem}_chunk{chunk_idx:02d}{suffix}"
        return base.with_name(filename)

    def _open_chunk_writer(self):
        if self._log_file:
            self._log_file.close()
        chunk_path = self._chunk_log_path(self._chunk_idx)
        chunk_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = chunk_path.open("w", newline="")
        self._writer = csv.DictWriter(self._log_file, fieldnames=self.fieldnames)
        self._writer.writeheader()
        print(f"[monitor] Writing chunk {self._chunk_idx} to {chunk_path}")

    def _close_chunk_writer(self):
        if self._log_file:
            self._log_file.close()
            self._log_file = None
        self._writer = None

    def _maybe_rotate_chunk(self, current_time: float):
        if current_time < self._chunk_end_time:
            return
        while current_time >= self._chunk_end_time:
            self._chunk_idx += 1
            self._chunk_start_time = self._chunk_end_time
            self._chunk_end_time = self._chunk_start_time + self.chunk_seconds
            self._open_chunk_writer()

    def run(self):
        """Execute the motion/temperature logging routine."""
        start_time = time.time()
        stop_time = start_time + self.duration_seconds
        self._next_periodic_log = (
            start_time + self.log_interval if self.log_interval > 0 else None
        )

        self._chunk_start_time = start_time
        self._chunk_end_time = start_time + self.chunk_seconds
        self._open_chunk_writer()

        print(
            f"[monitor] Logging temperatures for {self.duration_seconds/3600:.2f} h "
            f"to files like {self._chunk_log_path(self._chunk_idx)}"
        )
        cycle_idx = 0
        try:
            self._move_to_pose(self.open_pose)
            self._log_temperatures(
                finger="Init",
                stage="open",
                start_time=start_time,
                cycle_idx=cycle_idx,
                force=True,
            )
            while time.time() < stop_time:
                for finger in self.finger_order:
                    if time.time() >= stop_time:
                        break
                    cycle_idx = self._perform_finger_cycle(
                        finger=finger,
                        start_time=start_time,
                        cycle_idx=cycle_idx,
                        stop_time=stop_time,
                    )
        except KeyboardInterrupt:
            print("\n[monitor] Interrupted by user, shutting down...")
        finally:
            self.hand.close()
            self._close_chunk_writer()

        print("[monitor] Temperature logging finished.")

    def _perform_finger_cycle(
        self,
        finger: str,
        start_time: float,
        cycle_idx: int,
        stop_time: float,
    ):
        if self.idle_ref_hold > 0:
            self._move_to_pose(self.open_pose)
            self._log_temperatures(
                finger=finger,
                stage="idle_ref",
                start_time=start_time,
                cycle_idx=cycle_idx,
                force=True,
            )
            self._sleep_with_logging(
                duration=self.idle_ref_hold,
                finger=finger,
                stage="idle_ref_hold",
                start_time=start_time,
                cycle_idx=cycle_idx,
                stop_time=stop_time,
            )

        finger_indices = self._finger_indices(finger)
        for motor_index in finger_indices:
            if time.time() >= stop_time:
                break
            cycle_idx += 1
            motor_id = self.hand.motors[motor_index]
            motor_stage = f"motor_{motor_id}"
            flex_pose = self._pose_for_indices([motor_index], self.flex_ratio)
            self._move_to_pose(flex_pose)
            self._log_temperatures(
                finger=finger,
                stage=f"{motor_stage}_flexed",
                start_time=start_time,
                cycle_idx=cycle_idx,
                force=True,
            )
            self._sleep_with_logging(
                duration=self.flex_hold,
                finger=finger,
                stage=f"{motor_stage}_flex_hold",
                start_time=start_time,
                cycle_idx=cycle_idx,
                stop_time=stop_time,
            )
            if time.time() >= stop_time:
                break
            self._move_to_pose(self.open_pose)
            self._log_temperatures(
                finger=finger,
                stage=f"{motor_stage}_open",
                start_time=start_time,
                cycle_idx=cycle_idx,
                force=True,
            )
            self._sleep_with_logging(
                duration=self.open_hold,
                finger=finger,
                stage=f"{motor_stage}_open_hold",
                start_time=start_time,
                cycle_idx=cycle_idx,
                stop_time=stop_time,
            )
        return cycle_idx

    def _pose_for_indices(self, motor_indices: List[int], ratio: float) -> np.ndarray:
        degs = self.open_deg_pose.copy()
        for idx in motor_indices:
            degs[idx] = self._target_deg_for_index(idx, ratio)
        return self._deg_to_motor_positions(degs)

    def _finger_indices(self, finger: str) -> List[int]:
        raw_ids = FINGER_NAMES_TO_MOTOR_IDS[finger]
        indices: List[int] = []
        max_index = len(self.hand.motors) - 1
        for raw in raw_ids:
            if 0 <= raw <= max_index:
                indices.append(raw)
            elif raw in self.motor_index:
                indices.append(self.motor_index[raw])
            else:
                raise KeyError(f"Unable to resolve motor reference '{raw}' for {finger}")
        return indices

    def _normalize_ratio(self, ratio: float) -> float:
        if ratio > 1.0:
            return min(ratio, 100.0) / 100.0
        return ratio

    def _target_deg_for_index(self, idx: int, ratio: float) -> float:
        return self.min_deg[idx] + ratio * (self.max_deg[idx] - self.min_deg[idx])

    def _deg_to_motor_positions(self, deg_vec: np.ndarray) -> np.ndarray:
        input_pos = np.array(deg_vec, dtype=float)
        clamped = np.clip(input_pos, self.min_deg, self.max_deg)
        denom = self.max_deg - self.min_deg
        denom[denom == 0] = 1.0
        normed = (clamped - self.min_deg) / denom
        normed = normed.copy()
        # MCP coupling compensation (matches move.py tweaks)
        normed[9] = normed[9] - (40 / 110) * normed[10]
        normed[0] = normed[0] - (40 / 110) * normed[2]
        normed[5] = normed[5] - (40 / 110) * normed[4]
        normed[8] = normed[8] - (40 / 110) * normed[6]
        positions = (
            normed * (self.hand.curled_bound - self.hand.tensioned_pos)
            + self.hand.tensioned_pos
        )
        return positions

    def _move_to_pose(self, target: np.ndarray):
        curr_pos = np.array(self.hand.actual_pos, dtype=float)
        move_to_pos(
            curr_pos=curr_pos,
            des_pos=target,
            hand=self.hand,
            traj_len=self.traj_len,
            sleep_time=self.traj_sleep,
        )

    def _sleep_with_logging(
        self,
        duration: float,
        finger: str,
        stage: str,
        start_time: float,
        cycle_idx: int,
        stop_time: float,
    ):
        if duration <= 0:
            return
        end_time = min(time.time() + duration, stop_time)
        if self.log_interval <= 0:
            remaining = max(0.0, end_time - time.time())
            if remaining > 0:
                time.sleep(remaining)
            return
        poll = min(1.0, max(0.2, self.log_interval / 4.0))
        while time.time() < end_time:
            remaining = end_time - time.time()
            time.sleep(min(poll, max(0.0, remaining)))
            self._log_temperatures(
                finger=finger,
                stage=stage,
                start_time=start_time,
                cycle_idx=cycle_idx,
                force=False,
            )

    def _log_temperatures(
        self,
        finger: str,
        stage: str,
        start_time: float,
        cycle_idx: int,
        force: bool,
    ):
        now = time.time()
        if not force:
            if self.log_interval <= 0:
                return
            if self._next_periodic_log is None or now < self._next_periodic_log:
                return

        temps = self.hand.read_temp()
        if temps is None or len(temps) != self.num_motors:
            return
        if any(t is None for t in temps):
            return

        temps = [float(t) for t in temps]
        timestamp = datetime.utcnow().isoformat()
        elapsed = now - start_time
        row = {
            "timestamp": timestamp,
            "elapsed_sec": round(elapsed, 3),
            "cycle_index": cycle_idx,
            "finger": finger,
            "stage": stage,
            "chunk_index": self._chunk_idx,
        }
        for motor_id, temp in zip(self.hand.motors, temps):
            row[f"m{motor_id:02d}_temp_c"] = temp

        self._writer.writerow(row)
        self._log_file.flush()
        self._maybe_rotate_chunk(now)

        if self.log_interval > 0:
            if self._next_periodic_log is None:
                self._next_periodic_log = now + self.log_interval
            else:
                while self._next_periodic_log <= now:
                    self._next_periodic_log += self.log_interval


def parse_finger_order(raw_value: str) -> List[str]:
    tokens = [token.strip() for token in raw_value.split(",") if token.strip()]
    if not tokens:
        return DEFAULT_FINGER_ORDER
    normalized = []
    for token in tokens:
        name = token.strip().title()
        if name not in FINGER_NAMES_TO_MOTOR_IDS:
            raise ValueError(f"Unknown finger name '{token}'.")
        normalized.append(name)
    return normalized


def default_log_path() -> Path:
    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"motor_temps_{timestamp}.csv"


def main():
    parser = argparse.ArgumentParser(
        description="Cycle fingers sequentially while recording motor temperatures."
    )
    parser.add_argument(
        "--hand-type",
        choices=["left", "right"],
        default="right",
        help="Which hand to command.",
    )
    parser.add_argument(
        "--duration-hours",
        type=float,
        default=5.0,
        help="How long to run the routine.",
    )
    parser.add_argument(
        "--log-path",
        type=Path,
        default=None,
        help="Destination CSV file for recorded temperatures.",
    )
    parser.add_argument(
        "--finger-order",
        type=str,
        default=",".join(DEFAULT_FINGER_ORDER),
        help="Comma-separated list describing the order to actuate fingers.",
    )
    parser.add_argument(
        "--flex-ratio",
        type=float,
        default=90.0,
        help=(
            "Fraction of the tensioned->curled range used when flexing a finger. "
            "Values >1 will be treated as percentages (e.g. 90 -> 0.90)."
        ),
    )
    parser.add_argument(
        "--flex-hold",
        type=float,
        default=1.5,
        help="Seconds to hold each finger in the flexed pose.",
    )
    parser.add_argument(
        "--open-hold",
        type=float,
        default=0.5,
        help="Seconds to pause in the open pose before switching to the next finger.",
    )
    parser.add_argument(
        "--traj-len",
        type=int,
        default=40,
        help="Number of interpolation steps for each move (smaller is faster).",
    )
    parser.add_argument(
        "--traj-sleep",
        type=float,
        default=0.005,
        help="Delay between trajectory steps in seconds.",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=60.0,
        help=(
            "Seconds between periodic samples while holding poses. Set to 0 to only "
            "log when a finger changes stage."
        ),
    )
    parser.add_argument(
        "--chunk-minutes",
        type=float,
        default=30.0,
        help="Minutes per CSV chunk before starting a new log file.",
    )
    parser.add_argument(
        "--idle-ref-hold",
        type=float,
        default=0.0,
        help="Seconds to hold in open (idle) pose and log temps before each finger cycle.",
    )
    args = parser.parse_args()

    log_path = args.log_path or default_log_path()
    finger_order = parse_finger_order(args.finger_order)
    monitor = MotorTemperatureMonitor(
        hand_type=args.hand_type,
        duration_hours=args.duration_hours,
        log_path=log_path,
        finger_order=finger_order,
        flex_ratio=args.flex_ratio,
        flex_hold=args.flex_hold,
        open_hold=args.open_hold,
        traj_len=args.traj_len,
        traj_sleep=args.traj_sleep,
        log_interval=args.log_interval,
        chunk_minutes=args.chunk_minutes,
        idle_ref_hold=args.idle_ref_hold,
    )
    monitor.run()


if __name__ == "__main__":
    main()
