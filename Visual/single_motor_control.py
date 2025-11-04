#!/usr/bin/env python3
"""
Single-motor control script using the low-level Dynamixel client.

This script demonstrates how to:
 - connect to the Dynamixel USB device
 - enable torque
 - step a single motor from a start to end position in N steps
 - read back present/goal positions
 - safely disable torque on exit

Usage example:
    python Visual/single_motor_control.py --motor 7 --device /dev/ttyACM0 --baud 2000000 \
        --start 2000 --end 2500 --steps 50 --delay 0.05

If you only want to test without touching hardware, add --dry-run.

Notes:
 - Position values are in motor ticks (control-table units). Check your motor's range.
 - You may need appropriate permissions for the serial device (add your user to the dialout group or run with sudo).
"""

import argparse
import time
import numpy as np

from ruka_hand.utils.dynamixel_util import DynamixelClient
from ruka_hand.utils.control_table.control_table import ADDR_PRESENT_POSITION


def move_single_motor(motor_id, start, end, steps, delay, dry_run):
    
    positions = np.linspace(start, end, steps, dtype=int)

    if dry_run:
        print("DRY RUN: will not send commands. Sample positions:", positions[:10], "... total", len(positions))

    # Create client for the single motor
    client = DynamixelClient([motor_id], port=device)
    try:
        if not dry_run:
            print(f"Connecting to {device} at {baud}...")
            client.connect()
            # Some users may prefer to explicitly set baud via PortHandler; DynamixelClient uses BAUDRATE constant.
            # Enable torque
            print("Enabling torque...")
            client.set_torque_enabled(True)
            time.sleep(0.1)

        print("Starting stepping loop. Press Ctrl-C to stop.")
        step_idx = 0
        for pos in positions:
            t0 = time.time()
            if dry_run:
                print(f"[{step_idx}] would set motor {motor_id} -> {int(pos)}")
            else:
                # set single motor target
                client.set_pos_indv(motor_id, int(pos))
                # read back present/goal to confirm
                # read_pos returns list same order as motor_ids passed to DynamixelClient
                present = client.read_pos()
                goal = client.read_goal_pos()
                present_val = present[0] if present and len(present) > 0 else None
                goal_val = goal[0] if goal and len(goal) > 0 else None
                print(f"[{step_idx}] set -> {int(pos)}, goal(stored)={goal_val}, present={present_val}")

            step_idx += 1
            elapsed = time.time() - t0
            to_sleep = delay - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print("Error:", e)
    finally:
        if not dry_run:
            try:
                print("Disabling torque and disconnecting...")
                client.set_torque_enabled(False)
            except Exception:
                pass
            try:
                client.disconnect()
            except Exception:
                pass

    print("Done")

def main():
    parser = argparse.ArgumentParser(description="Single motor control script")
    parser.add_argument('--motor', type=int, required=True, help="Motor ID to control")
    parser.add_argument('--device', type=str, required=True, help="Serial device path (e.g., /dev/ttyACM0)")
    parser.add_argument('--start', type=int, required=True, help="Start position (in ticks)")
    parser.add_argument('--end', type=int, required=True, help="End position (in ticks)")
    parser.add_argument('--steps', type=int, default=100, help="Number of steps to move from start to end")
    parser.add_argument('--delay', type=float, default=0.1, help="Delay between steps (in seconds)")
    parser.add_argument('--dry-run', action='store_true', help="If set, do not send commands to hardware")

    args = parser.parse_args()

    move_single_motor(
        motor_id=args.motor,
        start=args.start,
        end=args.end,
        steps=args.steps,
        delay=args.delay,
        dry_run=args.dry_run
    )

if __name__ == '__main__':
    main()
