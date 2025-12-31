import argparse
import time
import numpy as np

from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

parser = argparse.ArgumentParser(description="Teleop robot hands.")
parser.add_argument(
    "-ht",
    "--hand_type",
    type=str,
    help="Hand you'd like to teleoperate",
    default="right",
)
args = parser.parse_args()
hand = Hand(args.hand_type)

# # Min/max degrees for each motor
# min_deg = np.array([0 , -40, 0 , -15, 0 , 0 , 0 , -5 , 0 , 0 , 0 , 0 , 0  , 0 , -60, -55], dtype=float)
# max_deg = np.array([90,  5 , 85,  5 , 90, 85, 70,  20, 90, 80, 90, 90, 145, 90,  60,  0 ], dtype=float)
#           motor   1   2   3    4   5    6     7    8      9   10  11   12   13   14   15   16
min_deg = np.array([0 , 0 , 0  , 0 , 0  , 0,    0  , 0 ,    0, 0 , 0  , 0  , 0  , 0 , -60, -30], dtype=float)
max_deg = np.array([90, 15, 120, 23, 110, 85 ,  120, 45,    90 , 80, 110, 110, 170, 90,  60,  30 ], dtype=float)

def normalize_to_motor(input_pos):
    input_pos = np.array(input_pos, dtype=float)
    clamped = np.clip(input_pos, min_deg, max_deg)
    normed = clamped / (max_deg - min_deg)
    # compensation of MCP movement
    normed[9] = normed[9] - (40/110) * normed[10]   # angle when dip motor keep its tensioned position while mcp motor bend to the maxmium position
    normed[0] = normed[0] - (40/110) * normed[2]
    # normed[5] = normed[5] - (45/110) * normed[4]
    normed[5] = normed[5] - (40/110) * normed[4]
    normed[8] = normed[8] - (40/110) * normed[6]
    print(normed)
    positions = normed * (hand.curled_bound - hand.tensioned_pos) + hand.tensioned_pos
    # positions[9] = normed[9] * (hand.curled_bound[9] - 800) + 800 #tensioned pos when MCP bend
    # positions[0] = normed[0] * (hand.curled_bound[0] - 3000) + 3000
    # positions[5] = normed[5] * (hand.curled_bound[5] - 1100) + 1100
    # positions[8] = normed[8] * (hand.curled_bound[8] - 2750) + 2750
    print(positions)
    return positions

while True:
    curr_pos = hand.read_pos()
    time.sleep(0.5)

    raw = input("Enter target joint positions: ")

    try:
        test_pos = list(map(float, raw.strip().split()))
        des_pos = normalize_to_motor(test_pos)

        print(f"curr_pos: {curr_pos}")
        print(f"input (deg): {test_pos}")
        print(f"normalized des_pos: {des_pos}")
        input()

        move_to_pos(curr_pos=curr_pos, des_pos=des_pos, hand=hand, traj_len=50)
    except Exception as e:
        print(f"Error: {e}")
        continue

hand.close()

"""
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0

-- index mcp test
0 0 80 0 0 0 0 0 0 0 0 0 0 0 0 0

-- middle mcp test
0 0 0 0 0 0 0 0 0 0 80 0 0 0 0 0

-- ring mcp test
0 0 0 0 80 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 50 0 0 0 0 0 0 0 0 0 0

-- pinky mcp test
0 0 0 0 0 0 90 0 0 0 0 0 0 0 0 0

"""
