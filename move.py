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
#           motor   1   2   3    4   5     6   7    8    9   10  11   12   13   14   15   16
min_deg = np.array([0 , 0 , 0  , 0 , 0  , -85, 0  , 0 , -90, 0 , 0  , 0  , 0  , 0 , -60, -30], dtype=float)
max_deg = np.array([90, 15, 120, 23, 110,  0 , 120, 45,  0 , 80, 110, 110, 170, 90,  60,  30 ], dtype=float)

def normalize_to_motor(test_pos):
    test_pos = np.array(test_pos, dtype=float)
    clamped = np.clip(test_pos, min_deg, max_deg)
    normed = clamped / (max_deg - min_deg)
    normed[9] = normed[9] - (35/110) * normed[10]
    normed[0] = normed[0] - (40/110) * normed[2]
    normed[5] = normed[5] + (48/110) * normed[4]
    normed[8] = normed[8] - (35/110) * normed[6]
    print(normed)
    positions = normed * (hand.curled_bound - hand.tensioned_pos) + hand.tensioned_pos
    positions[9] = normed[9] * (hand.curled_bound[9] - 1633) + 1633
    positions[0] = normed[0] * (hand.curled_bound[0] - 2520) + 2520
    positions[5] = normed[5] * (hand.curled_bound[5] - 1422) + 1422
    positions[8] = normed[8] * (hand.curled_bound[8] - 2552) + 2552
    # positions[1] = 2285 + normed[1] * abs(hand.curled_bound[1] - hand.tensioned_pos[1])
    # positions[3] = 2070 - normed[3] * abs(hand.curled_bound[3] - hand.tensioned_pos[3])
    # positions[7] = 2125 + normed[7] * abs(hand.curled_bound[7] - hand.tensioned_pos[7])
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

0 0 0 0 80 0 0 0 0 0 0 0 0 0 0 0

0 0 0 0 0 0 0 0 0 0 30 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 60 0 0 0 0 0
0 0 0 0 0 0 0 0 0 50 60 0 0 0 0 0

0 0 0 0 0 0 0 0 0 30 80 0 0 0 0 0

0 0 0 0 0 0 0 0 0 0 80 0 0 0 0 0
0 0 0 0 0 0 0 0 0 0 110 0 0 0 0 0


"""
