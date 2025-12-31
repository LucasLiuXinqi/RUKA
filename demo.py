import time
import numpy as np
from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

###########################################
# USE YOUR EXACT NORMALIZATION FUNCTION
###########################################

min_deg = np.array([0 , 0 , 0  , 0 , 0  , 0,    0  , 0 ,    0, 0 , 0  , 0  , 0  , 0 , -60, -30], dtype=float)
max_deg = np.array([90, 15, 120, 23, 110, 85 ,  120, 45,    90 , 80, 110, 110, 170, 90,  60,  30 ], dtype=float)

def normalize_to_motor(input_pos, hand):
    input_pos = np.array(input_pos, dtype=float)
    clamped = np.clip(input_pos, min_deg, max_deg)
    normed = (clamped-min_deg) / (max_deg - min_deg)

    # compensation of MCP movement
    normed[9]  = normed[9]  - (40/110) * normed[10]
    normed[0]  = normed[0]  - (40/110) * normed[2]
    normed[5]  = normed[5]  - (40/110) * normed[4]
    normed[8]  = normed[8]  - (40/110) * normed[6]

    positions = normed * (hand.curled_bound - hand.tensioned_pos) + hand.tensioned_pos
    return positions

###########################################
# DEMO ROUTINES
###########################################

def go(hand, target_deg, dur=50):
    """Wrapper to normalize + send a motor command."""
    curr = hand.read_pos()
    des  = normalize_to_motor(target_deg, hand)
    move_to_pos(curr_pos=curr, des_pos=des, hand=hand, traj_len=dur)
    
def wrist_lr(hand, pos, dur=50):
    """Move only the wrist motor (motor 15)."""
    curr = hand.read_pos()
    des  = curr.copy()
    des[14] = pos
    move_to_pos(curr_pos=curr, des_pos=des, hand=hand, traj_len=dur)
    
def wrist_fwd_back(hand, pos, dur=50):
    """Move only the wrist motor (motor 16)."""
    curr = hand.read_pos()
    des  = curr.copy()
    des[15] = pos
    move_to_pos(curr_pos=curr, des_pos=des, hand=hand, traj_len=dur)
    
def wrist_center(hand, dur=50):
    """Center the wrist motors."""
    curr = hand.read_pos()
    des  = curr.copy()
    des[14] = 2048
    des[15] = 2048
    move_to_pos(curr_pos=curr, des_pos=des, hand=hand, traj_len=dur)
    
def wrist_demo():
    wrist_center(hand); pause(0.5)
    
    wrist_fwd_back(hand, 2400); pause(0.5)
    wrist_lr(hand, 1800); pause(0.5)
    wrist_lr(hand, 2300); pause(0.5)
    
    wrist_lr(hand, 2048); pause(0.5)
    wrist_fwd_back(hand, 1800); pause(0.5)
    wrist_lr(hand, 1800); pause(0.5)
    wrist_lr(hand, 2300); pause(0.5)
    
    wrist_center(hand); pause(0.5)
    
def splay_demo():
    open_pose = [0]*16
    splay_demo = [0, 20, 0, 20, 0, 0, 0, 40, 0, 0, 0, 0, 0, 90, 0, 0]
    
    go(hand, open_pose); pause(0.1)
    go(hand, splay_demo); pause(0.1)
    go(hand, open_pose); pause(0.1)
    go(hand, splay_demo); pause(0.1)
    go(hand, open_pose); pause(0.1)
    
def thumb_demo():
    open_pose = [0]*16
    pose_1 = [0]*16
    pose_1[11] = 80
    pose_1[13] = 90
    
    pose_2 = [0]*16
    pose_2[12] = 150
    
    pose_3 = pose_2.copy()
    pose_3[11] = 80
    pose_3[13] = 90
    
    go(hand, open_pose); pause(0.1)
    go(hand, pose_1); pause(0.1)
    go(hand, open_pose); pause(0.1)
    
    go(hand, pose_2); pause(0.1)
    
    go(hand, pose_3); pause(0.1)
    go(hand, open_pose); pause(0.1)
    
def index_demo():
    open_pose = [0]*16
    pose_1 = [0]*16
    pose_1[1] = 5
    pose_1[0] = 80
    
    pose_2 = [0]*16
    pose_2[1] = 40
    pose_2[0] = 90
    
    pose_3 = [0]*16
    pose_3[1] = 5
    pose_3[2] = 80
    
    pose_4 = [0]*16
    pose_4[1] = 5
    pose_4[2] = 80
    pose_4[0] = 60
    
    
    go(hand, open_pose); pause(0.5)
    go(hand, pose_1); pause(0.1)
    go(hand, pose_2); pause(0.1)
    go(hand, pose_1); pause(0.1)
    go(hand, pose_3); pause(0.1)
    go(hand, pose_4); pause(0.1)
    go(hand, pose_3); pause(0.1)
    go(hand, open_pose); pause(0.1)
    
def rock_pose():
    open_pose = [0]*16
    pose_1 = [0]*16
    pose_1[3] = 20
    pose_1[4] = 80
    pose_1[5] = 40
    pose_1[10] = 85
    pose_1[9] = 40
    
    pose_1[11] = 80
    pose_1[13] = 60
    pose_1[12] = 120
    
    pose_2 = pose_1.copy()
    pose_2[1] = 20
    pose_2[7] = 40
    
    go(hand, pose_1); pause(0.5)
    go(hand, pose_2); pause(0.1)
    go(hand, pose_1); pause(0.1)
    go(hand, pose_2); pause(0.1)
    go(hand, open_pose); pause(1.0)
    
    


def pause(sec=1.0):
    """Simple pause."""
    time.sleep(sec)


###########################################
# DEMO SEQUENCE
###########################################

def demo(hand):

    print("\n=== DEMO: ROBOT HAND MOVEMENT ===")
    
    pause(5.0)
    wrist_demo()
    splay_demo()
    thumb_demo()
    index_demo()
    rock_pose()

    print("\n=== DEMO DONE ===")


###########################################
# MAIN
###########################################

if __name__ == "__main__":
    hand = Hand("right")
    try:
        demo(hand)
    finally:
        hand.close()