# TestGo SDK v6
from booster_robotics_sdk_python import (
    B1LocoClient, ChannelFactory,
    B1HandIndex, Position, Orientation, Posture, RobotMode,
)
from time import sleep
import signal

ChannelFactory.Instance().Init(0)
client = B1LocoClient()
client.Init()
sleep(1.0)

def _on_stop(sig, frame):
    for _ in range(5):
        client.Move(0.0, 0.0, 0.0)
        sleep(0.2)
    raise SystemExit(0)

signal.signal(signal.SIGTERM, _on_stop)
signal.signal(signal.SIGINT, _on_stop)

def _wave_hand(hand, raise_z=0.20, lower_z=0.05, reps=3):
    s = 1 if hand == B1HandIndex.kLeftHand else -1
    for _ in range(30):
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)
    _p = Posture()
    _p.orientation = Orientation(0.0, 0.0, 0.0)
    _p.position = Position(0.25, s * 0.30, raise_z)
    client.MoveHandEndEffector(_p, 1500, hand)
    sleep(1.5)
    for _ in range(reps):
        _p.position = Position(0.25, s * 0.38, raise_z)
        client.MoveHandEndEffector(_p, 500, hand)
        sleep(0.5)
        _p.position = Position(0.25, s * 0.22, raise_z)
        client.MoveHandEndEffector(_p, 500, hand)
        sleep(0.5)
    _p.position = Position(0.28, s * 0.25, lower_z)
    client.MoveHandEndEffector(_p, 1500, hand)
    sleep(1.5)

def _move_head(pitch=0.0, yaw=0.0, hold=1.5):
    client.RotateHead(pitch, yaw)
    sleep(hold)
    client.RotateHead(0.0, 0.0)
    sleep(0.5)

def _wave_both_hands(raise_z=0.20, lower_z=0.05, reps=3):
    import threading
    tl = threading.Thread(target=_wave_hand, args=(B1HandIndex.kLeftHand,), kwargs={"raise_z": raise_z, "lower_z": lower_z, "reps": reps})
    tr = threading.Thread(target=_wave_hand, args=(B1HandIndex.kRightHand,), kwargs={"raise_z": raise_z, "lower_z": lower_z, "reps": reps})
    tl.start(); tr.start()
    tl.join(); tr.join()

# ───────────────────────────────────────────────────────
# ⚠  DO NOT EDIT above this line — auto-generated header
# ───────────────────────────────────────────────────────
while True:
    pass  # ← drag a function here
 
  # rotate clockwise for 2.0s
    for _ in range(20):
        client.Move(0.0, 0.0, -0.2)
        sleep(0.1)
    for _ in range(10):  # decelerate
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)


    # head rotate anti-clockwise (left)
    _move_head(yaw=0.785, hold=1.5)

    


    # head up
    _move_head(pitch=-0.3, hold=1.5)



    # head down
    _move_head(pitch=1.0, hold=1.5)

  
    # head rotate clockwise (right)
    _move_head(yaw=-0.785, hold=1.5)


    # walk forward for 2.0s
    for _ in range(20):
        client.Move(0.8, 0.0, 0.0)
        sleep(0.1)
    for _ in range(10):  # decelerate
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)



    # rotate anti-clockwise for 2.0s
    for _ in range(20):
        client.Move(0.0, 0.0, 0.3)
        sleep(0.1)
    for _ in range(10):  # decelerate
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)


    # wave left hand
    _wave_hand(B1HandIndex.kLeftHand, raise_z=0.20, lower_z=0.05, reps=2)


    # wave right hand
    _wave_hand(B1HandIndex.kRightHand, raise_z=0.20, lower_z=0.05, reps=2)


    # wave both hands simultaneously
    _wave_both_hands(raise_z=0.20, lower_z=0.05, reps=2)
