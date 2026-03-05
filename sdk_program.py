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

while True:
    pass  # ← drag a function here
  

# walk backward for 2.0s
    for _ in range(60):
        client.Move(-0.2, 0.0, 0.0)
        sleep(0.1)
    for _ in range(10):  # decelerate
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)
    # stop and stabilize before arm control
    for _ in range(15):
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)

    
   
    # rotate clockwise for 2.0s
    for _ in range(50):
        client.Move(0.0, 0.0, -0.2)
        sleep(0.1)
    for _ in range(10):  # decelerate
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)



    # rotate anti-clockwise for 2.0s
    for _ in range(80):
        client.Move(0.0, 0.0, 0.3)
        sleep(0.1)
    for _ in range(30):  # decelerate
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)


         # stop and stabilise — stay in kWalking, no mode change
    for _ in range(30):
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)
    # V1 arm in kWalking mode — exact hand-down position from official demo
    # (mirrored for left hand: right was Position(0.28,-0.25,0.05) Orientation(0,0,0))
    _p = Posture()
    _p.orientation = Orientation(0.0, 0.0, 0.0)
    _p.position = Position(0.28, 0.25, 0.05)
    client.MoveHandEndEffector(_p, 1500, B1HandIndex.kLeftHand)
    sleep(1.5)
    for _ in range(3):
        _p.position = Position(0.28, 0.35, 0.05)  # swing out
        client.MoveHandEndEffector(_p, 600, B1HandIndex.kLeftHand)
        sleep(0.6)
        _p.position = Position(0.28, 0.15, 0.05)  # swing in
        client.MoveHandEndEffector(_p, 600, B1HandIndex.kLeftHand)
        sleep(0.6)
    _p.position = Position(0.28, 0.25, 0.05)
    client.MoveHandEndEffector(_p, 1500, B1HandIndex.kLeftHand)
    sleep(1.5)
