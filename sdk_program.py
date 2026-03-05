# TestGo SDK v6
from booster_robotics_sdk_python import (
    B1LocoClient, ChannelFactory,
    B1HandIndex, Position, Orientation, Posture, RobotMode,
    DanceId, WholeBodyDanceId,
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

def _wave_hand(hand, raise_z=0.20, lower_z=0.05, reps=2):
    s = 1 if hand == B1HandIndex.kLeftHand else -1
    for _ in range(10):
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)
    _p = Posture()
    _p.orientation = Orientation(0.0, 0.0, 0.0)
    _p.position = Position(0.25, s * 0.30, raise_z)
    client.MoveHandEndEffector(_p, 1000, hand)
    sleep(1.0)
    for _ in range(reps):
        _p.position = Position(0.25, s * 0.38, raise_z)
        client.MoveHandEndEffector(_p, 500, hand)
        sleep(0.5)
        _p.position = Position(0.25, s * 0.22, raise_z)
        client.MoveHandEndEffector(_p, 500, hand)
        sleep(0.5)
    _p.position = Position(0.28, s * 0.25, lower_z)
    client.MoveHandEndEffector(_p, 1000, hand)
    sleep(1.0)

def _move_head(pitch=0.0, yaw=0.0, hold=1.5):
    client.RotateHead(pitch, yaw)
    sleep(hold)
    client.RotateHead(0.0, 0.0)
    sleep(0.5)

def _wave_both_hands(raise_z=0.20, lower_z=0.05, reps=2):
    for _ in range(10):
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)
    _pl = Posture(); _pl.orientation = Orientation(0.0, 0.0, 0.0)
    _pr = Posture(); _pr.orientation = Orientation(0.0, 0.0, 0.0)
    _pl.position = Position(0.25,  0.30, raise_z)
    _pr.position = Position(0.25, -0.30, raise_z)
    client.MoveHandEndEffector(_pl, 1000, B1HandIndex.kLeftHand)
    client.MoveHandEndEffector(_pr, 1000, B1HandIndex.kRightHand)
    sleep(1.0)
    for _ in range(reps):
        _pl.position = Position(0.25,  0.38, raise_z)
        _pr.position = Position(0.25, -0.38, raise_z)
        client.MoveHandEndEffector(_pl, 500, B1HandIndex.kLeftHand)
        client.MoveHandEndEffector(_pr, 500, B1HandIndex.kRightHand)
        sleep(0.5)
        _pl.position = Position(0.25,  0.22, raise_z)
        _pr.position = Position(0.25, -0.22, raise_z)
        client.MoveHandEndEffector(_pl, 500, B1HandIndex.kLeftHand)
        client.MoveHandEndEffector(_pr, 500, B1HandIndex.kRightHand)
        sleep(0.5)
    _pl.position = Position(0.28,  0.25, lower_z)
    _pr.position = Position(0.28, -0.25, lower_z)
    client.MoveHandEndEffector(_pl, 1000, B1HandIndex.kLeftHand)
    client.MoveHandEndEffector(_pr, 1000, B1HandIndex.kRightHand)
    sleep(1.0)

# Dance IDs (pass as dance_id argument to _dance):
#   DanceId.kNewYear          — New Year dance
#   DanceId.kNezha            — Nezha dance
#   DanceId.kTowardsFuture    — Towards Future dance
#   DanceId.kDabbingGesture   — Dabbing gesture
#   DanceId.kUltramanGesture  — Ultraman gesture
#   DanceId.kRespectGesture   — Respect gesture
#   DanceId.kCheeringGesture  — Cheering gesture
#   DanceId.kLuckyCatGesture  — Lucky Cat gesture
# Note: robot must remain in kWalking mode — do NOT call ChangeMode before dance.
def _dance(dance_id=DanceId.kNewYear, duration=8.0):
    # Stop and stabilise before dancing (must stay in kWalking mode)
    for _ in range(10):
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)
    client.Dance(dance_id)
    sleep(duration)
    client.Dance(DanceId.kStop)
    sleep(0.5)

# Whole Body Dance IDs (pass as dance_id argument to _whole_body_dance):
#   WholeBodyDanceId.kArbicDance      — Arabic dance
#   WholeBodyDanceId.kMichaelDance1   — Michael Jackson move 1
#   WholeBodyDanceId.kMichaelDance2   — Michael Jackson move 2
#   WholeBodyDanceId.kMichaelDance3   — Michael Jackson move 3
#   WholeBodyDanceId.kMoonWalk        — Moonwalk
#   WholeBodyDanceId.kBoxingStyleKick — Boxing style kick
#   WholeBodyDanceId.kRoundhouseKick  — Roundhouse kick
# Note: robot must remain in kWalking mode — do NOT call ChangeMode before dance.
def _whole_body_dance(dance_id=WholeBodyDanceId.kArbicDance, duration=10.0):
    # Stop and stabilise before dancing (must stay in kWalking mode)
    for _ in range(10):
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)
    client.WholeBodyDance(dance_id)
    sleep(duration)

# ───────────────────────────────────────────────────────
# ⚠  DO NOT EDIT above this line — auto-generated header
# ───────────────────────────────────────────────────────
while True:
    pass  # ← drag a function here
    # dance — robot must be in kWalking mode (default on startup)
    # kNewYear / kNezha / kTowardsFuture / kDabbingGesture /
    # walk forward for 2.0s
    for _ in range(20):
        client.Move(0.8, 0.0, 0.0)
        sleep(0.1)
    for _ in range(10):  # decelerate
        client.Move(0.0, 0.0, 0.0)
        sleep(0.1)
    # whole body dance — robot must be in kWalking mode (default on startup)
    # kUltramanGesture / kRespectGesture / kCheeringGesture / kLuckyCatGesture
    _dance(DanceId.kNewYear)
    # change WholeBodyDanceId to try a different dance:
    # kArbicDance / kMichaelDance1 / kMichaelDance2 / kMichaelDance3 /
    # kMoonWalk / kBoxingStyleKick / kRoundhouseKick
    _whole_body_dance(WholeBodyDanceId.kArbicDance)
