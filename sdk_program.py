# TestGo SDK v4
from booster_robotics_sdk_python import B1LocoClient, ChannelFactory
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

def do_move(vx, vy, vz, duration=2.0):
    steps = max(1, int(duration / 0.1))
    for _ in range(steps):
        client.Move(vx, vy, vz)
        sleep(0.1)

while True:
    pass  # ← drag a function here
    do_move(0.0, 0.0, -0.2, 4.0)  # rotate clockwise for 2.0s
    do_move(0.0, 0.0,  0.3, 2.0)  # rotate anti-clockwise for 2.0s
