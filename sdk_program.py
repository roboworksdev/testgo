from booster_robotics_sdk_python import B1LocoClient, ChannelFactory
from time import sleep
import signal

ChannelFactory.Instance().Init(0)
client = B1LocoClient()
client.Init()
sleep(1.0)

def _on_stop(sig, frame):
    client.Move(0.0, 0.0, 0.0)
    sleep(2.0)
    raise SystemExit(0)

signal.signal(signal.SIGTERM, _on_stop)
signal.signal(signal.SIGINT, _on_stop)

print(client.Move(0.0, 0.0, 0.3))  # rotate anti-clockwise

while True:
    sleep(0.1)
    
