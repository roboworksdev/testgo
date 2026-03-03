from booster_robotics_sdk_python import B1LocoClient,ChannelFactory
from time import sleep

ChannelFactory.Instance().Init(0)
client = B1LocoClient()
client.Init()
sleep(1.0)
print(client.Move(0.0,0.0,0.3)) # Rotate anti clockwise


