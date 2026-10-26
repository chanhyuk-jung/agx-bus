# Agx Bus: Bus interface for AgileX Robotics arms

Agx Bus provides an easy way to use robot arms and grippers from AgileX Robotics. We vendor [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm.git) for maximum comparibility and provide a bus-like api for sending and receiving data to all joints.

## Installation

agx-bus can be installed from pip

```bash
pip install agx-bus
```

or from source to get the latest version of pyAgxArm

```bash
git clone https://github.com/chanhyuk-jung/agx-bus.git
cd agx-bus

git remote add pyAgxArm https://github.com/agilexrobotics/pyAgxArm.git
bash update.sh

pip install -e .
```

# Quick start

```python
from agx_bus import AgilexBus, AgilexGripper

bus = AgilexBus("can0", name="piper", version="default")
ee = AgilexGripper.attach_to(bus)

bus.connect()
bus.enable_torque()

bus.move_joints([0] * 6)
ee.move(width=0.1)

joints = bus.read_joints()
gripper = ee.read()

qpos = joints.position
qvel = joints.velocity

width = gripper.width
force = gripper.force

print(f"arm: {qpos=}, {qvel=}")
print(f"gripper: {width=}, {force=}")
```
