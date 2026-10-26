# Agx Bus: Bus interface for AgileX Robotics arms

Agx Bus provides an easy way to use robot arms and grippers from AgileX Robotics. We vendor [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm.git) for maximum comparibility and provide a bus-like api for sending and receiving data to all joints.

## Quick Start

Create the bus and attach a gripper:

```python
from agx_bus import AgilexBus, AgilexGripper

bus = AgilexBus("can0", name="piper", version="default")
ee = AgilexGripper.attach_to(bus)
```

Continue reading to learn about most of the features of this package.

### Reading

Connect to a CAN bus and start reading:

```python
import time

bus.connect()

while True:
    joints = bus.read_joints()
    gripper = ee.read()

    qpos = joints.position
    qvel = joints.velocity

    width = gripper.width
    force = gripper.force

    print(f"arm: {qpos=}, {qvel=}")
    print(f"gripper: {width=}, {force=}\n")

    time.sleep(0.1)
```

### Moving

Enable torque to start moving:

```python
bus.enable_torque()

bus.move_joints([0] * 6)
ee.move(width=0.1)
```

Make sure to support the arm when you disable torque with `bus.disable_torque()` to prevent damage to the robot arm:

```python
bus.disable_torque()
```

### Emergency Stop

If you need to quickly stop the robot, trigger e-stop:

```python
bus.e_stop()
```

### Reset

Make sure to reset the arm to clear e-stop or any other errors:

```python
bus.reset()
```

### Diagnostics

When something goes wrong, read the internals first:

```python
while True:
    status = bus.read_status()
    motor_status = bus.read_motor_status()
    gripper = ee.read()

    print(f"arm is ok: {status.arm_ok()}")
    print(f"comm is ok: {status.comm_ok()}")
    print(f"limit exceeded: {status.limit_exceeded()}")
    print(f"motor is ok: {motor_status.motor_ok()}")
    print(f"driver is ok: {motor_status.driver_ok()}")
    print(f"gripper is ok: {gripper.status.is_ok()}\n")

    time.sleep(0.1)
```

## Installation

Install agx-bus from pypi

```bash
pip install agx-bus
```

or from source to get the latest version of pyAgxArm

```bash
git clone https://github.com/chanhyuk-jung/agx-bus.git
cd agx-bus

bash update.sh

pip install -e .
```

## License
