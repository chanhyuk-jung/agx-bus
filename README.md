# Agx Bus: Bus interface for AgileX Robotics arms

Agx Bus is a library for robot arms and grippers from AgileX Robotics. By vendoring [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm.git), bug-fixes and features from first-party drivers are supported with a much simpler API.

**Why `agx_bus`?**

1. **Enforced correctness.** If there aren't any errors, your code will always work as expected. `agx_bus` raises errors when the desired behavior is not achievable.

2. **Automatic polling.** Unlike `piper_sdk` or `pyAgxArm`, you don't have to guess whether data is available. `agx_bus` either raises an error or returns a value. Polling is done internally so you don't have to.

3. **Synchronization.** `agx_bus` synchronizes reads across joints as tightly as possible, which gives a more accurate state of the robot arm.

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
```

Make sure to support the arm when you disable torque with `bus.disable_torque()` to prevent damage to the robot arm:

```python
bus.disable_torque()
```

The gripper can be used without enabling torque:

```python
bus.disable_torque()
ee.move(width=0.1)
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

You can also reset the gripper:

```python
ee.reset()
```

### Diagnostics

When something goes wrong, read the internals first:

```python
while True:
    status = bus.read_status()
    motor_status = bus.read_motor_status()
    gripper = ee.read()

    print(f"arm is ok: {status.is_arm_ok}")
    print(f"comm is ok: {status.is_comm_ok}")
    print(f"limit exceeded: {status.limit_exceeded}")
    print(f"motor is ok: {motor_status.is_motor_ok}")
    print(f"driver is ok: {motor_status.is_driver_ok}")

    print(f"gripper motor is ok: {gripper.status.is_motor_ok()}")
    print(f"gripper driver is ok: {gripper.status.is_driver_ok()}\n")

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

This project is licensed under the LGPL-3.0 License - see the [LICENSE](LICENSE) file for details.
