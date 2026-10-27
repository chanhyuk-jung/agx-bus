import time

import numpy as np

from agx_bus import AgilexBus, AgilexGripper


def test_piper():
    bus = AgilexBus("can0", name="piper", version="default")
    ee = AgilexGripper.attach_to(bus)

    bus.connect()

    bus.enable_torque()

    bus.e_stop()

    # wait for robot to fall down safely
    time.sleep(1)

    bus.reset()

    # wait for arm to reset
    time.sleep(1)

    bus.connect()

    bus.enable_torque()

    bus.move_joints([0.0] * 6)

    # wait for motors
    time.sleep(1)

    qpos = np.array([0.0] * 6)
    width = 0.05

    bus.move_joints(qpos)
    ee.move(width=width)

    joints = bus.read_joints()
    gripper = ee.read()

    curr_qpos = joints.position
    curr_width = gripper.width

    np.testing.assert_allclose(curr_qpos, qpos, atol=0.01)
    np.testing.assert_allclose(curr_width, width, atol=0.005)

    status = bus.read_status()
    motor_status = bus.read_motor_status()
    gripper = ee.read()

    assert all(
        [
            status.is_arm_ok,
            status.is_comm_ok,
            motor_status.is_motor_ok,
            motor_status.is_driver_ok,
            gripper.status.is_motor_ok,
            gripper.status.is_driver_ok,
        ]
    )

    assert not status.limit_exceeded
