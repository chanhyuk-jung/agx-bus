import time

import numpy as np
import pytest

from agx_bus import AgilexBus, AgilexGripper
from agx_bus.utils import poll_until

pytestmark = pytest.mark.piper

NORMAL = 0x00
EMERGENCY_STOP = 0x01

REACHED = 0x00


@pytest.fixture(scope="module")
def bus(channel, name, version):
    bus = AgilexBus(channel, name=name, version=version)

    bus.connect()

    yield bus

    bus.robot.electronic_emergency_stop()

    time.sleep(2)

    bus.robot.reset()

    time.sleep(1)

    bus.robot.set_speed_percent(100)
    bus.robot.set_speed_percent(100)

    bus.disconnect()


@pytest.fixture
def ee(bus):
    return AgilexGripper.attach_to(bus)


@pytest.fixture
def robot(bus):
    robot = bus.robot

    yield robot


@pytest.fixture
def end_effector(ee):
    return ee.end_effector


def test_torque_enable(bus, robot):
    bus.enable_torque()

    for i in range(6):

        def read():
            return robot.get_driver_states(i + 1)

        ds = poll_until(read, timeout=1.0, wait=0)

        torque_enabled = ds.msg.foc_status.driver_enable_status

        assert torque_enabled is True

def test_move_joints(bus, robot):
    test_torque_enable(bus, robot)

    qpos = np.array([0.0] * 6)

    bus.move_joints(qpos)
    time.sleep(0.05)

    def reached():
        return (
            poll_until(robot.get_arm_status, timeout=1.0, wait=0).msg.motion_status
            == REACHED
        )

    while not poll_until(reached, timeout=4.0, wait=0):
        time.sleep(0.1)

    ja = poll_until(robot.get_joint_angles, timeout=1.0, wait=0)
    joints = ja.msg

    np.testing.assert_allclose(joints, qpos, atol=0.01)


def test_move_gripper(ee, end_effector):
    width = 0.05

    ee.move(width=width)
    time.sleep(0.05)

    gs = poll_until(end_effector.get_gripper_status, timeout=1.0, wait=0)

    assert gs.msg.mode == "width"
    np.testing.assert_allclose(gs.msg.value, width, atol=0.001)


def test_estop(bus, robot):
    bus.e_stop()
    time.sleep(2)

    status = poll_until(robot.get_arm_status, timeout=1.0, wait=0)
    arm_status = status.msg.arm_status
    assert arm_status == EMERGENCY_STOP


def test_reset(bus, robot):
    bus.reset()

    bus.connect()

    status = poll_until(robot.get_arm_status, timeout=1.0, wait=0)
    arm_status = status.msg.arm_status

    assert arm_status == NORMAL

    test_move_joints(bus, robot)
