import pytest

from agx_bus import (
    AgilexBus,
    AgilexGripper,
    BusNotConnectedError,
    TorqueNotEnabledError,
)


@pytest.fixture
def bus():
    bus = AgilexBus("virtual-can", interface="virtual", name="piper", version="default")

    return bus


@pytest.fixture
def ee(bus):
    return AgilexGripper(bus)


def test_not_connected_enable_torque(bus):
    with pytest.raises(BusNotConnectedError):
        bus.enable_torque()


def test_not_connected_read_joints(bus):
    with pytest.raises(BusNotConnectedError):
        bus.read_joints()


def test_not_connected_read_state(bus):
    with pytest.raises(BusNotConnectedError):
        bus.read_state()


def test_not_connected_read_drivers(bus):
    with pytest.raises(BusNotConnectedError):
        bus.read_drivers()


def test_not_connected_reset(bus):
    with pytest.raises(BusNotConnectedError):
        bus.reset()


def test_not_connected_e_stop(bus):
    with pytest.raises(BusNotConnectedError):
        bus.e_stop()


def test_disabled_torque_move_joints(bus):
    bus.is_connected = True

    with pytest.raises(TorqueNotEnabledError):
        bus.move_joints([0.0] * 6)


def test_not_connected_move_gripper(ee):
    with pytest.raises(BusNotConnectedError):
        ee.move_gripper(width=0.1)


def test_not_connected_read_gripper(ee):
    with pytest.raises(BusNotConnectedError):
        ee.read_gripper()


def test_not_connected_gripper_read_state(ee):
    with pytest.raises(BusNotConnectedError):
        ee.read_driver()
