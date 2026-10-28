import pytest

from agx_bus import AgilexBus, BusNotConnectedError, TorqueNotEnabledError


@pytest.fixture
def bus():
    bus = AgilexBus("virtual-can", interface="virtual", name="piper", version="default")

    return bus

def test_not_connected_enable_torque(bus):
    with pytest.raises(BusNotConnectedError):
        bus.enable_torque()

def test_not_connected_read_joints(bus):
    with pytest.raises(BusNotConnectedError):
        bus.read_joints()

def test_not_connected_read_status(bus):
    with pytest.raises(BusNotConnectedError):
        bus.read_status()

def test_not_connected_read_motor_status(bus):
    with pytest.raises(BusNotConnectedError):
        bus.read_motor_status()

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
