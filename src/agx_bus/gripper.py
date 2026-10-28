from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from .bus import AgilexBus
from .pyAgxArm.protocols.can_protocol.drivers import AgxGripperDriverDefault
from .utils import BusNotConnectedError, poll_until


@dataclass(frozen=True)
class Status:
    low_voltage: bool
    motor_overheating: bool
    overcurrent: bool
    overheating: bool
    sensor: bool
    error: bool
    torque_enabled: bool
    homing: bool

    @property
    def is_driver_ok(self) -> bool:
        return not any(
            [
                self.low_voltage,
                self.overcurrent,
                self.overheating,
                self.sensor,
                self.error,
            ]
        )

    @property
    def is_motor_ok(self) -> bool:
        return not self.motor_overheating


@dataclass(frozen=True)
class Gripper:
    timestamp: float

    value: float
    force: float

    mode: Literal["width", "angle"]

    status: Status

    @property
    def width(self) -> float:
        if self.mode == "width":
            return self.value
        else:
            raise ValueError

    @property
    def angle(self) -> float:
        if self.mode == "angle":
            return self.value
        else:
            raise ValueError


class AgilexGripper:
    def __init__(self, end_effector: AgxGripperDriverDefault, *, timeout: float = 1.0):
        self.end_effector = end_effector

        self.torque_enabled = False
        self.timeout = timeout

    @property
    def is_connected(self):
        return self.end_effector._ctx.get_comm() is not None

    @classmethod
    def attach_to(cls, bus: AgilexBus):
        end_effector = bus.robot.init_effector("agx_gripper")
        return cls(end_effector, timeout=bus.timeout)

    def disable_torque(self):
        if not self.is_connected:
            raise BusNotConnectedError

        poll_until(self.end_effector.disable_gripper, timeout=self.timeout, wait=0.01)

    def move(
        self,
        *,
        width: float | None = None,
        angle: float | None = None,
        force: float = 1.0,
    ) -> None:
        if not self.is_connected:
            raise BusNotConnectedError

        if width is not None:
            self.end_effector.move_gripper_m(width, force)
        elif angle is not None:
            self.end_effector.move_gripper_deg(angle, force)
        else:
            raise TypeError("either width or angle must be specified")

    def read(self) -> Gripper:
        if not self.is_connected:
            raise BusNotConnectedError

        def read():
            gs = self.end_effector.get_gripper_status()

            if gs is None:
                return None

            msg = gs.msg

            mode = msg.mode
            if mode not in ["width", "angle"]:
                raise RuntimeError(f"Unexpected mode: {mode}")

            status = msg.foc_status

            status = Status(
                low_voltage=status.voltage_too_low,
                motor_overheating=status.motor_overheating,
                overcurrent=status.driver_overcurrent,
                overheating=status.driver_overheating,
                sensor=status.sensor_status,
                error=status.driver_error_status,
                torque_enabled=status.driver_enable_status,
                homing=status.homing_status,
            )

            return Gripper(
                timestamp=gs.timestamp,
                value=msg.value,
                force=msg.force,
                mode=mode,
                status=status,
            )

        return poll_until(read, timeout=self.timeout, wait=0)

    def set_zero(self):
        if not self.is_connected:
            raise BusNotConnectedError

        self.end_effector.calibrate_gripper(timeout=self.timeout)

    def clear_errors(self):
        if not self.is_connected:
            raise BusNotConnectedError

        poll_until(self.end_effector.reset_gripper, timeout=self.timeout, wait=0.01)
